#!/usr/bin/env python3
"""World Composer — generate sim/worlds/patrol_world.sdf from sim/config/checkpoints.yaml (M5).

Resolves OQ-3 (generate-from-YAML, not static SDF): checkpoint positions live in ONE place
(checkpoints.yaml, Tenet 2), and the loadable world is generated from it by injecting one AprilTag
``<include>`` per checkpoint into the hand-authored template at the ``<!-- CHECKPOINT_MARKERS -->``
placeholder. ``check_drift()`` (run in CI) asserts the committed world still matches the YAML, so the
two cannot silently diverge (INF-S2).

Pure stdlib — no ``pyyaml`` / ``uv`` / ROS dependency — so the CI ``world-drift`` gate runs on the
runner's system ``python3`` with zero setup, exactly like the ``manifest-drift`` gate. The checkpoint
reader is a focused, FAIL-LOUD parser for the canonical 03 schema (keyed ``checkpoints:`` mapping or a
bare list; ``position`` as an inline flow mapping ``{x: .., y: .., z: ..}``). Anything it cannot parse
unambiguously raises rather than silently misreading (INF-S3).

Guards (all fail loud, no partial/broken world is ever written):
  - every checkpoint carries checkpoint_id, position{x,y,z}, tag_family, tag_id
  - position values are numeric
  - no duplicate tag_id and no duplicate checkpoint_id
  - each tag_id has a matching sim/models/apriltag_36h11_<id> directory
  - every emitted <uri> is model:// or repo-relative (never an absolute/host path) — SIM-6

Usage:
    python3 sim/tools/compose_world.py            # regenerate sim/worlds/patrol_world.sdf
    python3 sim/tools/compose_world.py --check    # fail if the committed world drifted from the YAML
"""

from __future__ import annotations

import math
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = REPO_ROOT / "sim" / "config" / "checkpoints.yaml"
TEMPLATE_PATH = REPO_ROOT / "sim" / "worlds" / "patrol_world.template.sdf"
WORLD_PATH = REPO_ROOT / "sim" / "worlds" / "patrol_world.sdf"
MODELS_DIR = REPO_ROOT / "sim" / "models"

PLACEHOLDER = "<!-- CHECKPOINT_MARKERS -->"
MARKER_NAME_PREFIX = "checkpoint_"
# checkpoint_id is f-string-interpolated into <name>checkpoint_{id}</name> in the world SDF and into
# the model:// namespace, so it must be inert: alphanumerics, '_' and '-' only (no XML metacharacters,
# whitespace, or '#'). Fail loud on anything else (F-01) - the same fail-loud bar as config.py.
_CHECKPOINT_ID_RE = re.compile(r"^[A-Za-z0-9_-]+$")
# A checkpoint entry carries exactly these keys — reject extras fail-loud (e.g. a stray waypoint field
# like dwell_s, which belongs in 02's mission YAML, not here).
_ALLOWED_ENTRY_KEYS = frozenset({"checkpoint_id", "position", "tag_family", "tag_id"})
# The only AprilTag family generated today (gen_apriltag_models) — M5 is single-family.
SUPPORTED_TAG_FAMILY = "tag36h11"
_POSE_TOL = 1e-6
_XYZ = 3  # x, y, z position axes
# Canonical world-design invariants (checkpoints.yaml header / OQ-6) — enforced on the canonical
# config only, by validate_world_design (the CI gate + local --check), never the unit fixtures.
_WORLD_HALF_EXTENT = 20.0  # |x|,|y| <= 20  → the ~40x40 m world
_MIN_SEPARATION = 8.0  # each checkpoint pair >= 8 m apart
_MIN_CHECKPOINTS = 3


class ComposeError(ValueError):
    """A fail-loud composer/guard violation (malformed config, missing model, drift)."""


@dataclass(frozen=True)
class Checkpoint:
    checkpoint_id: str
    x: float
    y: float
    z: float
    tag_family: str
    tag_id: int

    @property
    def model_name(self) -> str:
        return f"apriltag_36h11_{self.tag_id}"

    @property
    def marker_name(self) -> str:
        return f"{MARKER_NAME_PREFIX}{self.checkpoint_id}"

    @property
    def uri(self) -> str:
        return f"model://{self.model_name}"


# --- focused checkpoints.yaml reader (pure stdlib) -----------------------------------------------


def _strip_comment(line: str) -> str:
    """Drop a full-line or whitespace-preceded trailing ``# ...`` comment (our values carry no #)."""
    if re.match(r"\s*#", line):
        return ""
    return re.sub(r"\s+#.*$", "", line)


def _entry_field_lines(config_path: str, text: str) -> list[tuple[str, str, str]]:
    """Scan to (kind, key, value) tuples: kind 'item' starts an entry, 'field' extends it.

    Recognizes ``- key: value`` (a new list item) and ``  key: value`` (a field). The top-level
    ``checkpoints:`` header and blank lines are skipped. Anything else fails loud.
    """
    out: list[tuple[str, str, str]] = []
    for raw in text.splitlines():
        line = _strip_comment(raw).rstrip()
        if not line.strip() or line.strip() == "checkpoints:":
            continue
        if m := re.match(r"\s*-\s+(\w+):\s*(.*)$", line):
            out.append(("item", m.group(1), m.group(2).strip()))
        elif m := re.match(r"\s+(\w+):\s*(.*)$", line):
            out.append(("field", m.group(1), m.group(2).strip()))
        else:
            raise ComposeError(f"{config_path}: cannot parse line {line!r}")
    return out


def _group_entries(config_path: str, fields: list[tuple[str, str, str]]) -> list[dict[str, str]]:
    """Fold (kind, key, value) tuples into per-checkpoint ``{key: raw_value}`` dicts."""
    entries: list[dict[str, str]] = []
    for kind, key, value in fields:
        if kind == "item":
            entries.append({})
        elif not entries:
            raise ComposeError(f"{config_path}: field {key!r} appears before any '- ' checkpoint")
        if key in entries[-1]:
            raise ComposeError(f"{config_path}: duplicate field {key!r} in one checkpoint")
        entries[-1][key] = value
    return entries


def _parse_inline_mapping(value: str) -> dict[str, str]:
    """Parse ``{ x: 1.0, y: 2.0 }`` into ``{'x': '1.0', 'y': '2.0'}``; ``{}`` if not a flow mapping."""
    s = value.strip()
    if not (s.startswith("{") and s.endswith("}")):
        return {}
    out: dict[str, str] = {}
    for part in s[1:-1].split(","):
        key, sep, token = part.partition(":")
        if sep and key.strip():
            out[key.strip()] = token.strip()
    return out


def _numeric_token(token: str) -> float | None:
    """Strict float: reject trailing junk (``12abc``), multi-dot (``1.2.3``) and inf/nan; else None."""
    try:
        num = float(token)
    except ValueError:
        return None
    return num if math.isfinite(num) else None


def _parse_position(config_path: str, cid: str, value: str) -> tuple[float, float, float]:
    """Parse ``position: { x: .., y: .., z: .. }`` (inline flow mapping) into numeric (x, y, z)."""
    mapping = _parse_inline_mapping(value)
    coords = []
    for axis in ("x", "y", "z"):
        token = mapping.get(axis)
        num = _numeric_token(token) if token is not None else None
        if num is None:
            raise ComposeError(
                f"{config_path}: checkpoint {cid!r} position must be an inline flow mapping with "
                f"numeric x/y/z (e.g. '{{x: 1.0, y: 2.0, z: 1.5}}'); missing/invalid {axis!r} in {value!r}"
            )
        coords.append(num)
    return coords[0], coords[1], coords[2]


def _build_checkpoint(config_path: str, entry: dict[str, str]) -> Checkpoint:
    """Validate one raw entry dict into a :class:`Checkpoint` (fail loud on any missing field)."""
    for field in ("checkpoint_id", "position", "tag_family", "tag_id"):
        if field not in entry:
            raise ComposeError(f"{config_path}: a checkpoint is missing required field {field!r}")
    extra = set(entry) - _ALLOWED_ENTRY_KEYS
    if extra:
        raise ComposeError(
            f"{config_path}: checkpoint has unexpected field(s) {sorted(extra)}; "
            f"allowed {sorted(_ALLOWED_ENTRY_KEYS)} (waypoint fields belong in the mission YAML)"
        )
    cid = entry["checkpoint_id"].strip().strip("\"'")
    if not _CHECKPOINT_ID_RE.match(cid):
        raise ComposeError(
            f"{config_path}: checkpoint_id {cid!r} must match {_CHECKPOINT_ID_RE.pattern} "
            f"(letters, digits, '_' and '-' only) - it is interpolated into the world SDF"
        )
    x, y, z = _parse_position(config_path, cid, entry["position"])
    family = entry["tag_family"].strip().strip("\"'")
    if family != SUPPORTED_TAG_FAMILY:
        raise ComposeError(
            f"{config_path}: checkpoint {cid!r} tag_family {family!r} is unsupported; only "
            f"{SUPPORTED_TAG_FAMILY!r} is generated today (M5 single-family)"
        )
    try:
        tag_id = int(entry["tag_id"])
    except ValueError as exc:
        raise ComposeError(
            f"{config_path}: checkpoint {cid!r} tag_id must be an int, got {entry['tag_id']!r}"
        ) from exc
    return Checkpoint(cid, x, y, z, family, tag_id)


def load_checkpoints(config_path: str | Path = CONFIG_PATH) -> list[Checkpoint]:
    """Read + validate the canonical checkpoints file into ordered :class:`Checkpoint` records."""
    config_path = str(config_path)
    text = Path(config_path).read_text()
    fields = _entry_field_lines(config_path, text)
    entries = _group_entries(config_path, fields)
    if not entries:
        raise ComposeError(f"{config_path}: no checkpoints found")
    return [_build_checkpoint(config_path, e) for e in entries]


# --- guards --------------------------------------------------------------------------------------


def _check_unique(checkpoints: list[Checkpoint], config_path: str) -> None:
    for attr, label in (("tag_id", "tag_id"), ("checkpoint_id", "checkpoint_id")):
        seen: set = set()
        for cp in checkpoints:
            value = getattr(cp, attr)
            if value in seen:
                raise ComposeError(f"{config_path}: duplicate {label} {value!r}")
            seen.add(value)


def _check_model_dirs(checkpoints: list[Checkpoint], models_dir: Path) -> None:
    for cp in checkpoints:
        if not (models_dir / cp.model_name).is_dir():
            raise ComposeError(
                f"checkpoint {cp.checkpoint_id!r} references tag_id {cp.tag_id} but model dir "
                f"{(models_dir / cp.model_name)} is missing (run sim/tools/gen_apriltag_models.py)"
            )


def _check_uri_portable(uri: str) -> None:
    """Reject any non-portable <uri> (absolute path, host path, file:// URL) — SIM-6."""
    if uri.startswith("model://"):
        return
    if uri.startswith(("/", "~", "file://")) or re.match(r"[A-Za-z]:[\\/]", uri):
        raise ComposeError(
            f"non-portable <uri> {uri!r}: use model:// or a repo-relative path (SIM-6)"
        )
    if uri.startswith("../") or "/../" in uri:
        raise ComposeError(f"non-portable <uri> {uri!r}: escapes the repo (SIM-6)")


def _run_guards(checkpoints: list[Checkpoint], config_path: str, models_dir: Path) -> None:
    _check_unique(checkpoints, config_path)
    _check_model_dirs(checkpoints, models_dir)
    for cp in checkpoints:
        _check_uri_portable(cp.uri)


# --- world-design invariants (canonical config only) ---------------------------------------------
# The checkpoints.yaml header documents firm layout invariants (OQ-6) that describe the *shipped*
# world's design, not parser robustness. They are enforced here, by validate_world_design, only
# against the canonical config (the CI world-drift gate + local --check) - NOT inside _run_guards /
# check_drift, which the unit fixtures drive with deliberately smaller/edge-case worlds.


def _design_count(cps: list[Checkpoint], out: str, problems: list[str]) -> None:
    if len(cps) < _MIN_CHECKPOINTS:
        problems.append(
            f"{out}: only {len(cps)} checkpoint(s); the world requires >= {_MIN_CHECKPOINTS}"
        )


def _design_bounds(cps: list[Checkpoint], out: str, problems: list[str]) -> None:
    problems.extend(
        f"{out}: checkpoint {cp.checkpoint_id!r} at ({cp.x}, {cp.y}) is outside the "
        f"+/-{_WORLD_HALF_EXTENT} m world"
        for cp in cps
        if abs(cp.x) > _WORLD_HALF_EXTENT or abs(cp.y) > _WORLD_HALF_EXTENT
    )


def _design_spacing(cps: list[Checkpoint], out: str, problems: list[str]) -> None:
    for i in range(len(cps)):
        for b in cps[i + 1 :]:
            a = cps[i]
            d = math.dist((a.x, a.y), (b.x, b.y))
            if d >= _MIN_SEPARATION:
                continue
            problems.append(
                f"{out}: checkpoints {a.checkpoint_id!r} and {b.checkpoint_id!r} are "
                f"{d:.1f} m apart (< {_MIN_SEPARATION} m)"
            )


def _design_contiguous(cps: list[Checkpoint], out: str, problems: list[str]) -> None:
    ids = sorted(cp.tag_id for cp in cps)
    if ids != list(range(len(cps))):
        problems.append(f"{out}: tag_ids {ids} are not contiguous 0..{len(cps) - 1}")


def validate_world_design(config_path: str | Path = CONFIG_PATH) -> list[str]:
    """Return canonical world-design problems (empty == clean).

    Checks the layout invariants the checkpoints.yaml header documents (>= 3 checkpoints, each pair
    >= 8 m apart, all within the ~40x40 m world, contiguous tag_id 0..N-1) but the per-render guards
    do not enforce. Run only against the canonical config (CI gate + local --check), never the unit
    fixtures, which build smaller worlds on purpose.
    """
    cps = load_checkpoints(config_path)
    problems: list[str] = []
    for check in (_design_count, _design_bounds, _design_spacing, _design_contiguous):
        check(cps, str(config_path), problems)
    return problems


# --- emit ----------------------------------------------------------------------------------------


def _fmt(value: float) -> str:
    """Format a coordinate compactly and stably (drops a trailing .0-only? no — keep one decimal)."""
    return f"{value:g}"


def _include_block(cp: Checkpoint, indent: str) -> str:
    pose = f"{_fmt(cp.x)} {_fmt(cp.y)} {_fmt(cp.z)} 0 0 0"
    return (
        f"{indent}<include>\n"
        f"{indent}  <name>{cp.marker_name}</name>\n"
        f"{indent}  <uri>{cp.uri}</uri>\n"
        f"{indent}  <pose>{pose}</pose>   <!-- x y z roll pitch yaw, world/ENU -->\n"
        f"{indent}</include>"
    )


def _render_markers(checkpoints: list[Checkpoint], indent: str) -> str:
    return "\n".join(_include_block(cp, indent) for cp in checkpoints)


def render_world(
    config_path: str | Path = CONFIG_PATH,
    template_path: str | Path = TEMPLATE_PATH,
    models_dir: Path = MODELS_DIR,
) -> str:
    """Render the patrol world SDF text from checkpoints + template. Runs every guard; no write.

    The single source of truth for what the committed ``patrol_world.sdf`` must contain — both
    ``compose_world`` (which writes it) and ``check_drift`` (which verifies it) go through here, so the
    gate validates the *whole* contract (template body + markers + guards), not just marker positions.

    ``models_dir`` (default the live ``sim/models``) lets a test point the model-dir guard at a fixture
    instead of the committed tree, so isolated generated-asset tests need no monkeypatch.
    """
    checkpoints = load_checkpoints(config_path)
    _run_guards(checkpoints, str(config_path), models_dir)
    template = Path(template_path).read_text()
    placeholder = _find_placeholder_line(str(template_path), template)
    indent = placeholder[: len(placeholder) - len(placeholder.lstrip())]
    return template.replace(placeholder, _render_markers(checkpoints, indent), 1)


def compose_world(
    config_path: str | Path = CONFIG_PATH,
    template_path: str | Path = TEMPLATE_PATH,
    out_path: str | Path = WORLD_PATH,
    models_dir: Path = MODELS_DIR,
) -> str:
    """Generate the patrol world SDF from the checkpoints config. Returns the written text.

    Idempotent: always renders from the (placeholder-bearing) template, so re-running reproduces the
    same output.
    """
    world = render_world(config_path, template_path, models_dir)
    Path(out_path).write_text(world)
    return world


def _find_placeholder_line(template_path: str, template: str) -> str:
    """Return the full placeholder line (with its indentation).

    Fail loud if the token is absent, or if it appears more than once (e.g. echoed in a header
    comment) — injecting into the wrong occurrence would corrupt the SDF, so the token must appear
    exactly once, on its own line.
    """
    matches = [line for line in template.splitlines() if PLACEHOLDER in line]
    if not matches:
        raise ComposeError(f"{template_path}: missing the {PLACEHOLDER} placeholder")
    if len(matches) > 1:
        raise ComposeError(
            f"{template_path}: the {PLACEHOLDER} placeholder appears {len(matches)} times; "
            "it must appear exactly once on its own line"
        )
    return matches[0]


# --- drift check ---------------------------------------------------------------------------------


def _marker_from_include(
    inc: ET.Element, world_path: str
) -> tuple[str, str, tuple[float, float, float]] | None:
    """Parse one ``<include>`` -> (name, uri, (x, y, z)), or ``None`` if it's not a checkpoint."""
    name = (inc.findtext("name") or "").strip()
    if not name.startswith(MARKER_NAME_PREFIX):
        return None
    uri = (inc.findtext("uri") or "").strip()
    nums = [float(t) for t in (inc.findtext("pose") or "").split()[:_XYZ]]
    if len(nums) != _XYZ:
        raise ComposeError(f"{world_path}: include {name!r} has a malformed <pose>")
    return name, uri, (nums[0], nums[1], nums[2])


def _world_markers(world_path: str) -> dict[str, tuple[str, tuple[float, float, float]]]:
    """Parse the world SDF for ``checkpoint_*`` includes -> {marker_name: (uri, (x, y, z))}."""
    root = ET.fromstring(Path(world_path).read_text())
    markers: dict[str, tuple[str, tuple[float, float, float]]] = {}
    for inc in root.iter("include"):
        parsed = _marker_from_include(inc, world_path)
        if parsed is not None:
            name, uri, pose = parsed
            markers[name] = (uri, pose)
    return markers


def _drift_problems(
    checkpoints: list[Checkpoint],
    markers: dict[str, tuple[str, tuple[float, float, float]]],
) -> list[str]:
    expected = {cp.marker_name for cp in checkpoints}
    problems: list[str] = [
        f"world has marker {extra!r} with no matching checkpoint in the YAML"
        for extra in sorted(set(markers) - expected)
    ]
    for cp in checkpoints:
        if cp.marker_name not in markers:
            problems.append(f"checkpoint {cp.checkpoint_id!r} has no marker in the world")
            continue
        uri, (wx, wy, wz) = markers[cp.marker_name]
        if uri != cp.uri:
            problems.append(f"marker {cp.marker_name!r} uri {uri!r} != expected {cp.uri!r}")
        if not _poses_match((wx, wy, wz), (cp.x, cp.y, cp.z)):
            problems.append(
                f"marker {cp.marker_name!r} position ({wx}, {wy}, {wz}) != "
                f"YAML ({cp.x}, {cp.y}, {cp.z})"
            )
    return problems


def _poses_match(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
    return all(math.isclose(p, q, abs_tol=_POSE_TOL) for p, q in zip(a, b, strict=True))


def check_drift(
    config_path: str | Path = CONFIG_PATH,
    world_path: str | Path = WORLD_PATH,
    template_path: str | Path = TEMPLATE_PATH,
    models_dir: Path = MODELS_DIR,
) -> list[str]:
    """Return drift problems (empty == clean).

    The committed world must byte-match a fresh render of the *current* template + checkpoints, with
    every generation guard satisfied. ``render_world`` raises :class:`ComposeError` on a guard
    violation (e.g. a missing model dir — F-02); the CI wrapper catches it as a drift problem. A byte
    mismatch (e.g. a template-body edit that was never regenerated — F-01) is reported with the
    per-marker diffs kept as supplemental detail so messages stay specific.
    """
    expected = render_world(config_path, template_path, models_dir)
    committed = Path(world_path).read_text()
    if committed == expected:
        return []
    problems = [f"{world_path}: differs from a fresh render of the template + checkpoints"]
    checkpoints = load_checkpoints(config_path)
    problems += _drift_problems(checkpoints, _world_markers(str(world_path)))
    return problems


def _run_check() -> int:
    """Run the drift gate; print problems (if any) to stderr. Return the process exit code."""
    try:
        problems = check_drift() + validate_world_design()
    except ComposeError as exc:
        problems = [str(exc)]
    if not problems:
        print("patrol_world.sdf: in sync with checkpoints.yaml.")
        return 0
    print("patrol_world.sdf has drifted from checkpoints.yaml:", file=sys.stderr)
    for p in problems:
        print(f"  - {p}", file=sys.stderr)
    print("re-run: python3 sim/tools/compose_world.py", file=sys.stderr)
    return 1


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if "--check" in argv:
        return _run_check()
    compose_world()
    print(f"wrote {WORLD_PATH.relative_to(REPO_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
