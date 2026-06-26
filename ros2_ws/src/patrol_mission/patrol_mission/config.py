"""Mission YAML parse + schema + validation (design §4.2.5, MC-3, INF-M3).

ROS-free and pure. Loads a reviewable, diffable mission file into frozen
dataclasses so no route/param data is hardcoded in source. Validation is
**fail-loud**: a missing field, an unknown frame, or (M4) an unresolvable
``checkpoint_id`` raises at load time, so the node refuses to start and a bad
config never flies.

M1 supports the basic mission (no waypoints) and inline ``position``/``frame``
waypoints. ``checkpoint_id`` resolution against 03's ``checkpoints.yaml`` lands
in M4; until then a ``checkpoint_id`` waypoint fails loud.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

from patrol_mission.frames import Point

_VALID_FRAMES = ("enu", "ned")

# Every top-level mission key the loader recognizes. A key outside this set is a typo (a misspelled
# section name like `completino:` would otherwise be silently ignored — `_section` only fails loud on
# an unknown key *inside* a present section, via `raw.get(key, {})` defaulting an absent section to
# {} — so defaults would apply and a config error would fly, Hermes Medium PR #8 R11). Validated up
# front so a misspelled section fails loud at the same boundary as a missing required field.
_KNOWN_TOP_LEVEL_KEYS = frozenset(
    {"takeoff_alt_m", "hover_time_s", "completion", "abort", "approach", "home", "waypoints"}
)


@dataclass(frozen=True)
class Completion:
    """Waypoint-completion criterion (MC-5). Tolerance + hold, never equality."""

    tolerance_m: float = 0.5  # OQ-4 default
    hold_time_s: float = 2.0  # OQ-4 default


@dataclass(frozen=True)
class AbortConfig:
    # Parsed and validated now so the mission file carries the full schema, but NOT yet enforced:
    # the abort/RTH state + battery_status telemetry that consume this land in M4. Until then this
    # is forward-declared config, not live behavior.
    low_battery_threshold: float = 0.20  # OQ-6 default (battery_status.remaining)


@dataclass(frozen=True)
class Approach:
    """Checkpoint approach geometry (SIM-4 / ADR-0012). How the drone hovers to resolve a tag.

    Tags are emitted at zero yaw (face normal along world +Y), so the stand-off is taken along +Y and
    the waypoint yaw is computed to face the tag. The hover also climbs above the tag so the airframe's
    fixed down-pitched camera boresight lands on the tag center rather than the tag sitting jammed at
    the top frame edge (where apriltag could not resolve it at dwell — ADR-0012). Optional section —
    the defaults hold the shipped stand-off and the airframe's ~20-deg camera down-pitch.
    """

    standoff_m: float = 3.0
    # The airframe camera's fixed downward pitch (rad), gz_x500_patrol model.sdf camera_link pose
    # (0 0.35 0). The hover rises standoff_m*tan(this) above the tag so the boresight centers it.
    camera_pitch_rad: float = 0.35


@dataclass(frozen=True)
class Waypoint:
    position: Point  # source position (inline `position`, or a checkpoint's resolved hover pose)
    frame: str  # "enu" | "ned" — the frame `position` is expressed in (NOT necessarily ENU)
    dwell_s: float
    checkpoint_id: str | None = None  # set when resolved from checkpoints.yaml (M4)
    yaw_enu: float | None = None  # ENU heading to hold (checkpoint waypoints); None = no constraint


@dataclass(frozen=True)
class MissionConfig:
    takeoff_alt_m: float
    hover_time_s: float
    completion: Completion
    abort: AbortConfig
    home_position: Point
    home_frame: str
    waypoints: tuple[Waypoint, ...]
    approach: Approach = Approach()  # SIM-4; default holds the sane stand-off


def _require(raw: dict[str, Any], key: str, what: str = "mission config") -> Any:
    """Fetch a required field or fail loud, with mapping context (defaults to the top-level doc).

    ``what`` names the mapping the key is required in, so a nested miss reads with context — e.g.
    ``_require(home, "position", "mission config 'home'")`` raises "...'home' missing required field
    'position'" rather than an unqualified top-level-sounding message (Hermes PR #8 R11 polish).
    """
    if key not in raw:
        raise ValueError(f"{what} missing required field {key!r}")
    return raw[key]


def _reject_unknown_top_level_keys(raw: dict[str, Any]) -> None:
    """Fail loud on any top-level mission key outside the known set (a misspelled section, Hermes)."""
    unknown = set(raw) - _KNOWN_TOP_LEVEL_KEYS
    if unknown:
        raise ValueError(
            f"mission config has unknown top-level key(s) {sorted(unknown)}; "
            f"allowed {sorted(_KNOWN_TOP_LEVEL_KEYS)}"
        )


def _require_mapping(value: Any, what: str) -> dict[str, Any]:
    """Fail loud unless ``value`` is a mapping, with field context (Hermes Medium).

    A null, empty, or scalar YAML node (an empty mission document, ``home:`` with no value, a
    non-mapping waypoint/checkpoint entry) would otherwise leak a bare ``TypeError`` from the
    downstream ``key in value`` / ``value[...]`` access. Convert it to the loader's contracted
    :class:`ValueError` at the same fail-loud boundary as the missing-field check.
    """
    if not isinstance(value, dict):
        raise ValueError(f"{what} must be a mapping, got {type(value).__name__}")
    return value


def _require_list(value: Any, what: str) -> list:
    """Fail loud unless ``value`` is a list, with field context (Hermes Medium).

    A present-but-null or scalar ``waypoints:`` would otherwise leak a bare ``TypeError`` when
    iterated; raise the contracted :class:`ValueError` instead.
    """
    if not isinstance(value, list):
        raise ValueError(f"{what} must be a list, got {type(value).__name__}")
    return value


def _coerce_section_floats(section: dict[str, Any], cls: type, key: str) -> dict[str, Any]:
    """Coerce each provided ``float``-typed section field through :func:`_number` so a quoted YAML
    scalar (``tolerance_m: "0.5"``) is stored as a float — not a str that passes range validation yet
    later crashes a state-machine comparison (Hermes Medium). This mirrors the top-level numerics,
    which are already cast in :func:`load`. Unknown keys pass through untouched so ``_section``'s
    ``cls(**section)`` still raises the contracted ValueError for them.
    """
    float_fields = {f.name for f in fields(cls) if f.type in (float, "float")}
    return {
        name: _number(value, f"{key}.{name}") if name in float_fields else value
        for name, value in section.items()
    }


def _section[T](raw: dict[str, Any], key: str, cls: type[T]) -> T:
    """Build an optional config-section dataclass, fail-loud on a null/non-mapping/unknown-key section.

    A section may be omitted entirely — the dataclass defaults then apply. But a present-but-null
    section (``completion:`` with no value), a non-mapping value, or an unknown key must raise the
    loader's contracted :class:`ValueError` with field context, not the bare ``TypeError`` that
    ``cls(**section)`` would otherwise throw (review #3). Numeric fields are coerced (a quoted scalar
    becomes a float, or fails loud as "<field> must be a number") before construction.
    """
    section = raw.get(key, {})
    if section is None:
        raise ValueError(
            f"mission config section {key!r} is null; omit it for defaults or give a mapping"
        )
    if not isinstance(section, dict):
        raise ValueError(
            f"mission config section {key!r} must be a mapping, got {type(section).__name__}"
        )
    try:
        return cls(**_coerce_section_floats(section, cls, key))
    except TypeError as exc:  # unknown / misspelled key in the section
        raise ValueError(f"mission config section {key!r} is invalid: {exc}") from exc


def _validate_frame(frame: str, where: str) -> str:
    if frame not in _VALID_FRAMES:
        raise ValueError(f"{where} declares unknown frame {frame!r}: expected 'enu' or 'ned'")
    return frame


def _number(value: Any, name: str) -> float:
    """Coerce a scalar numeric field to a *finite* float, fail loud with field context on a bad value.

    A non-numeric numeric field would otherwise leak an inconsistent bare exception: an un-fielded
    ``ValueError`` from a construction-time ``float(...)`` cast for ``takeoff_alt_m`` / ``hover_time_s``
    / ``dwell_s``, but a bare ``TypeError`` from the later ``<=`` range comparison for the section
    fields (``completion.*`` / ``abort.*``), which are built straight into their dataclass un-cast.
    Funnel every scalar numeric field through here so operators get one consistent
    ``"<field> must be a number, got <value>"`` diagnostic (Hermes polish).

    Two scalars that ``float()`` accepts but a numeric mission field must NOT (PR #8 post-mortem D —
    "NaN, Inf, and booleans without proper coercion") are rejected here, at the same fail-loud
    boundary:

    * A YAML boolean (``true``/``false``). ``bool`` is an ``int`` subclass, so ``float(True)`` is
      ``1.0`` — a boolean would silently become a 1/0 magnitude. A bool in a numeric field is a
      config typo, not a quantity, so reject it before the cast can swallow it.
    * A non-finite value (``NaN``/``Inf``, parsed from ``.nan`` / ``.inf`` or a quoted ``"nan"`` /
      ``"inf"``). It would pass every range guard below — ``nan <= 0`` and ``inf <= 0`` are both
      ``False``, so ``_positive`` / ``_non_negative`` accept it — then poison a downstream
      distance/threshold comparison in the state machine. Reject it so a non-finite mission
      parameter never flies.
    """
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, got bool {value!r}")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number, got {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number, got {value!r}")
    return number


def _positive(value: Any, name: str) -> None:
    if _number(value, name) <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")


def _non_negative(value: Any, name: str) -> None:
    if _number(value, name) < 0:
        raise ValueError(f"{name} must be >= 0, got {value}")


def _unit_interval(value: Any, name: str) -> None:
    if not 0.0 <= _number(value, name) <= 1.0:
        raise ValueError(f"{name} must be in [0, 1], got {value}")


def _half_turn_down(value: Any, name: str) -> None:
    """A fixed downward camera pitch in [0, pi/2): non-negative (the camera looks down, not up) and
    strictly below vertical so the ``tan`` that sets the hover climb stays finite (ADR-0012)."""
    if not 0.0 <= _number(value, name) < math.pi / 2.0:
        raise ValueError(f"{name} must be in [0, pi/2), got {value}")


def _validate_semantics(cfg: MissionConfig) -> None:
    """Fail loud on a numerically well-typed but semantically impossible mission (Hermes Medium).

    Casting in the loader guarantees the fields are floats; it does NOT stop a config that would
    immediately land (``takeoff_alt_m <= 0``), never complete (``tolerance_m <= 0`` so the machine
    never settles within tolerance), or carry an out-of-range future-abort threshold. Guard those
    invariants here, at the same fail-loud boundary as the missing-field / unknown-frame checks.
    """
    _positive(cfg.takeoff_alt_m, "takeoff_alt_m")
    _non_negative(cfg.hover_time_s, "hover_time_s")
    _positive(cfg.completion.tolerance_m, "completion.tolerance_m")
    _non_negative(cfg.completion.hold_time_s, "completion.hold_time_s")
    _unit_interval(cfg.abort.low_battery_threshold, "abort.low_battery_threshold")
    _positive(cfg.approach.standoff_m, "approach.standoff_m")
    _half_turn_down(cfg.approach.camera_pitch_rad, "approach.camera_pitch_rad")
    for i, wp in enumerate(cfg.waypoints):
        _non_negative(wp.dwell_s, f"waypoints[{i}].dwell_s")


def _coord(p: dict[str, Any], where: str, axis: str) -> float:
    """Read one numeric position axis, fail loud on a missing or non-numeric value (Hermes)."""
    if axis not in p:
        raise ValueError(f"{where} position missing required axis {axis!r}")
    return _number(p[axis], f"{where} position axis {axis!r}")


def _point(p: Any, where: str) -> Point:
    """Build an ``(x, y, z)`` point from a mapping, fail loud on a malformed position (Hermes).

    A null/scalar ``position`` (or one missing/non-numeric ``x``/``y``/``z``) would otherwise leak a
    bare ``TypeError``/``KeyError`` from the subscript or ``float(...)`` cast; raise the loader's
    contracted :class:`ValueError` with field context instead, consistent with the document/section/
    waypoint/checkpoint guards.
    """
    p = _require_mapping(p, f"{where} position")
    return (_coord(p, where, "x"), _coord(p, where, "y"), _coord(p, where, "z"))


def _checkpoint_list(raw: Any, checkpoints_yaml_path: str) -> list:
    """Extract the checkpoint list from either the canonical keyed form or a bare list.

    03's canonical ``checkpoints.yaml`` (design §4.2.3, M5) is a mapping with a top-level
    ``checkpoints:`` key holding the list. The interim 02-authored stand-in was a bare list; the
    loader stays back-compatible with it (a list ``raw`` is used directly) so this consumer does not
    break across the schema migration. Any other shape — a mapping without ``checkpoints``, a scalar
    — fails loud with field context (INF-M3).
    """
    if isinstance(raw, dict):
        if "checkpoints" not in raw:
            raise ValueError(
                f"checkpoints file {checkpoints_yaml_path!r} is a mapping but has no 'checkpoints:' "
                "key; give a top-level 'checkpoints:' list (or a bare list, back-compat)"
            )
        return _require_list(raw["checkpoints"], f"{checkpoints_yaml_path!r} 'checkpoints'")
    if isinstance(raw, list):
        return raw
    raise ValueError(
        f"checkpoints file {checkpoints_yaml_path!r} must be a 'checkpoints:' mapping or a bare list"
    )


def _load_checkpoints(checkpoints_yaml_path: str) -> dict[str, Point]:
    """Load 03's checkpoint-positions YAML into ``{checkpoint_id: ENU position}`` (read-only).

    Called only when a waypoint references a ``checkpoint_id`` (so a basic mission with no
    checkpoint references never needs the file to exist). Fail loud — a missing file, a
    wrong-shaped document, or an entry missing its ``position`` raises with field context so an
    unresolvable route never flies (INF-M3). The path is the caller-supplied parameter (OQ-2:
    03 owns the file; an agreed-different location is a one-line config change, not a code edit).
    Accepts both the canonical ``checkpoints:``-keyed mapping (M5) and the interim bare list.
    """
    try:
        with open(checkpoints_yaml_path) as fh:
            raw = yaml.safe_load(fh)
    except FileNotFoundError as exc:
        raise ValueError(
            f"checkpoints file {checkpoints_yaml_path!r} not found, "
            "but a waypoint references a checkpoint_id"
        ) from exc
    return _checkpoints_map(_checkpoint_list(raw, checkpoints_yaml_path), checkpoints_yaml_path)


def _checkpoints_map(raw: list, checkpoints_yaml_path: str) -> dict[str, Point]:
    """Fold checkpoint entries into ``{checkpoint_id: ENU position}``, fail-loud on duplicate ids.

    A plain ``dict(...)`` comprehension would let a duplicate ``checkpoint_id`` silently overwrite
    an earlier coordinate, hiding conflicting 03-owned checkpoint data (Hermes Medium). Building the
    map explicitly lets a duplicate raise with field context, matching this loader's fail-loud
    contract for every other malformed-checkpoint case (INF-M3).
    """
    checkpoints: dict[str, Point] = {}
    for entry in raw:
        cid, point = _checkpoint_entry(entry)
        if cid in checkpoints:
            raise ValueError(
                f"checkpoints file {checkpoints_yaml_path!r} declares duplicate checkpoint_id "
                f"{cid!r}; checkpoint ids must be unique (a duplicate silently overwrites coordinates)"
            )
        checkpoints[cid] = point
    return checkpoints


def _checkpoint_id(value: Any, where: str) -> str:
    """A ``checkpoint_id`` must be a string — the 02/03 checkpoint namespace is string-keyed.

    YAML happily parses ``checkpoint_id: 1`` as an ``int``; left unchecked it would be keyed into the
    checkpoints map and stored on ``Waypoint.checkpoint_id`` (declared ``str``), silently violating
    the shared-namespace contract and risking an ``int``/``str`` key mismatch between a checkpoint and
    the waypoint that references it. Validate the type on both sides at the loader's fail-loud boundary
    (Hermes Medium). ``bool`` is rejected too (``isinstance(True, str)`` is already False).
    """
    if not isinstance(value, str):
        raise ValueError(
            f"{where} checkpoint_id must be a string, got {type(value).__name__} {value!r}"
        )
    return value


def _checkpoint_entry(entry: Any) -> tuple[str, Point]:
    """Validate one checkpoints entry into ``(checkpoint_id, ENU position)``. Fail loud (INF-M3)."""
    entry = _require_mapping(entry, "checkpoints entry")
    if "checkpoint_id" not in entry:
        raise ValueError(f"checkpoints entry missing required 'checkpoint_id': {entry!r}")
    cid = _checkpoint_id(entry["checkpoint_id"], "checkpoints entry")
    if "position" not in entry:
        raise ValueError(f"checkpoint {cid!r} missing required 'position'")
    return cid, _point(entry["position"], f"checkpoint {cid!r}")


def _references_checkpoint(raw_waypoints: list) -> bool:
    """Whether any waypoint resolves via a ``checkpoint_id`` (so checkpoints must be loaded)."""
    return any("checkpoint_id" in w for w in raw_waypoints)


def _resolve_checkpoints(raw_waypoints: list, checkpoints_yaml_path: str) -> dict[str, Point]:
    """Load checkpoints iff referenced; fail loud when referenced but no path was supplied (OQ-2).

    The checkpoints file is 03's deliverable, so its path is a caller-supplied parameter with no
    in-package default — a CWD-relative default resolves differently depending on where the launch
    ran from (Hermes Medium). When a waypoint references a ``checkpoint_id`` but no path was given,
    fail loud with guidance rather than silently depending on the current directory.
    """
    if not _references_checkpoint(raw_waypoints):
        return {}
    if not checkpoints_yaml_path:
        raise ValueError(
            "a waypoint references a checkpoint_id but no checkpoints_yaml path was provided; "
            "pass checkpoints_yaml:=<absolute path> (the checkpoints file is 03-owned, OQ-2)"
        )
    if not Path(checkpoints_yaml_path).is_absolute():
        raise ValueError(
            f"checkpoints_yaml path {checkpoints_yaml_path!r} must be absolute so resolution does "
            "not depend on the working directory; pass checkpoints_yaml:=<absolute path> (Hermes)"
        )
    return _load_checkpoints(checkpoints_yaml_path)


_CHECKPOINT_WAYPOINT_KEYS = frozenset({"checkpoint_id", "dwell_s"})
_INLINE_WAYPOINT_KEYS = frozenset({"position", "frame", "dwell_s"})


def _waypoint_shape(w: dict, index: int) -> str:
    """Classify a waypoint as ``'checkpoint'`` or ``'inline'``; fail loud if it is both or neither.

    A waypoint must be exactly one shape — checkpoint-based ``{checkpoint_id, dwell_s}`` or inline
    ``{position, frame, dwell_s}``. Mixing a ``checkpoint_id`` with inline ``position``/``frame`` is
    rejected rather than silently honoring the checkpoint and dropping the stray inline keys, which
    would hide a conflicting/typo'd route (Hermes Medium).
    """
    has_checkpoint = "checkpoint_id" in w
    has_inline = "position" in w or "frame" in w
    if has_checkpoint and has_inline:
        raise ValueError(
            f"waypoint[{index}] is ambiguous: it mixes a checkpoint_id with inline position/frame; "
            "give exactly one of {checkpoint_id, dwell_s} or {position, frame, dwell_s}"
        )
    if has_checkpoint:
        return "checkpoint"
    if has_inline:
        return "inline"
    raise ValueError(
        f"waypoint[{index}] must be checkpoint-based {{checkpoint_id, dwell_s}} or inline "
        f"{{position, frame, dwell_s}}; got keys {sorted(w)}"
    )


def _check_waypoint_keys(w: dict, index: int, allowed: frozenset[str]) -> None:
    """A waypoint must carry exactly its shape's keys — a missing or unexpected key fails loud."""
    keys = set(w)
    missing = allowed - keys
    if missing:
        raise ValueError(f"waypoint[{index}] is missing required key(s) {sorted(missing)}")
    extra = keys - allowed
    if extra:
        raise ValueError(
            f"waypoint[{index}] has unexpected key(s) {sorted(extra)}; allowed {sorted(allowed)}"
        )


def _approach_pose(
    tag: Point, standoff_m: float, camera_pitch_rad: float = 0.35
) -> tuple[Point, float]:
    """Hover pose that looks at a zero-yaw AprilTag from a stand-off (SIM-4 / ADR-0012).

    The World Composer emits every marker at zero yaw, so a tag's readable face normal lies along
    world +Y. The drone hovers ``standoff_m`` north (+Y) of the tag and yaws to face the tag center.

    It also climbs ``standoff_m * tan(camera_pitch_rad)`` **above** the tag. The airframe camera is
    rigidly pitched ``camera_pitch_rad`` down, so a same-altitude hover put the tag ~20 deg up — at the
    extreme top of the frame, foreshortened past apriltag's reach at dwell (every SITL checkpoint
    capture came back empty — ADR-0012). Rising by that much makes the down-pitched boresight land on
    the tag center at the stand-off range, so the forward camera squarely frames the marker. Returns
    the ENU hover point and the ENU yaw (CCW from East) toward the tag (yaw is unaffected by the climb).
    """
    tx, ty, tz = tag
    hover: Point = (tx, ty + standoff_m, tz + standoff_m * math.tan(camera_pitch_rad))
    return hover, math.atan2(ty - hover[1], tx - hover[0])


def _checkpoint_waypoint(
    w: dict, index: int, checkpoints: dict[str, Point], approach: Approach
) -> Waypoint:
    """Resolve a checkpoint waypoint to a stand-off hover pose facing the tag (SIM-4). Fail loud."""
    cid = _checkpoint_id(w["checkpoint_id"], f"waypoint[{index}]")
    if cid not in checkpoints:
        raise ValueError(f"waypoint[{index}] references unknown checkpoint_id {cid!r}")
    hover, yaw_enu = _approach_pose(
        checkpoints[cid], approach.standoff_m, approach.camera_pitch_rad
    )
    return Waypoint(
        position=hover,
        frame="enu",
        dwell_s=_number(w["dwell_s"], f"waypoints[{index}].dwell_s"),
        checkpoint_id=cid,
        yaw_enu=yaw_enu,
    )


def _inline_waypoint(w: dict, index: int) -> Waypoint:
    """Build an inline ``position``+``frame`` waypoint (fail loud on an unknown frame)."""
    frame = _validate_frame(w["frame"], f"waypoint[{index}]")
    return Waypoint(
        position=_point(w["position"], f"waypoint[{index}]"),
        frame=frame,
        dwell_s=_number(w["dwell_s"], f"waypoints[{index}].dwell_s"),
    )


def _parse_waypoint(
    w: dict, index: int, checkpoints: dict[str, Point], approach: Approach
) -> Waypoint:
    """Build a Waypoint from exactly one shape: a checkpoint reference or an inline position.

    Each waypoint must be unambiguously one shape and carry exactly that shape's keys; a mixed,
    incomplete, or extra-keyed waypoint is rejected with an indexed ValueError so a malformed route
    never flies (Hermes Medium).
    """
    if _waypoint_shape(w, index) == "checkpoint":
        _check_waypoint_keys(w, index, _CHECKPOINT_WAYPOINT_KEYS)
        return _checkpoint_waypoint(w, index, checkpoints, approach)
    _check_waypoint_keys(w, index, _INLINE_WAYPOINT_KEYS)
    return _inline_waypoint(w, index)


def _waypoint_entries(raw: dict[str, Any]) -> list:
    """Fetch the required ``waypoints`` list and validate every entry is a mapping (fail loud).

    ``waypoints`` is required (a basic mission uses ``waypoints: []``). A present-but-null/scalar
    ``waypoints:``, or a non-mapping entry (``- 123``), would otherwise leak a bare ``TypeError``
    from the downstream ``checkpoint_id in w`` membership test (Hermes Medium); guard it here with
    index context at the loader's fail-loud boundary.
    """
    raw_waypoints = _require_list(_require(raw, "waypoints"), "mission config 'waypoints'")
    for i, w in enumerate(raw_waypoints):
        _require_mapping(w, f"waypoint[{i}]")
    return raw_waypoints


def load_mission_config(
    mission_yaml_path: str,
    checkpoints_yaml_path: str = "",
) -> MissionConfig:
    """Parse + validate a mission YAML into a frozen :class:`MissionConfig` (MC-3).

    Args:
        mission_yaml_path: path to the mission YAML to load.
        checkpoints_yaml_path: path to 03's checkpoint-positions YAML (OQ-2). Read only
            when a waypoint references a ``checkpoint_id``; there is no in-package default
            (the file is 03-owned), so a checkpoint-referencing mission must pass an
            explicit path — a CWD-relative default would resolve differently depending on
            where the launch ran from (Hermes Medium).

    Raises:
        ValueError: on a malformed top-level shape (a null/scalar mission document, a
            null/scalar ``home`` section, a null/scalar ``waypoints`` list, or a non-mapping
            waypoint/checkpoint entry), a missing required field, an unknown frame, an
            out-of-range numeric field (see :func:`_validate_semantics`), an unresolvable
            ``checkpoint_id``, a missing/malformed checkpoints file when one is referenced,
            or a ``checkpoint_id`` reference with no checkpoints path supplied.
    """
    with open(mission_yaml_path) as fh:
        raw = _require_mapping(yaml.safe_load(fh), "mission config")

    _reject_unknown_top_level_keys(raw)
    approach = _section(raw, "approach", Approach)
    home = _require_mapping(_require(raw, "home"), "mission config 'home'")
    raw_waypoints = _waypoint_entries(raw)
    checkpoints = _resolve_checkpoints(raw_waypoints, checkpoints_yaml_path)
    waypoints = tuple(
        _parse_waypoint(w, i, checkpoints, approach) for i, w in enumerate(raw_waypoints)
    )

    cfg = MissionConfig(
        takeoff_alt_m=_number(_require(raw, "takeoff_alt_m"), "takeoff_alt_m"),
        hover_time_s=_number(_require(raw, "hover_time_s"), "hover_time_s"),
        completion=_section(raw, "completion", Completion),
        abort=_section(raw, "abort", AbortConfig),
        home_position=_point(_require(home, "position", "mission config 'home'"), "home"),
        home_frame=_validate_frame(_require(home, "frame", "mission config 'home'"), "home"),
        waypoints=waypoints,
        approach=approach,
    )
    _validate_semantics(cfg)
    return cfg
