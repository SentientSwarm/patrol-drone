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

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from patrol_mission.frames import Point

_VALID_FRAMES = ("enu", "ned")


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
class Waypoint:
    position: Point  # source position (from inline `position` or, M4, a checkpoint_id)
    frame: str  # "enu" | "ned" — the frame `position` is expressed in (NOT necessarily ENU)
    dwell_s: float
    checkpoint_id: str | None = None  # set when resolved from checkpoints.yaml (M4)


@dataclass(frozen=True)
class MissionConfig:
    takeoff_alt_m: float
    hover_time_s: float
    completion: Completion
    abort: AbortConfig
    home_position: Point
    home_frame: str
    waypoints: tuple[Waypoint, ...]


def _require(raw: dict[str, Any], key: str) -> Any:
    """Fetch a required top-level field or fail loud."""
    if key not in raw:
        raise ValueError(f"mission config missing required field {key!r}")
    return raw[key]


def _section[T](raw: dict[str, Any], key: str, cls: type[T]) -> T:
    """Build an optional config-section dataclass, fail-loud on a null/non-mapping/unknown-key section.

    A section may be omitted entirely — the dataclass defaults then apply. But a present-but-null
    section (``completion:`` with no value), a non-mapping value, or an unknown key must raise the
    loader's contracted :class:`ValueError` with field context, not the bare ``TypeError`` that
    ``cls(**section)`` would otherwise throw (review #3).
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
        return cls(**section)
    except TypeError as exc:  # unknown / misspelled key in the section
        raise ValueError(f"mission config section {key!r} is invalid: {exc}") from exc


def _validate_frame(frame: str, where: str) -> str:
    if frame not in _VALID_FRAMES:
        raise ValueError(f"{where} declares unknown frame {frame!r}: expected 'enu' or 'ned'")
    return frame


def _positive(value: float, name: str) -> None:
    if value <= 0:
        raise ValueError(f"{name} must be > 0, got {value}")


def _non_negative(value: float, name: str) -> None:
    if value < 0:
        raise ValueError(f"{name} must be >= 0, got {value}")


def _unit_interval(value: float, name: str) -> None:
    if not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must be in [0, 1], got {value}")


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
    for i, wp in enumerate(cfg.waypoints):
        _non_negative(wp.dwell_s, f"waypoints[{i}].dwell_s")


def _point(p: dict) -> Point:
    return (float(p["x"]), float(p["y"]), float(p["z"]))


def _load_checkpoints(checkpoints_yaml_path: str) -> dict[str, Point]:
    """Load 03's checkpoint-positions YAML into ``{checkpoint_id: ENU position}`` (read-only).

    Called only when a waypoint references a ``checkpoint_id`` (so a basic mission with no
    checkpoint references never needs the file to exist). Fail loud — a missing file, a
    non-list document, or an entry missing its ``position`` raises with field context so an
    unresolvable route never flies (INF-M3). The path is the caller-supplied parameter (OQ-2:
    03 owns the file; an agreed-different location is a one-line config change, not a code edit).
    """
    try:
        with open(checkpoints_yaml_path) as fh:
            raw = yaml.safe_load(fh)
    except FileNotFoundError as exc:
        raise ValueError(
            f"checkpoints file {checkpoints_yaml_path!r} not found, "
            "but a waypoint references a checkpoint_id"
        ) from exc
    if not isinstance(raw, list):
        raise ValueError(
            f"checkpoints file {checkpoints_yaml_path!r} must be a list of checkpoints"
        )
    return dict(_checkpoint_entry(entry) for entry in raw)


def _checkpoint_entry(entry: dict) -> tuple[str, Point]:
    """Validate one checkpoints entry into ``(checkpoint_id, ENU position)``. Fail loud (INF-M3)."""
    if "checkpoint_id" not in entry:
        raise ValueError(f"checkpoints entry missing required 'checkpoint_id': {entry!r}")
    if "position" not in entry:
        raise ValueError(f"checkpoint {entry['checkpoint_id']!r} missing required 'position'")
    return entry["checkpoint_id"], _point(entry["position"])


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


def _parse_waypoint(w: dict, checkpoints: dict[str, Point]) -> Waypoint:
    """Build a Waypoint from a resolved ``checkpoint_id`` (ENU) or an inline ``position``+``frame``."""
    if "checkpoint_id" in w:
        cid = w["checkpoint_id"]
        if cid not in checkpoints:
            raise ValueError(f"waypoint references unknown checkpoint_id {cid!r}")
        return Waypoint(
            position=checkpoints[cid], frame="enu", dwell_s=float(w["dwell_s"]), checkpoint_id=cid
        )
    frame = _validate_frame(w["frame"], "waypoint")
    return Waypoint(position=_point(w["position"]), frame=frame, dwell_s=float(w["dwell_s"]))


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
        ValueError: on a missing required field, an unknown frame, an out-of-range
            numeric field (see :func:`_validate_semantics`), an unresolvable
            ``checkpoint_id``, a missing/malformed checkpoints file when one is
            referenced, or a ``checkpoint_id`` reference with no checkpoints path supplied.
    """
    with open(mission_yaml_path) as fh:
        raw = yaml.safe_load(fh)

    home = _require(raw, "home")
    raw_waypoints = _require(raw, "waypoints")
    checkpoints = _resolve_checkpoints(raw_waypoints, checkpoints_yaml_path)
    waypoints = tuple(_parse_waypoint(w, checkpoints) for w in raw_waypoints)

    cfg = MissionConfig(
        takeoff_alt_m=float(_require(raw, "takeoff_alt_m")),
        hover_time_s=float(_require(raw, "hover_time_s")),
        completion=_section(raw, "completion", Completion),
        abort=_section(raw, "abort", AbortConfig),
        home_position=_point(home["position"]),
        home_frame=_validate_frame(home["frame"], "home"),
        waypoints=waypoints,
    )
    _validate_semantics(cfg)
    return cfg
