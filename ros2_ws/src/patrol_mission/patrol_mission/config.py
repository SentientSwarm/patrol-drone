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
    position_enu: Point  # source position (from inline `position` or, M4, a checkpoint_id)
    frame: str  # "enu" | "ned"
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


def _validate_frame(frame: str, where: str) -> str:
    if frame not in _VALID_FRAMES:
        raise ValueError(f"{where} declares unknown frame {frame!r}: expected 'enu' or 'ned'")
    return frame


def _point(p: dict) -> Point:
    return (float(p["x"]), float(p["y"]), float(p["z"]))


def _parse_waypoint(w: dict) -> Waypoint:
    if "checkpoint_id" in w:
        # M4 (T2.2) resolves checkpoint_id against 03's sim/config/checkpoints.yaml.
        raise ValueError(
            f"waypoint references checkpoint_id {w['checkpoint_id']!r}: "
            "checkpoint_id resolution lands in M4 (basic mission uses inline waypoints only)"
        )
    frame = _validate_frame(w["frame"], "waypoint")
    return Waypoint(position_enu=_point(w["position"]), frame=frame, dwell_s=float(w["dwell_s"]))


def load_mission_config(mission_yaml_path: str) -> MissionConfig:
    """Parse + validate a mission YAML into a frozen :class:`MissionConfig` (MC-3).

    Args:
        mission_yaml_path: path to the mission YAML to load.

    Raises:
        ValueError: on a missing required field, an unknown frame, or (M1) a
            ``checkpoint_id`` waypoint.

    M4 (T2.2) adds a ``checkpoints_yaml_path`` parameter (default
    ``sim/config/checkpoints.yaml``, the OQ-2 file location, kept behind a
    parameter so an agreed-different location is a one-line config change) for
    ``checkpoint_id`` resolution against 03's checkpoint-positions file.
    """
    with open(mission_yaml_path) as fh:
        raw = yaml.safe_load(fh)

    home = _require(raw, "home")
    waypoints = tuple(_parse_waypoint(w) for w in _require(raw, "waypoints"))

    return MissionConfig(
        takeoff_alt_m=float(_require(raw, "takeoff_alt_m")),
        hover_time_s=float(_require(raw, "hover_time_s")),
        completion=Completion(**raw.get("completion", {})),
        abort=AbortConfig(**raw.get("abort", {})),
        home_position=_point(home["position"]),
        home_frame=_validate_frame(home["frame"], "home"),
        waypoints=waypoints,
    )
