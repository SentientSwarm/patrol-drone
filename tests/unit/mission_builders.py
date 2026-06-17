"""Shared Layer-A builders for the MissionStateMachine unit suite (ROS-free).

Kept in one module so ``test_state_machine.py`` and ``test_abort.py`` share the
config / telemetry / state-machine scaffolds instead of copy-pasting them (the
CodeScene duplication trap on M-milestone test PRs). Each test varies only the
part under test; the scaffold lives here once.
"""

from __future__ import annotations

from typing import Any

from patrol_mission.config import AbortConfig, Completion, MissionConfig, Waypoint
from patrol_mission.frames import Point
from patrol_mission.state_machine import MissionStateMachine, Telemetry

TAKEOFF_ALT = 5.0
HOVER_TIME = 10.0
TOL = 0.5
HOLD = 2.0
LOW_BATTERY = 0.20
DWELL = 3.0
HOME_NED: Point = (0.0, 0.0, -2.0)
# Two sample waypoints in NED (already at the single frame boundary), reused across the suite.
WP0: Point = (10.0, 0.0, -2.0)
WP1: Point = (0.0, 10.0, -2.0)
# Takeoff target in NED: home xy, takeoff_alt_m AGL above home (down increases downward, so "up"
# subtracts from home's own down coordinate). Derived from HOME_NED so it tracks the AGL-from-home
# computation rather than assuming home sits at z=0.
TAKEOFF_NED: Point = (HOME_NED[0], HOME_NED[1], HOME_NED[2] - TAKEOFF_ALT)


def make_config(waypoints: tuple[Waypoint, ...] = ()) -> MissionConfig:
    return MissionConfig(
        takeoff_alt_m=TAKEOFF_ALT,
        hover_time_s=HOVER_TIME,
        completion=Completion(tolerance_m=TOL, hold_time_s=HOLD),
        abort=AbortConfig(low_battery_threshold=LOW_BATTERY),
        home_position=(0.0, 0.0, 2.0),
        home_frame="enu",
        waypoints=waypoints,
    )


def make_sm(
    waypoints_ned: list[Point] | None = None,
    home_ned: Point = HOME_NED,
    config: MissionConfig | None = None,
) -> MissionStateMachine:
    """A basic (no-waypoint) machine by default; pass ``config`` to override params."""
    return MissionStateMachine(config or make_config(), waypoints_ned or [], home_ned)


def make_patrol_sm(waypoints_ned: list[Point], dwell_s: float = DWELL) -> MissionStateMachine:
    """A patrol machine whose config.waypoints align 1:1 with ``waypoints_ned`` (uniform dwell)."""
    wps = tuple(Waypoint(position=p, frame="ned", dwell_s=dwell_s) for p in waypoints_ned)
    return MissionStateMachine(make_config(wps), waypoints_ned, HOME_NED)


def make_telem(**overrides: Any) -> Telemetry:
    """Telemetry with safe defaults (full battery, no abort) — override any field by keyword.

    Keyword-only by design: keeps the builder to a single parameter while every Telemetry field
    stays overridable, e.g. ``make_telem(now_s=1.0, abort_requested=True)``.
    """
    fields: dict[str, Any] = {
        "now_s": 0.0,
        "position_ned": (0.0, 0.0, 0.0),
        "armed": False,
        "offboard_active": False,
        "battery_remaining": 1.0,
        "abort_requested": False,
        "manual_takeover": False,
        "timed_out": False,
    }
    fields.update(overrides)
    return Telemetry(**fields)
