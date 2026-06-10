"""The basic-mission MissionStateMachine (design §4.2.3, OQ-1, MC-4/MC-5).

Hand-rolled ``enum`` + ``tick()`` dispatch (OQ-1: no state-machine library). The
machine owns every mission *decision* and holds **no rclpy import and no I/O** —
the node injects :class:`Telemetry` (including the clock via ``now_s``) and
consumes the returned :class:`Command`. This is what makes every transition
unit-testable in <5 s with no ROS/Gazebo/PX4 (INF-M1).

M1 implements the basic happy path::

    IDLE -> ARMING -> TAKEOFF -> HOVER -> LANDING -> DONE

Abort guards (external-signal / low-battery / scaffolded), WAYPOINT/DWELL
sequencing, and the explicit RTH home-waypoint sequence are M2 (plan M4); they
thicken this same machine without changing the ``tick()`` contract.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from patrol_mission.config import MissionConfig
    from patrol_mission.frames import Point


class MissionState(Enum):
    IDLE = auto()  # pre-arm; node publishes keepalive only
    ARMING = auto()  # arm + set-offboard requested; waiting for confirmation
    TAKEOFF = auto()  # climbing to takeoff_alt_m
    HOVER = auto()  # holding the takeoff point for hover_time_s
    LANDING = auto()  # land command issued; descending
    DONE = auto()  # disarmed on the ground; terminal


@dataclass(frozen=True)
class Telemetry:
    """Everything the machine needs about the world this tick (design §4.2.3).

    The clock is injected via ``now_s`` so timing is deterministic in tests.
    """

    now_s: float
    position_ned: Point
    armed: bool
    offboard_active: bool


@dataclass(frozen=True)
class Command:
    """What the node should issue this tick (design §4.2.3)."""

    arm: bool = False
    set_offboard: bool = False
    land: bool = False
    setpoint_ned: Point | None = None
    yaw: float = 0.0
    mission_state: str = ""  # published verbatim to /patrol/mission_state (M4)


@dataclass
class _Progress:
    """Mutable per-run bookkeeping carried between ticks."""

    last_state: MissionState | None = None
    state_entered_s: float = 0.0
    inside_since_s: float | None = None  # first time continuously inside tolerance (MC-5)


def _distance(a: Point, b: Point) -> float:
    return math.dist(a, b)


class MissionStateMachine:
    def __init__(
        self,
        config: MissionConfig,
        waypoints_ned: list[Point],
        home_ned: Point,
    ) -> None:
        self._cfg = config
        self._wps = waypoints_ned  # unused in M1 (basic mission); WAYPOINT sequencing is M4
        self._home = home_ned
        self._p = _Progress()
        # Basic-mission takeoff target: home x/y at takeoff altitude (NED down is negative).
        self._takeoff_ned: Point = (home_ned[0], home_ned[1], -config.takeoff_alt_m)
        self._dispatch: dict[MissionState, Callable[[Telemetry], tuple[MissionState, Command]]] = {
            MissionState.IDLE: self._idle,
            MissionState.ARMING: self._arming,
            MissionState.TAKEOFF: self._takeoff,
            MissionState.HOVER: self._hover,
            MissionState.LANDING: self._landing,
            MissionState.DONE: self._done,
        }

    def tick(self, state: MissionState, telem: Telemetry) -> tuple[MissionState, Command]:
        """Advance one tick: returns ``(next_state, command)``. Pure — no I/O.

        On entry to a new state (the node feeds the returned ``next_state`` back),
        per-state bookkeeping (entry timestamp, tolerance-hold clock) is reset.
        """
        if state is not self._p.last_state:
            self._p.last_state = state
            self._p.state_entered_s = telem.now_s
            self._p.inside_since_s = None
        return self._dispatch[state](telem)

    def _within_tolerance_for_hold(self, telem: Telemetry, target: Point) -> bool:
        """MC-5: complete iff continuously within tolerance for hold_time_s — never on equality."""
        if _distance(telem.position_ned, target) <= self._cfg.completion.tolerance_m:
            if self._p.inside_since_s is None:
                self._p.inside_since_s = telem.now_s
            return (telem.now_s - self._p.inside_since_s) >= self._cfg.completion.hold_time_s
        self._p.inside_since_s = None  # left the tolerance ball; reset the hold clock
        return False

    @staticmethod
    def _cmd(state: MissionState, **kwargs: object) -> Command:
        return Command(mission_state=state.name, **kwargs)  # type: ignore[arg-type]

    # --- per-state handlers -------------------------------------------------

    def _idle(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        return MissionState.ARMING, self._cmd(MissionState.ARMING, arm=True)

    def _arming(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if telem.armed and telem.offboard_active:
            return MissionState.TAKEOFF, self._cmd(
                MissionState.TAKEOFF, setpoint_ned=self._takeoff_ned
            )
        return MissionState.ARMING, self._cmd(MissionState.ARMING, arm=True, set_offboard=True)

    def _takeoff(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if self._within_tolerance_for_hold(telem, self._takeoff_ned):
            return MissionState.HOVER, self._cmd(MissionState.HOVER, setpoint_ned=self._takeoff_ned)
        return MissionState.TAKEOFF, self._cmd(MissionState.TAKEOFF, setpoint_ned=self._takeoff_ned)

    def _hover(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if (telem.now_s - self._p.state_entered_s) >= self._cfg.hover_time_s:
            return MissionState.LANDING, self._cmd(MissionState.LANDING, land=True)
        return MissionState.HOVER, self._cmd(MissionState.HOVER, setpoint_ned=self._takeoff_ned)

    def _landing(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if not telem.armed:
            return MissionState.DONE, self._cmd(MissionState.DONE)
        return MissionState.LANDING, self._cmd(MissionState.LANDING, land=True)

    def _done(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        return MissionState.DONE, self._cmd(MissionState.DONE)
