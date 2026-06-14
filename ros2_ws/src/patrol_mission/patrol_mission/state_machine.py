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
    """What the node should issue this tick (design §4.2.3).

    The mission state is NOT carried here: ``tick()`` returns ``(next_state, command)``, so the
    returned :class:`MissionState` enum is the single source of truth. When ``/patrol/mission_state``
    publication lands (M4) it derives from that returned enum, not a duplicated string on the command
    (Hermes Low — avoid two sources of truth).
    """

    arm: bool = False
    set_offboard: bool = False
    land: bool = False
    setpoint_ned: Point | None = None
    yaw: float = 0.0


@dataclass
class _Progress:
    """Mutable per-run bookkeeping carried between ticks."""

    last_state: MissionState | None = None
    state_entered_s: float = 0.0
    inside_since_s: float | None = None  # first time continuously inside tolerance (MC-5)


def local_position_usable(xy_valid: bool, z_valid: bool) -> bool:
    """Precondition for trusting a PX4 ``VehicleLocalPosition`` fix this tick.

    PX4 publishes ``xy_valid`` / ``z_valid`` (VehicleLocalPosition.msg) to flag
    when the EKF's horizontal / vertical estimate is converged. The node must not
    feed position into the machine — i.e. must not begin offboard/arm sequencing —
    until both are true, or the mission could arm on an unconverged estimate. Pure
    (no rclpy) so the gate is Layer-A testable without ROS.
    """
    return xy_valid and z_valid


def telemetry_fresh(age_s: float, timeout_s: float) -> bool:
    """Whether a cached PX4 sample of ``age_s`` seconds is still fresh enough to act on.

    The node caches the latest ``/fmu/out/*`` sample and would otherwise act on it indefinitely; if
    PX4 stops publishing after a valid sample, the mission must not keep advancing on a frozen fix
    (Hermes Medium). The node measures ``age_s`` from message receipt and skips state-machine
    progression while any required stream is stale. Pure (no rclpy) so the gate is Layer-A testable.
    """
    return age_s <= timeout_s


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
        # Basic mission (M3) flies takeoff -> hover -> land and consumes no waypoints. Fail loud
        # rather than silently ignore them: config parses inline waypoints as M4 forward-schema, so
        # a non-empty list reaching here means a patrol mission was handed to the basic machine.
        # WAYPOINT/DWELL sequencing thickens this machine in M4 and removes this guard.
        if waypoints_ned:
            raise ValueError(
                f"basic mission (M3) accepts no waypoints, got {len(waypoints_ned)}; "
                "WAYPOINT sequencing lands in M4"
            )
        self._home = home_ned
        self._p = _Progress()
        # Basic-mission takeoff target: home x/y, takeoff_alt_m AGL above home. NED down increases
        # downward, so "alt above home" subtracts from home's own down coordinate (home_ned[2] -
        # alt). This is correct for any home altitude; for the shipped mission_basic.yaml (home
        # z=0) it reduces to -takeoff_alt_m, so the flown numbers are unchanged.
        self._takeoff_ned: Point = (home_ned[0], home_ned[1], home_ned[2] - config.takeoff_alt_m)
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

    # --- per-state handlers -------------------------------------------------

    def _idle(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        # Carry the takeoff setpoint from the very first tick: PX4 only accepts the offboard
        # switch once it has seen BOTH the OffboardControlMode stream AND a TrajectorySetpoint
        # stream during warmup (A-2). The node gates arm/set-offboard behind the warmup window
        # but streams this setpoint immediately, so the dual stream is established before arm.
        return MissionState.ARMING, Command(arm=True, setpoint_ned=self._takeoff_ned)

    def _arming(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if telem.armed and telem.offboard_active:
            return MissionState.TAKEOFF, Command(setpoint_ned=self._takeoff_ned)
        # Keep streaming the takeoff setpoint while waiting for arm+offboard confirmation, so the
        # pre-offboard setpoint stream PX4 requires (A-2) is never interrupted.
        return MissionState.ARMING, Command(
            arm=True, set_offboard=True, setpoint_ned=self._takeoff_ned
        )

    def _takeoff(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if self._within_tolerance_for_hold(telem, self._takeoff_ned):
            return MissionState.HOVER, Command(setpoint_ned=self._takeoff_ned)
        return MissionState.TAKEOFF, Command(setpoint_ned=self._takeoff_ned)

    def _hover(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if (telem.now_s - self._p.state_entered_s) >= self._cfg.hover_time_s:
            return MissionState.LANDING, Command(land=True)
        return MissionState.HOVER, Command(setpoint_ned=self._takeoff_ned)

    def _landing(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if not telem.armed:
            return MissionState.DONE, Command()
        return MissionState.LANDING, Command(land=True)

    def _done(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        return MissionState.DONE, Command()
