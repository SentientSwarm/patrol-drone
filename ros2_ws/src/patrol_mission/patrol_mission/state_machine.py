"""The MissionStateMachine — every mission decision, ROS-free (design §4.2.3, OQ-1, MC-4/5/6/9).

Hand-rolled ``enum`` + ``tick()`` dispatch (OQ-1: no state-machine library). The machine owns
every mission *decision* and holds **no rclpy import and no I/O** — the node injects
:class:`Telemetry` (including the clock via ``now_s``) and consumes the returned :class:`Command`.
This is what makes every transition — including all four abort guards — unit-testable in <5 s with
no ROS/Gazebo/PX4 (INF-M1, AC-8).

M4 (plan-M4 / docset-02 M2) thickens the M3 basic path into a real patrol::

    IDLE -> ARMING -> TAKEOFF -> HOVER -> WAYPOINT <-> DWELL ... -> RTH -> LANDING -> DONE

with the abort safety floor on top: abort guards are evaluated **first every tick** (highest
precedence), so a latched abort pre-empts any normal transition and routes ABORT -> RTH. External-
signal and low-battery aborts are live; manual-takeover and timeout are scaffolded (their guards
exist and are unit-tested, but the telemetry that fires them is never True in SITL — Phase 2 RC).

When a mission has no waypoints (the basic mission), HOVER routes straight to RTH (design §4.2.3
table), so the basic mission returns to home and lands rather than landing in place.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass, field
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
    WAYPOINT = auto()  # flying toward waypoints_ned[i]
    DWELL = auto()  # arrived at waypoints_ned[i]; holding dwell_s (capture trigger for 04)
    RTH = auto()  # explicit home-waypoint offboard sequence (OQ-8) — no PX4 RTL
    LANDING = auto()  # land command issued; descending
    ABORT = auto()  # abort latched + observable for one tick; routes to RTH
    DONE = auto()  # disarmed on the ground; terminal


class AbortReason(Enum):
    NONE = auto()
    EXTERNAL_SIGNAL = auto()  # /patrol/abort True (MC-6) — live
    LOW_BATTERY = auto()  # battery_remaining < threshold (MC-6/OQ-6) — live
    MANUAL_TAKEOVER = auto()  # scaffold (MC-11) — never True in SITL
    TIMEOUT = auto()  # scaffold (MC-11) — never True in SITL


@dataclass(frozen=True)
class Telemetry:
    """Everything the machine needs about the world this tick (design §4.2.3).

    The clock is injected via ``now_s`` so timing is deterministic in tests. ``battery_remaining``
    and ``abort_requested`` are required (the node always wires them — a default would mask a node
    that forgot to feed battery/abort); the two scaffold flags default False (never True in SITL).
    """

    now_s: float
    position_ned: Point
    armed: bool
    offboard_active: bool
    battery_remaining: float  # 0.0..1.0 (BatteryStatus.remaining)
    abort_requested: bool  # latched value from /patrol/abort
    manual_takeover: bool = False  # scaffold — always False in SITL
    timed_out: bool = False  # scaffold — always False in SITL


@dataclass(frozen=True)
class Command:
    """What the node should issue this tick (design §4.2.3).

    The mission state is NOT carried here: ``tick()`` returns ``(next_state, command)``, so the
    returned :class:`MissionState` enum is the single source of truth and the node publishes
    ``next_state.name`` to ``/patrol/mission_state`` (one source — avoid two). ``current_waypoint``
    IS carried because it is information the enum does not encode: the active waypoint index,
    published to ``/patrol/current_waypoint`` (``-1`` outside WAYPOINT/DWELL).
    """

    arm: bool = False
    set_offboard: bool = False
    land: bool = False
    setpoint_ned: Point | None = None
    yaw: float = 0.0
    current_waypoint: int = -1


@dataclass
class _Progress:
    """Mutable per-run bookkeeping carried between ticks."""

    last_state: MissionState | None = None
    state_entered_s: float = 0.0
    inside_since_s: float | None = None  # first time continuously inside tolerance (MC-5)
    waypoint_index: int = 0  # index of the waypoint currently being flown / dwelled
    abort_reason: AbortReason = field(default=AbortReason.NONE)


# States from which an abort guard must NOT re-fire: once aborting/returning/landing/done the
# mission is already heading home or terminal, so re-latching would loop. This exclusion also
# means a momentary external-abort pulse still drives the full RTH (the abort "sticks").
_NON_ABORTABLE = frozenset(
    {MissionState.ABORT, MissionState.RTH, MissionState.LANDING, MissionState.DONE}
)


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
        # waypoints_ned is config.waypoints resolved to NED at the single frame boundary (MC-7), so
        # the two lists are parallel; dwell_s for waypoint i is config.waypoints[i].dwell_s. A length
        # mismatch is a node wiring bug — fail loud rather than fly a misaligned route.
        if len(waypoints_ned) != len(config.waypoints):
            raise ValueError(
                f"waypoints_ned ({len(waypoints_ned)}) must align with "
                f"config.waypoints ({len(config.waypoints)})"
            )
        self._wps = waypoints_ned
        self._home = home_ned
        self._p = _Progress()
        # Takeoff target: home x/y, takeoff_alt_m AGL above home. NED down increases downward, so
        # "alt above home" subtracts from home's own down coordinate (home_ned[2] - alt). Correct
        # for any home altitude.
        self._takeoff_ned: Point = (home_ned[0], home_ned[1], home_ned[2] - config.takeoff_alt_m)
        self._dispatch: dict[MissionState, Callable[[Telemetry], tuple[MissionState, Command]]] = {
            MissionState.IDLE: self._idle,
            MissionState.ARMING: self._arming,
            MissionState.TAKEOFF: self._takeoff,
            MissionState.HOVER: self._hover,
            MissionState.WAYPOINT: self._waypoint,
            MissionState.DWELL: self._dwell,
            MissionState.RTH: self._rth,
            MissionState.LANDING: self._landing,
            MissionState.ABORT: self._abort,
            MissionState.DONE: self._done,
        }

    def tick(self, state: MissionState, telem: Telemetry) -> tuple[MissionState, Command]:
        """Advance one tick: returns ``(next_state, command)``. Pure — no I/O.

        Abort guards are evaluated FIRST (highest precedence): a latched abort pre-empts the normal
        transition and surfaces ABORT this tick (observable on ``/patrol/mission_state``); the ABORT
        handler then routes to RTH on the next tick. On entry to a new state, per-state bookkeeping
        (entry timestamp, tolerance-hold clock) is reset.
        """
        reason = self._abort_reason(telem)
        if reason is not AbortReason.NONE and state not in _NON_ABORTABLE:
            self._p.abort_reason = reason
            self._enter_if_new(MissionState.ABORT, telem)
            return MissionState.ABORT, Command(setpoint_ned=self._home)
        self._enter_if_new(state, telem)
        return self._dispatch[state](telem)

    def reset_timing(self) -> None:
        """Restart the active state's time-based windows on the next ``tick()``.

        The time-based completions (HOVER/DWELL duration, TAKEOFF/WAYPOINT/RTH tolerance-hold)
        measure elapsed wall time via ``telem.now_s`` deltas. If the node stops ticking the machine
        while ``/fmu/out`` telemetry is stale (it pauses progression — see the node's stale gate) and
        later resumes, the injected ``now_s`` has jumped forward by the whole blackout, so a window
        could otherwise "complete" on time that elapsed with no fresh in-tolerance evidence. The node
        calls this on the stale->fresh resume edge so the current state re-enters fresh (entry
        timestamp and the tolerance-hold clock reset on the next tick). Conservative by design: it
        can only ever delay a completion, never bring one forward.
        """
        self._p.last_state = None

    def _abort_reason(self, telem: Telemetry) -> AbortReason:
        """Highest-precedence abort condition active this tick (external > battery > scaffolds)."""
        if telem.abort_requested:
            return AbortReason.EXTERNAL_SIGNAL
        if telem.battery_remaining < self._cfg.abort.low_battery_threshold:
            return AbortReason.LOW_BATTERY
        if telem.manual_takeover:  # scaffold (MC-11) — never True in SITL
            return AbortReason.MANUAL_TAKEOVER
        if telem.timed_out:  # scaffold (MC-11) — never True in SITL
            return AbortReason.TIMEOUT
        return AbortReason.NONE

    def _enter_if_new(self, state: MissionState, telem: Telemetry) -> None:
        if state is not self._p.last_state:
            self._p.last_state = state
            self._p.state_entered_s = telem.now_s
            self._p.inside_since_s = None

    def _within_tolerance_for_hold(self, telem: Telemetry, target: Point) -> bool:
        """MC-5: complete iff continuously within tolerance for hold_time_s — never on equality."""
        if _distance(telem.position_ned, target) <= self._cfg.completion.tolerance_m:
            if self._p.inside_since_s is None:
                self._p.inside_since_s = telem.now_s
            return (telem.now_s - self._p.inside_since_s) >= self._cfg.completion.hold_time_s
        self._p.inside_since_s = None  # left the tolerance ball; reset the hold clock
        return False

    def _elapsed_since_entry(self, telem: Telemetry, duration_s: float) -> bool:
        return (telem.now_s - self._p.state_entered_s) >= duration_s

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
        if self._elapsed_since_entry(telem, self._cfg.hover_time_s):
            return self._begin_route()
        return MissionState.HOVER, Command(setpoint_ned=self._takeoff_ned)

    def _begin_route(self) -> tuple[MissionState, Command]:
        """After hover: head to the first waypoint, or RTH if the mission has none (design §4.2.3)."""
        if self._wps:
            self._p.waypoint_index = 0
            return MissionState.WAYPOINT, Command(setpoint_ned=self._wps[0], current_waypoint=0)
        return MissionState.RTH, Command(setpoint_ned=self._home)

    def _waypoint(self, telem: Telemetry) -> tuple[MissionState, Command]:
        i = self._p.waypoint_index
        target = self._wps[i]
        if self._within_tolerance_for_hold(telem, target):
            return MissionState.DWELL, Command(setpoint_ned=target, current_waypoint=i)
        return MissionState.WAYPOINT, Command(setpoint_ned=target, current_waypoint=i)

    def _dwell(self, telem: Telemetry) -> tuple[MissionState, Command]:
        i = self._p.waypoint_index
        if self._elapsed_since_entry(telem, self._cfg.waypoints[i].dwell_s):
            return self._advance_from_dwell(i)
        # Hold the waypoint while dwelling; DWELL + this index is 04's once-per-checkpoint capture
        # trigger (OQ-7), surfaced on /patrol/{mission_state,current_waypoint} by the node.
        return MissionState.DWELL, Command(setpoint_ned=self._wps[i], current_waypoint=i)

    def _advance_from_dwell(self, i: int) -> tuple[MissionState, Command]:
        """After dwelling at waypoint i: fly the next waypoint, or RTH once the last is done."""
        if i + 1 < len(self._wps):
            self._p.waypoint_index = i + 1
            return MissionState.WAYPOINT, Command(
                setpoint_ned=self._wps[i + 1], current_waypoint=i + 1
            )
        return MissionState.RTH, Command(setpoint_ned=self._home)

    def _rth(self, telem: Telemetry) -> tuple[MissionState, Command]:
        # Explicit home-waypoint offboard sequence (OQ-8): fly home_ned, settle, then land. No PX4
        # RTL mode handoff — control authority stays in the state machine.
        if self._within_tolerance_for_hold(telem, self._home):
            return MissionState.LANDING, Command(land=True)
        return MissionState.RTH, Command(setpoint_ned=self._home)

    def _abort(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        # ABORT was surfaced (observable) for one tick by tick(); route to the explicit home-waypoint
        # return. The latched abort_reason stays in _Progress for the record.
        return MissionState.RTH, Command(setpoint_ned=self._home)

    def _landing(self, telem: Telemetry) -> tuple[MissionState, Command]:
        if not telem.armed:
            return MissionState.DONE, Command()
        return MissionState.LANDING, Command(land=True)

    def _done(self, _telem: Telemetry) -> tuple[MissionState, Command]:
        return MissionState.DONE, Command()
