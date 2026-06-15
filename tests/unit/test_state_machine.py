"""Unit tests for the basic-mission MissionStateMachine (design §4.2.3, MC-4/MC-5, INF-M1).

Layer-A: ROS-free, deterministic. The clock is injected via ``Telemetry.now_s`` so
timing (tolerance+hold, hover duration) is exercised with no real wall clock and no
rclpy / Gazebo / PX4.

M1 scope is the basic happy path: IDLE -> ARMING -> TAKEOFF -> HOVER -> LANDING -> DONE.
Abort guards, WAYPOINT/DWELL, and RTH are M2 (plan M4).
"""

import pytest
from patrol_mission.config import AbortConfig, Completion, MissionConfig
from patrol_mission.state_machine import (
    MissionState,
    MissionStateMachine,
    Telemetry,
    local_position_usable,
    telemetry_fresh,
)

TAKEOFF_ALT = 5.0
HOVER_TIME = 10.0
TOL = 0.5
HOLD = 2.0
HOME_NED = (0.0, 0.0, -2.0)
# Takeoff target in NED: home xy, takeoff_alt_m AGL above home (down increases downward, so "up"
# subtracts from home's own down coordinate). Derived from HOME_NED so it tracks the AGL-from-home
# computation rather than assuming home sits at z=0 (Hermes Low #3).
TAKEOFF_NED = (HOME_NED[0], HOME_NED[1], HOME_NED[2] - TAKEOFF_ALT)


def _config() -> MissionConfig:
    return MissionConfig(
        takeoff_alt_m=TAKEOFF_ALT,
        hover_time_s=HOVER_TIME,
        completion=Completion(tolerance_m=TOL, hold_time_s=HOLD),
        abort=AbortConfig(low_battery_threshold=0.20),
        home_position=(0.0, 0.0, 2.0),
        home_frame="enu",
        waypoints=(),
    )


def _sm() -> MissionStateMachine:
    return MissionStateMachine(_config(), waypoints_ned=[], home_ned=HOME_NED)


def _telem(
    now_s=0.0, position_ned=(0.0, 0.0, 0.0), armed=False, offboard_active=False
) -> Telemetry:
    return Telemetry(
        now_s=now_s, position_ned=position_ned, armed=armed, offboard_active=offboard_active
    )


# TS-1: IDLE issues arm and advances to ARMING, streaming the takeoff setpoint from tick 0 so
# PX4's pre-offboard setpoint stream (A-2) is established before mode/arm.
def test_idle_issues_arm_and_advances_to_arming():
    nxt, cmd = _sm().tick(MissionState.IDLE, _telem())
    assert nxt is MissionState.ARMING
    assert cmd.arm is True
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-2: ARMING keeps requesting arm + offboard (and keeps streaming the setpoint, A-2) until both
# are confirmed, then -> TAKEOFF.
def test_arming_waits_for_armed_and_offboard():
    sm = _sm()
    nxt, cmd = sm.tick(MissionState.ARMING, _telem(armed=False, offboard_active=False))
    assert nxt is MissionState.ARMING
    assert cmd.arm is True
    assert cmd.set_offboard is True
    assert cmd.setpoint_ned == TAKEOFF_NED


def test_arming_advances_to_takeoff_when_armed_and_offboard():
    sm = _sm()
    nxt, cmd = sm.tick(MissionState.ARMING, _telem(armed=True, offboard_active=True))
    assert nxt is MissionState.TAKEOFF
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-3: TAKEOFF commands the altitude setpoint and holds until within tolerance for hold_time.
def test_takeoff_commands_altitude_setpoint():
    sm = _sm()
    nxt, cmd = sm.tick(MissionState.TAKEOFF, _telem(now_s=0.0, position_ned=(0.0, 0.0, 0.0)))
    assert nxt is MissionState.TAKEOFF
    assert cmd.setpoint_ned == TAKEOFF_NED


def test_takeoff_does_not_advance_before_hold_elapses():
    sm = _sm()
    # enter TAKEOFF at t=0 (inside tolerance immediately)
    sm.tick(MissionState.TAKEOFF, _telem(now_s=0.0, position_ned=TAKEOFF_NED))
    # still within tolerance but only 1.0 s < hold 2.0 s
    nxt, _ = sm.tick(MissionState.TAKEOFF, _telem(now_s=1.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF


def test_takeoff_advances_to_hover_after_tolerance_hold():
    sm = _sm()
    sm.tick(MissionState.TAKEOFF, _telem(now_s=0.0, position_ned=TAKEOFF_NED))
    nxt, cmd = sm.tick(MissionState.TAKEOFF, _telem(now_s=HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-3 (MC-5): leaving the tolerance ball resets the hold clock — never completes on equality alone.
def test_takeoff_hold_clock_resets_when_leaving_tolerance():
    sm = _sm()
    sm.tick(MissionState.TAKEOFF, _telem(now_s=0.0, position_ned=TAKEOFF_NED))
    # drift out of tolerance at t=1.0 (well beyond TOL on the z axis)
    sm.tick(MissionState.TAKEOFF, _telem(now_s=1.0, position_ned=(0.0, 0.0, 0.0)))
    # back inside at t=1.5; only 0.5 s of continuous hold by t=2.0 -> not complete
    sm.tick(MissionState.TAKEOFF, _telem(now_s=1.5, position_ned=TAKEOFF_NED))
    nxt, _ = sm.tick(MissionState.TAKEOFF, _telem(now_s=2.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF


# TS-4: HOVER holds the takeoff point for hover_time_s, then -> LANDING.
def test_hover_holds_then_lands():
    sm = _sm()
    sm.tick(MissionState.HOVER, _telem(now_s=100.0, position_ned=TAKEOFF_NED))
    nxt, cmd = sm.tick(
        MissionState.HOVER, _telem(now_s=100.0 + HOVER_TIME - 0.1, position_ned=TAKEOFF_NED)
    )
    assert nxt is MissionState.HOVER
    assert cmd.setpoint_ned == TAKEOFF_NED
    nxt, cmd = sm.tick(
        MissionState.HOVER, _telem(now_s=100.0 + HOVER_TIME, position_ned=TAKEOFF_NED)
    )
    assert nxt is MissionState.LANDING


# TS-5: LANDING issues land and holds until disarmed, then -> DONE (terminal).
def test_landing_issues_land_until_disarmed():
    sm = _sm()
    nxt, cmd = sm.tick(MissionState.LANDING, _telem(armed=True))
    assert nxt is MissionState.LANDING
    assert cmd.land is True


def test_landing_advances_to_done_when_disarmed():
    sm = _sm()
    nxt, _ = sm.tick(MissionState.LANDING, _telem(armed=False))
    assert nxt is MissionState.DONE


def test_done_is_terminal():
    sm = _sm()
    nxt, cmd = sm.tick(MissionState.DONE, _telem())
    assert nxt is MissionState.DONE
    assert cmd.arm is False
    assert cmd.land is False


# INF-M1: the whole basic mission runs deterministically end-to-end via injected telemetry.
def test_full_basic_mission_sequence():
    sm = _sm()
    state = MissionState.IDLE
    t = 0.0
    seen = [state]
    armed = False
    pos = (0.0, 0.0, 0.0)
    for _ in range(400):  # 40 s at 10 Hz — generous bound
        # crude SITL stand-in: arm once requested; snap toward the commanded setpoint
        nxt, cmd = sm.tick(
            state, _telem(now_s=t, position_ned=pos, armed=armed, offboard_active=armed)
        )
        if cmd.arm:
            armed = True
        if cmd.land:
            armed = False  # disarm completes the landing
        if cmd.setpoint_ned is not None:
            pos = cmd.setpoint_ned  # reach the setpoint immediately (unit-time abstraction)
        if nxt is not state:
            seen.append(nxt)
        state = nxt
        t += 0.1
        if state is MissionState.DONE:
            break
    assert state is MissionState.DONE
    assert seen == [
        MissionState.IDLE,
        MissionState.ARMING,
        MissionState.TAKEOFF,
        MissionState.HOVER,
        MissionState.LANDING,
        MissionState.DONE,
    ]


# M3 (Hermes Low #3): the takeoff target is takeoff_alt_m AGL above home — it incorporates home's
# own NED-down altitude, so it is correct even when home does not sit at the EKF-origin ground.
def test_takeoff_target_is_takeoff_alt_above_home():
    sm = MissionStateMachine(_config(), waypoints_ned=[], home_ned=(1.0, 2.0, -3.0))
    _, cmd = sm.tick(MissionState.IDLE, _telem())
    assert cmd.setpoint_ned == (1.0, 2.0, -3.0 - TAKEOFF_ALT)


# M3 (Hermes Medium #1): the basic machine consumes no waypoints — a non-empty list (a patrol
# mission handed to the basic machine) must fail loud at construction, never be silently ignored.
def test_basic_mission_rejects_waypoints():
    with pytest.raises(ValueError, match="no waypoints"):
        MissionStateMachine(_config(), waypoints_ned=[(1.0, 2.0, -3.0)], home_ned=HOME_NED)


# Review #1: reset_timing() restarts the active state's time-based windows so the machine never
# completes a HOVER / tolerance-hold on wall-time that elapsed while the node was NOT ticking it
# (the stale-telemetry pause). After a reset mid-HOVER the hover clock starts over from the resume
# time, so a now_s that already spans hover_time_s does not complete until a fresh full window.
def test_reset_timing_restarts_hover_window():
    sm = _sm()
    sm.tick(MissionState.HOVER, _telem(now_s=0.0, position_ned=TAKEOFF_NED))  # enter HOVER at t=0
    sm.tick(MissionState.HOVER, _telem(now_s=5.0, position_ned=TAKEOFF_NED))  # 5 s of real hover
    sm.reset_timing()  # node observed a stale->fresh resume edge: restart the window
    # now_s has jumped past hover_time_s (the blackout), but the window restarts on the next tick...
    nxt, _ = sm.tick(MissionState.HOVER, _telem(now_s=100.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER  # does NOT complete on the unobserved elapsed wall-time
    # ...and only completes after a fresh full hover_time_s of observed ticking from the resume.
    nxt, _ = sm.tick(MissionState.HOVER, _telem(now_s=100.0 + HOVER_TIME, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.LANDING


# Review #1: reset_timing() also restarts the TAKEOFF tolerance-hold clock — continuous in-tolerance
# evidence must be re-established after a resume, never credited across the unobserved gap.
def test_reset_timing_restarts_tolerance_hold():
    sm = _sm()
    sm.tick(MissionState.TAKEOFF, _telem(now_s=0.0, position_ned=TAKEOFF_NED))  # inside tol at t=0
    sm.reset_timing()
    nxt, _ = sm.tick(MissionState.TAKEOFF, _telem(now_s=HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF  # hold clock restarted at the resume tick, not yet elapsed
    nxt, _ = sm.tick(MissionState.TAKEOFF, _telem(now_s=2 * HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER


# M3 (Hermes Medium): a cached PX4 sample is usable only while its age is within the freshness
# timeout — once /fmu/out/* stops updating, the node must stop advancing on the frozen fix.
@pytest.mark.parametrize(
    ("age_s", "timeout_s", "expected"),
    [(0.0, 1.0, True), (1.0, 1.0, True), (1.0001, 1.0, False), (5.0, 1.0, False)],
)
def test_telemetry_fresh_within_timeout(age_s, timeout_s, expected):
    assert telemetry_fresh(age_s, timeout_s) is expected


# M1 (Hermes Medium #1): a position fix is usable only when PX4 reports BOTH the
# horizontal and vertical EKF estimate valid — the node's precondition for arming.
@pytest.mark.parametrize(
    ("xy_valid", "z_valid", "expected"),
    [(True, True, True), (True, False, False), (False, True, False), (False, False, False)],
)
def test_local_position_usable_requires_both_flags(xy_valid, z_valid, expected):
    assert local_position_usable(xy_valid, z_valid) is expected
