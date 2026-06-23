"""Unit tests for the MissionStateMachine (design §4.2.3, MC-4/MC-5, INF-M1).

Layer-A: ROS-free, deterministic. The clock is injected via ``Telemetry.now_s`` so timing
(tolerance+hold, hover/dwell duration) is exercised with no real wall clock and no
rclpy / Gazebo / PX4.

This file covers the non-abort path: arm/takeoff/hover, waypoint→dwell sequencing, and the
RTH home-waypoint sequence (M4). The four abort guards + abort precedence live in
``test_abort.py`` (AC-7/AC-8). Shared config/telemetry/SM builders live in ``mission_builders``.
"""

import pytest
from mission_builders import (
    HOLD,
    HOME_NED,
    HOVER_TIME,
    TAKEOFF_ALT,
    TAKEOFF_NED,
    WP0,
    WP1,
    make_config,
    make_patrol_sm,
    make_sm,
    make_telem,
)
from patrol_mission.state_machine import (
    MissionState,
    MissionStateMachine,
    local_position_usable,
    telemetry_fresh,
)


# TS-1: IDLE issues arm and advances to ARMING, streaming the takeoff setpoint from tick 0 so
# PX4's pre-offboard setpoint stream (A-2) is established before mode/arm.
def test_idle_issues_arm_and_advances_to_arming():
    nxt, cmd = make_sm().tick(MissionState.IDLE, make_telem())
    assert nxt is MissionState.ARMING
    assert cmd.arm is True
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-2: ARMING keeps requesting arm + offboard (and keeps streaming the setpoint, A-2) until both
# are confirmed, then -> TAKEOFF.
def test_arming_waits_for_armed_and_offboard():
    nxt, cmd = make_sm().tick(MissionState.ARMING, make_telem(armed=False, offboard_active=False))
    assert nxt is MissionState.ARMING
    assert cmd.arm is True
    assert cmd.set_offboard is True
    assert cmd.setpoint_ned == TAKEOFF_NED


def test_arming_advances_to_takeoff_when_armed_and_offboard():
    nxt, cmd = make_sm().tick(MissionState.ARMING, make_telem(armed=True, offboard_active=True))
    assert nxt is MissionState.TAKEOFF
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-3: TAKEOFF commands the altitude setpoint and holds until within tolerance for hold_time.
def test_takeoff_commands_altitude_setpoint():
    nxt, cmd = make_sm().tick(MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=(0, 0, 0)))
    assert nxt is MissionState.TAKEOFF
    assert cmd.setpoint_ned == TAKEOFF_NED


def test_takeoff_does_not_advance_before_hold_elapses():
    sm = make_sm()
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    nxt, _ = sm.tick(MissionState.TAKEOFF, make_telem(now_s=1.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF  # within tolerance but only 1.0 s < hold 2.0 s


def test_takeoff_advances_to_hover_after_tolerance_hold():
    sm = make_sm()
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    nxt, cmd = sm.tick(MissionState.TAKEOFF, make_telem(now_s=HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER
    assert cmd.setpoint_ned == TAKEOFF_NED


# TS-3 (MC-5): leaving the tolerance ball resets the hold clock — never completes on equality alone.
def test_takeoff_hold_clock_resets_when_leaving_tolerance():
    sm = make_sm()
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=1.0, position_ned=(0, 0, 0)))  # drift out
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=1.5, position_ned=TAKEOFF_NED))  # back inside
    nxt, _ = sm.tick(MissionState.TAKEOFF, make_telem(now_s=2.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF  # only 0.5 s of continuous hold by t=2.0


# TS-5: HOVER holds the takeoff point for hover_time_s, then routes to the route start. With NO
# waypoints (basic mission) that is RTH (design §4.2.3 table) — return home, then land.
def test_hover_no_waypoints_routes_to_rth():
    sm = make_sm()
    sm.tick(MissionState.HOVER, make_telem(now_s=100.0, position_ned=TAKEOFF_NED))
    nxt, _ = sm.tick(MissionState.HOVER, make_telem(now_s=109.9, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER
    nxt, cmd = sm.tick(MissionState.HOVER, make_telem(now_s=110.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.RTH
    assert cmd.setpoint_ned == HOME_NED


# TS-5: HOVER with waypoints routes to the first WAYPOINT (index 0).
def test_hover_with_waypoints_routes_to_first_waypoint():
    sm = make_patrol_sm([WP0, WP1])
    sm.tick(MissionState.HOVER, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    nxt, cmd = sm.tick(MissionState.HOVER, make_telem(now_s=HOVER_TIME, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.WAYPOINT
    assert cmd.current_waypoint == 0
    assert cmd.setpoint_ned == WP0


# TS-1: WAYPOINT commands its target and advances to DWELL on continuous-in-tolerance-for-hold.
def test_waypoint_advances_to_dwell_on_tolerance_hold():
    sm = make_patrol_sm([WP0, WP1])
    sm.tick(MissionState.WAYPOINT, make_telem(now_s=0.0, position_ned=WP0))
    nxt, cmd = sm.tick(MissionState.WAYPOINT, make_telem(now_s=HOLD, position_ned=WP0))
    assert nxt is MissionState.DWELL
    assert cmd.current_waypoint == 0
    assert cmd.setpoint_ned == WP0


def test_waypoint_does_not_advance_before_hold():
    sm = make_patrol_sm([WP0])
    sm.tick(MissionState.WAYPOINT, make_telem(now_s=0.0, position_ned=WP0))
    nxt, _ = sm.tick(MissionState.WAYPOINT, make_telem(now_s=1.0, position_ned=WP0))
    assert nxt is MissionState.WAYPOINT


# TS-12 (MC-5): leaving the tolerance ball resets the waypoint hold clock.
def test_waypoint_hold_clock_resets_when_leaving_tolerance():
    sm = make_patrol_sm([WP0])
    sm.tick(MissionState.WAYPOINT, make_telem(now_s=0.0, position_ned=WP0))
    sm.tick(MissionState.WAYPOINT, make_telem(now_s=1.0, position_ned=(0, 0, -2)))  # left ball
    sm.tick(MissionState.WAYPOINT, make_telem(now_s=1.5, position_ned=WP0))  # back inside
    nxt, _ = sm.tick(MissionState.WAYPOINT, make_telem(now_s=2.0, position_ned=WP0))
    assert nxt is MissionState.WAYPOINT  # only 0.5 s of continuous hold


# TS-2: DWELL holds the waypoint for dwell_s, then advances to the next WAYPOINT (index+1).
def test_dwell_holds_then_advances_to_next_waypoint():
    sm = make_patrol_sm([WP0, WP1], dwell_s=3.0)
    sm.tick(MissionState.DWELL, make_telem(now_s=0.0, position_ned=WP0))
    nxt, cmd = sm.tick(MissionState.DWELL, make_telem(now_s=3.0, position_ned=WP0))
    assert nxt is MissionState.WAYPOINT
    assert cmd.current_waypoint == 1
    assert cmd.setpoint_ned == WP1


# TS-SIM4: WAYPOINT/DWELL emit the per-waypoint NED yaw the node supplies (yaw-to-tag), and default to
# 0.0 (hold North) when no yaw list is given — the pre-SIM-4 behavior, so non-checkpoint routes are
# unaffected. A yaw list that does not align 1:1 with the waypoints is a node wiring bug — fail loud.
_YAWS_NED = [0.5, -1.25]


@pytest.mark.parametrize("state", [MissionState.WAYPOINT, MissionState.DWELL])
@pytest.mark.parametrize(
    ("yaws", "expected_yaw"),
    [(_YAWS_NED, _YAWS_NED[0]), (None, 0.0)],  # supplied list -> faces the tag; None -> hold North
    ids=["per_waypoint_yaw", "default_north"],
)
def test_waypoint_and_dwell_emit_yaw(state, yaws, expected_yaw):
    sm = make_patrol_sm([WP0, WP1], waypoint_yaws_ned=yaws)
    _, cmd = sm.tick(state, make_telem(now_s=0.0, position_ned=WP0))
    assert cmd.yaw == expected_yaw


def test_misaligned_yaw_list_raises():
    with pytest.raises(ValueError, match="waypoint_yaws_ned"):
        make_patrol_sm([WP0, WP1], waypoint_yaws_ned=[0.0])


def test_dwell_holds_target_before_elapsed():
    sm = make_patrol_sm([WP0, WP1], dwell_s=3.0)
    sm.tick(MissionState.DWELL, make_telem(now_s=0.0, position_ned=WP0))
    nxt, cmd = sm.tick(MissionState.DWELL, make_telem(now_s=1.0, position_ned=WP0))
    assert nxt is MissionState.DWELL
    assert cmd.current_waypoint == 0  # OQ-7 capture trigger: DWELL + active index
    assert cmd.setpoint_ned == WP0


# TS-3: DWELL on the LAST waypoint routes to RTH (no next waypoint).
def test_dwell_on_last_waypoint_routes_to_rth():
    sm = make_patrol_sm([WP0], dwell_s=3.0)
    sm.tick(MissionState.DWELL, make_telem(now_s=0.0, position_ned=WP0))
    nxt, cmd = sm.tick(MissionState.DWELL, make_telem(now_s=3.0, position_ned=WP0))
    assert nxt is MissionState.RTH
    assert cmd.setpoint_ned == HOME_NED


# TS-4: RTH commands the home setpoint until within tolerance for hold, then -> LANDING (issues
# land). Explicit home-waypoint sequence — no PX4 RTL handoff (OQ-8).
def test_rth_commands_home_until_settled():
    nxt, cmd = make_sm().tick(MissionState.RTH, make_telem(now_s=0.0, position_ned=(0, 0, -7)))
    assert nxt is MissionState.RTH
    assert cmd.setpoint_ned == HOME_NED


def test_rth_advances_to_landing_on_tolerance_hold():
    sm = make_sm()
    sm.tick(MissionState.RTH, make_telem(now_s=0.0, position_ned=HOME_NED))
    nxt, cmd = sm.tick(MissionState.RTH, make_telem(now_s=HOLD, position_ned=HOME_NED))
    assert nxt is MissionState.LANDING
    assert cmd.land is True


# TS-5: LANDING issues land and holds until disarmed, then -> DONE (terminal).
def test_landing_issues_land_until_disarmed():
    nxt, cmd = make_sm().tick(MissionState.LANDING, make_telem(armed=True))
    assert nxt is MissionState.LANDING
    assert cmd.land is True


def test_landing_advances_to_done_when_disarmed():
    nxt, _ = make_sm().tick(MissionState.LANDING, make_telem(armed=False))
    assert nxt is MissionState.DONE


def test_done_is_terminal():
    nxt, cmd = make_sm().tick(MissionState.DONE, make_telem())
    assert nxt is MissionState.DONE
    assert cmd.arm is False
    assert cmd.land is False


def _run_to_done(sm, *, max_ticks: int) -> list[MissionState]:
    """Drive the machine from IDLE with a crude unit-time SITL stand-in; return the states seen.

    Arms once requested, disarms once land is issued (completing the landing), and snaps position
    to the commanded setpoint each tick (the unit-time abstraction — no flight dynamics in the
    decision layer). Records each distinct state entered.
    """
    state = MissionState.IDLE
    seen = [state]
    armed = False
    pos = (0.0, 0.0, 0.0)
    t = 0.0
    for _ in range(max_ticks):
        nxt, cmd = sm.tick(
            state, make_telem(now_s=t, position_ned=pos, armed=armed, offboard_active=armed)
        )
        if cmd.arm:
            armed = True
        if cmd.land:
            armed = False
        if cmd.setpoint_ned is not None:
            pos = cmd.setpoint_ned
        if nxt is not state:
            seen.append(nxt)
        state = nxt
        if state is MissionState.DONE:
            break
        t += 0.1
    return seen


# INF-M1: the whole basic mission runs deterministically end-to-end — now via RTH (return home)
# before landing, since the basic mission has no waypoints.
def test_full_basic_mission_sequence():
    seen = _run_to_done(make_sm(), max_ticks=400)
    assert seen == [
        MissionState.IDLE,
        MissionState.ARMING,
        MissionState.TAKEOFF,
        MissionState.HOVER,
        MissionState.RTH,
        MissionState.LANDING,
        MissionState.DONE,
    ]


# AC-2 (INF-M1): a full multi-waypoint patrol runs deterministically — visit each waypoint in
# order with dwell, return home, land.
def test_full_patrol_sequence():
    seen = _run_to_done(make_patrol_sm([WP0, WP1]), max_ticks=800)
    assert seen == [
        MissionState.IDLE,
        MissionState.ARMING,
        MissionState.TAKEOFF,
        MissionState.HOVER,
        MissionState.WAYPOINT,
        MissionState.DWELL,
        MissionState.WAYPOINT,
        MissionState.DWELL,
        MissionState.RTH,
        MissionState.LANDING,
        MissionState.DONE,
    ]


# Hermes Low #3: the takeoff target is takeoff_alt_m AGL above home — it incorporates home's own
# NED-down altitude, so it is correct even when home does not sit at the EKF-origin ground.
def test_takeoff_target_is_takeoff_alt_above_home():
    sm = MissionStateMachine(make_config(), waypoints_ned=[], home_ned=(1.0, 2.0, -3.0))
    _, cmd = sm.tick(MissionState.IDLE, make_telem())
    assert cmd.setpoint_ned == (1.0, 2.0, -3.0 - TAKEOFF_ALT)


# TS-14: the machine accepts a waypoint route (the M3 "no waypoints" guard is gone in M4) — but a
# waypoints_ned list that does not align 1:1 with config.waypoints is a node wiring bug, fail loud.
def test_constructor_accepts_aligned_waypoints():
    assert isinstance(make_patrol_sm([WP0, WP1]), MissionStateMachine)


def test_constructor_rejects_misaligned_waypoints():
    with pytest.raises(ValueError, match="align"):
        MissionStateMachine(make_config(), waypoints_ned=[WP0], home_ned=HOME_NED)


# Review #1: reset_timing() restarts the active state's time-based windows so the machine never
# completes a HOVER / tolerance-hold on wall-time that elapsed while the node was NOT ticking it.
def test_reset_timing_restarts_hover_window():
    sm = make_sm()
    sm.tick(MissionState.HOVER, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    sm.tick(MissionState.HOVER, make_telem(now_s=5.0, position_ned=TAKEOFF_NED))  # 5 s real hover
    sm.reset_timing()  # node observed a stale->fresh resume edge: restart the window
    nxt, _ = sm.tick(MissionState.HOVER, make_telem(now_s=100.0, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER  # does NOT complete on the unobserved elapsed wall-time
    nxt, _ = sm.tick(
        MissionState.HOVER, make_telem(now_s=100.0 + HOVER_TIME, position_ned=TAKEOFF_NED)
    )
    assert nxt is MissionState.RTH  # only after a fresh full hover_time_s of observed ticking


def test_reset_timing_restarts_tolerance_hold():
    sm = make_sm()
    sm.tick(MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=TAKEOFF_NED))
    sm.reset_timing()
    nxt, _ = sm.tick(MissionState.TAKEOFF, make_telem(now_s=HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.TAKEOFF  # hold clock restarted at the resume tick, not yet elapsed
    nxt, _ = sm.tick(MissionState.TAKEOFF, make_telem(now_s=2 * HOLD, position_ned=TAKEOFF_NED))
    assert nxt is MissionState.HOVER


# Hermes Medium: a cached PX4 sample is usable only while its age is within the freshness timeout.
@pytest.mark.parametrize(
    ("age_s", "timeout_s", "expected"),
    [(0.0, 1.0, True), (1.0, 1.0, True), (1.0001, 1.0, False), (5.0, 1.0, False)],
)
def test_telemetry_fresh_within_timeout(age_s, timeout_s, expected):
    assert telemetry_fresh(age_s, timeout_s) is expected


# Hermes Medium #1: a position fix is usable only when PX4 reports BOTH the horizontal and vertical
# EKF estimate valid — the node's precondition for arming.
@pytest.mark.parametrize(
    ("xy_valid", "z_valid", "expected"),
    [(True, True, True), (True, False, False), (False, True, False), (False, False, False)],
)
def test_local_position_usable_requires_both_flags(xy_valid, z_valid, expected):
    assert local_position_usable(xy_valid, z_valid) is expected
