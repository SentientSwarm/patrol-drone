"""Abort-logic unit tests for the MissionStateMachine — the hard floor (AC-7, AC-8, MC-6/MC-9).

Layer-A: ROS-free, deterministic. Every abort path is unit-covered here regardless of whether SITL
can trigger it (the design's whole point — "if it doesn't work in sim it won't work in flight", and
some triggers like manual-takeover/timeout are never fired in SITL at all). These tests must survive
even if SITL observability slips: they are the contract that the safety floor works.

Covered:
  * AC-8 — all four abort guards (external-signal + low-battery live; manual-takeover + timeout
    scaffold) each drive the ABORT transition and latch their reason.
  * AC-7 — the low-battery threshold transition, including the boundary.
  * abort precedence (external > battery > scaffolds), the abort-pre-empts-any-state property,
    the ABORT -> RTH route, and the "non-abortable once returning/landing/done" exclusion (so a
    momentary abort still drives the full return home).
"""

import pytest
from mission_builders import HOME_NED, WP0, WP1, make_patrol_sm, make_sm, make_telem
from patrol_mission.state_machine import AbortReason, MissionState

# Each abort trigger (the telemetry that fires it) paired with the reason it must latch (AC-8).
ABORT_TRIGGERS = [
    pytest.param({"abort_requested": True}, AbortReason.EXTERNAL_SIGNAL, id="external_signal"),
    pytest.param({"battery_remaining": 0.1}, AbortReason.LOW_BATTERY, id="low_battery"),
    pytest.param({"manual_takeover": True}, AbortReason.MANUAL_TAKEOVER, id="manual_scaffold"),
    pytest.param({"timed_out": True}, AbortReason.TIMEOUT, id="timeout_scaffold"),
]


# AC-8: every guard drives a transition to ABORT (from a normal in-flight state) and latches the
# matching reason — external-signal + low-battery (live) AND manual-takeover + timeout (scaffold).
@pytest.mark.parametrize(("telem_kwargs", "reason"), ABORT_TRIGGERS)
def test_abort_guard_transitions_to_abort_and_latches_reason(telem_kwargs, reason):
    sm = make_sm()
    nxt, cmd = sm.tick(
        MissionState.TAKEOFF, make_telem(now_s=0.0, position_ned=(0, 0, -7), **telem_kwargs)
    )
    assert nxt is MissionState.ABORT
    assert cmd.setpoint_ned == HOME_NED
    assert sm._p.abort_reason is reason


# AC-8: abort pre-empts whichever normal state the mission is in — not just one.
@pytest.mark.parametrize(
    "state",
    [MissionState.TAKEOFF, MissionState.HOVER, MissionState.WAYPOINT, MissionState.DWELL],
)
def test_abort_preempts_any_normal_state(state):
    sm = make_patrol_sm([WP0, WP1])
    nxt, _ = sm.tick(state, make_telem(abort_requested=True, position_ned=WP0))
    assert nxt is MissionState.ABORT


# Abort precedence: external-signal outranks a simultaneous low-battery condition.
def test_external_signal_takes_precedence_over_low_battery():
    sm = make_sm()
    reason = sm._abort_reason(make_telem(abort_requested=True, battery_remaining=0.05))
    assert reason is AbortReason.EXTERNAL_SIGNAL


# No guard fires when the vehicle is healthy (full battery, no abort, no scaffold flags).
def test_no_abort_when_healthy():
    sm = make_sm()
    assert sm._abort_reason(make_telem(battery_remaining=1.0)) is AbortReason.NONE


# AC-7: the low-battery transition keys on remaining < threshold (strict) — assert the boundary.
@pytest.mark.parametrize(
    ("battery_remaining", "expected"),
    [
        (0.19, AbortReason.LOW_BATTERY),
        (0.20, AbortReason.NONE),  # exactly at threshold is NOT low (strict <)
        (0.50, AbortReason.NONE),
    ],
)
def test_low_battery_threshold_boundary(battery_remaining, expected):
    sm = make_sm()  # make_config low_battery_threshold = 0.20
    assert sm._abort_reason(make_telem(battery_remaining=battery_remaining)) is expected


# The ABORT state routes to the explicit home-waypoint return (RTH) on the next tick (OQ-8).
def test_abort_state_routes_to_rth():
    nxt, cmd = make_sm().tick(MissionState.ABORT, make_telem())
    assert nxt is MissionState.RTH
    assert cmd.setpoint_ned == HOME_NED


# Once aborting/returning/landing/done the abort guard must NOT re-fire (would loop) — and this
# exclusion is what makes a momentary abort "stick" through the whole return home.
@pytest.mark.parametrize(
    ("state", "telem_kwargs", "expected"),
    [
        (MissionState.RTH, {"position_ned": (0, 0, -7)}, MissionState.RTH),
        (MissionState.LANDING, {"armed": True}, MissionState.LANDING),
        (MissionState.ABORT, {}, MissionState.RTH),
        (MissionState.DONE, {}, MissionState.DONE),
    ],
)
def test_abort_does_not_refire_in_non_abortable_states(state, telem_kwargs, expected):
    sm = make_sm()
    nxt, _ = sm.tick(state, make_telem(abort_requested=True, **telem_kwargs))
    assert nxt is expected


# AC-6 (unit analog): an external abort raised mid-patrol drives ABORT -> RTH -> LANDING -> DONE,
# and the mission completes the return home even after the abort signal is released (it sticks).
# The SITL-observable half of AC-6 is asserted in tests/integration/test_mission_patrol.py.
def test_mid_patrol_abort_drives_full_return_home():
    sm = make_patrol_sm([WP0, WP1])
    state, _ = sm.tick(
        MissionState.WAYPOINT, make_telem(now_s=0.0, position_ned=WP0, abort_requested=True)
    )
    assert state is MissionState.ABORT

    seen: list[MissionState] = [state]
    armed, pos, t = True, WP0, 0.1
    for _ in range(400):  # 40 s at 10 Hz — generous bound for RTH-settle + land
        nxt, cmd = sm.tick(
            state,
            # abort signal released after the initial pulse — the mission must still return home
            make_telem(now_s=t, position_ned=pos, armed=armed, offboard_active=True),
        )
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
    assert seen == [
        MissionState.ABORT,
        MissionState.RTH,
        MissionState.LANDING,
        MissionState.DONE,
    ]
