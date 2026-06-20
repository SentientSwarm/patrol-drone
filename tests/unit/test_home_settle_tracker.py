"""Unit tests for the pure home-settle tracker (Hermes High — continuous-hold return-home oracle).

Layer-A: ROS-free, deterministic. The patrol acceptance harness's return-home decision is exercised
with no live bridge. The tracker lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(mirrors ``test_settle_tracker`` and how ``verify_patrol.py`` reaches that directory).
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from home_settle_tracker import HomeSettleTracker  # noqa: E402  (import after the path bootstrap)

HOME = (0.0, 0.0, -2.0)  # home at 2 m ENU -> -2 m NED down (matches the shipped patrol config)
TOL = 0.5
HOLD_S = 2.0  # completion.hold_time_s — the continuous in-tolerance hold RTH must sustain
# A point well inside the tolerance ball around HOME, and one well outside it.
AT_HOME = (0.0, 0.0, -2.0)
AWAY = (10.0, 0.0, -2.0)

Sample = tuple[tuple[float, float, float], float]  # (position_ned, now_s)


def _tracker() -> HomeSettleTracker:
    return HomeSettleTracker(home_ned=HOME, tolerance_m=TOL, hold_time_s=HOLD_S)


def _feed(
    t: HomeSettleTracker, samples: list[Sample], *, rth: bool = True, valid: bool = True
) -> None:
    """Fold a list of ``(position, now_s)`` fixes in with shared ``rth_started`` / ``valid`` flags."""
    for position, now_s in samples:
        t.update(position, now_s, rth_started=rth, valid=valid)


# The Hermes regression: a SINGLE post-RTH in-tolerance fix must NOT latch returned-home — the old
# oracle did, but the state machine requires a continuous hold before leaving RTH.
def test_single_post_rth_sample_does_not_latch():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0)])  # one fix at home, but the hold_time_s hold has not elapsed
    assert t.settled is False
    assert t.min_distance_m == pytest.approx(0.0)  # it DID count toward the diagnostic


# Held continuously within tolerance for hold_time_s -> latches (the genuine, sustained return-home).
def test_continuous_hold_latches():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0), (AT_HOME, 1.0)])  # 1 s held so far
    assert t.settled is False
    _feed(t, [(AT_HOME, 2.0)])  # 2 s >= hold_time_s
    assert t.settled is True


# Leaving the tolerance ball resets the hold clock: a crossing-then-return must hold the FULL
# hold_time_s again. (Without the reset, the elapsed-from-first-entry time would false-latch here.)
def test_leaving_tolerance_resets_hold_clock():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0), (AWAY, 1.0), (AT_HOME, 2.0), (AT_HOME, 3.9)])
    assert t.settled is False  # the hold restarted at 2.0 s; only 1.9 s elapsed by 3.9 s
    _feed(t, [(AT_HOME, 4.0)])  # 2.0 s since the restart
    assert t.settled is True


# A vehicle sitting at home BEFORE RTH (the takeoff climb through home altitude) never latches, even
# if it lingers there — pre-RTH samples are ignored entirely (Hermes High).
def test_pre_rth_home_crossing_does_not_latch():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0), (AT_HOME, 5.0)], rth=False)
    assert t.settled is False
    assert t.min_distance_m == float(
        "inf"
    )  # pre-RTH samples don't count toward the diagnostic either


# A pre-RTH crossing followed by an RTH that never reaches home stays unsatisfied — a mis-aimed return
# (Hermes Medium) and the stale pre-RTH latch the old oracle risked are both rejected.
def test_pre_rth_crossing_then_rth_far_stays_unsatisfied():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0)], rth=False)  # climb through home altitude
    _feed(t, [(AWAY, 1.0), (AWAY, 5.0)])  # RTH underway but never settles at home
    assert t.settled is False
    assert t.min_distance_m == pytest.approx(10.0)


# An invalid EKF fix is ignored even after RTH (mirrors the node's xy_valid/z_valid gate).
def test_invalid_fix_is_ignored():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0), (AT_HOME, 5.0)], valid=False)
    assert t.settled is False
    assert t.min_distance_m == float("inf")


# The tolerance ball edge is inclusive; just outside it never enters the hold (and so never settles).
@pytest.mark.parametrize(
    ("position", "settles"),
    [((TOL, 0.0, -2.0), True), ((TOL + 0.01, 0.0, -2.0), False)],
    ids=["on_edge", "just_outside"],
)
def test_tolerance_edge_inclusive(position, settles):
    t = _tracker()
    _feed(t, [(position, 0.0), (position, HOLD_S)])  # held for hold_time_s at the same offset
    assert t.settled is settles


# min_distance_m keeps the closest post-RTH approach across samples (a later far sample can't worsen it),
# independent of whether the hold ever completes.
def test_min_distance_tracks_closest_post_rth_approach():
    t = _tracker()
    _feed(t, [(AWAY, 0.0), ((1.0, 0.0, -2.0), 1.0), (AWAY, 2.0)])  # 10 m, 1 m (closer), 10 m again
    assert t.min_distance_m == pytest.approx(1.0)
    assert t.settled is False  # 1 m is outside the 0.5 m ball, so it never settles


# Once a genuine hold latches the settle, a later drift away does NOT un-latch it (return-home proven).
def test_settle_latches_and_stays():
    t = _tracker()
    _feed(t, [(AT_HOME, 0.0), (AT_HOME, HOLD_S)])
    assert t.settled is True
    _feed(t, [(AWAY, 3.0)])
    assert t.settled is True
