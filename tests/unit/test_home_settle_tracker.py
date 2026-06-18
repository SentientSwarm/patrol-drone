"""Unit tests for the pure home-settle tracker (Hermes High — pre-RTH home crossing).

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
# A point well inside the tolerance ball around HOME, and one well outside it.
AT_HOME = (0.0, 0.0, -2.0)
AWAY = (10.0, 0.0, -2.0)


def _tracker() -> HomeSettleTracker:
    return HomeSettleTracker(home_ned=HOME, tolerance_m=TOL)


# The regression Hermes asked for: a vehicle sitting exactly at home BEFORE RTH (the takeoff climb
# passing through the home altitude at home x/y) must NOT latch returned-home.
def test_pre_rth_home_crossing_does_not_latch():
    t = _tracker()
    t.update(AT_HOME, rth_started=False)  # at home, but RTH has not begun -> ignored
    assert t.settled is False
    assert t.min_distance_m == float(
        "inf"
    )  # pre-RTH samples don't even count toward the diagnostic


# Once RTH has begun, an in-tolerance sample latches the settle (the genuine return-home).
def test_post_rth_in_tolerance_latches():
    t = _tracker()
    t.update(AT_HOME, rth_started=True)
    assert t.settled is True
    assert t.min_distance_m == pytest.approx(0.0)


# A pre-RTH crossing followed by an RTH that never reaches home stays unsatisfied — the exact false
# pass the gate must reject (saw_rth True earlier would otherwise ride on the stale pre-RTH latch).
def test_pre_rth_crossing_then_rth_far_from_home_stays_unsatisfied():
    t = _tracker()
    t.update(AT_HOME, rth_started=False)  # climb through home altitude
    t.update(AWAY, rth_started=True)  # RTH underway but never settles at home
    assert t.settled is False
    assert t.min_distance_m == pytest.approx(10.0)


# An invalid EKF fix is ignored even after RTH (mirrors the node's xy_valid/z_valid gate).
def test_invalid_fix_is_ignored():
    t = _tracker()
    t.update(AT_HOME, rth_started=True, valid=False)
    assert t.settled is False
    assert t.min_distance_m == float("inf")


# The tolerance ball edge is inclusive; just outside it does not settle (and tracks closest approach).
@pytest.mark.parametrize(
    ("position", "settles"),
    [((TOL, 0.0, -2.0), True), ((TOL + 0.01, 0.0, -2.0), False)],
    ids=["on_edge", "just_outside"],
)
def test_tolerance_edge_inclusive(position, settles):
    t = _tracker()
    t.update(position, rth_started=True)
    assert t.settled is settles


# min_distance_m keeps the closest post-RTH approach across samples (a later far sample can't worsen it).
def test_min_distance_tracks_closest_post_rth_approach():
    t = _tracker()
    t.update(AWAY, rth_started=True)  # 10 m out
    t.update((1.0, 0.0, -2.0), rth_started=True)  # 1 m out — closer
    t.update(AWAY, rth_started=True)  # 10 m again — must not overwrite the closer approach
    assert t.min_distance_m == pytest.approx(1.0)
