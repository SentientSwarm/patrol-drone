"""Unit tests for the pure dwell tracker (Hermes High — split-topic dwell false-positive).

Layer-A: ROS-free, deterministic. The patrol acceptance harness's dwell attribution is exercised
with no live bridge. The tracker lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(mirrors ``test_home_settle_tracker`` and how ``verify_patrol.py`` reaches that directory).
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from dwell_tracker import DwellTracker  # noqa: E402  (import after the path bootstrap)


# A normal leg: the active index is published (WAYPOINT i) and then DWELL is observed -> i counts.
def test_dwell_observed_with_active_index_counts():
    t = DwellTracker()
    t.on_waypoint(0)
    t.on_state("WAYPOINT")
    t.on_state("DWELL")
    assert t.dwelled == {0}


# The exact reorder Hermes flagged: while the cached state is still DWELL(i), a reordered
# current_waypoint=i+1 is delivered BEFORE the WAYPOINT(i+1) state. The next waypoint must NOT be
# counted as dwelled from the index update alone — only the authoritative DWELL state attributes a
# dwell, and the state stream has no DWELL after the index advances.
def test_reordered_next_waypoint_before_waypoint_state_not_dwelled():
    t = DwellTracker()
    t.on_waypoint(0)
    t.on_state("DWELL")  # dwelled at 0
    t.on_waypoint(1)  # current_waypoint advances early (reordered ahead of the WAYPOINT state)
    assert t.dwelled == {0}  # 1 is NOT counted — the vehicle only started flying toward it
    t.on_state("WAYPOINT")  # the state catches up; still not a dwell for 1
    assert t.dwelled == {0}


# A full nominal patrol counts every leg exactly once, including the final waypoint.
def test_full_patrol_counts_every_waypoint():
    t = DwellTracker()
    for i in range(4):
        t.on_waypoint(i)
        t.on_state("WAYPOINT")
        t.on_state("DWELL")
    assert t.dwelled == {0, 1, 2, 3}


# The sentinel index (-1, published before the first waypoint / during non-waypoint states) never
# counts, even if DWELL is somehow observed while it is the active index.
def test_negative_index_never_counts():
    t = DwellTracker()
    t.on_waypoint(-1)
    t.on_state("DWELL")
    assert t.dwelled == set()


# Non-DWELL states never attribute a dwell, regardless of the active index.
@pytest.mark.parametrize("state", ["TAKEOFF", "HOVER", "WAYPOINT", "RTH", "LANDING", "ABORT"])
def test_non_dwell_states_do_not_count(state):
    t = DwellTracker()
    t.on_waypoint(2)
    t.on_state(state)
    assert t.dwelled == set()
