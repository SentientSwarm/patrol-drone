"""Unit tests for the pure dwell tracker (Hermes High — split-topic dwell false-positive).

Layer-A: ROS-free, deterministic. The patrol acceptance harness's dwell verdict is exercised with no
live bridge. The tracker lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(mirrors ``test_home_settle_tracker`` and how ``verify_patrol.py`` reaches that directory).

The tracker counts DWELL *episodes* in the (per-topic-ordered) mission_state stream and never reads
current_waypoint — so a cross-topic reorder cannot influence the count. These cases drive only the
state stream, matching how the watcher feeds the tracker.
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from dwell_tracker import DwellTracker  # noqa: E402  (import after the path bootstrap)


def _feed(t: DwellTracker, states: list[str]) -> None:
    """Replay a mission_state stream (every sample, including consecutive duplicates) into the tracker."""
    for s in states:
        t.on_state(s)


# A single leg: WAYPOINT(i) then DWELL(i) — held for several ticks — is one episode -> waypoint 0.
def test_one_dwell_episode_counts_one_waypoint():
    t = DwellTracker()
    _feed(t, ["WAYPOINT", "WAYPOINT", "DWELL", "DWELL", "DWELL"])
    assert t.episodes == 1
    assert t.dwelled == {0}


# A full nominal patrol: four WAYPOINT->DWELL episodes count every leg exactly once, the final
# waypoint included.
def test_full_patrol_counts_every_waypoint():
    t = DwellTracker()
    for _ in range(4):
        _feed(t, ["WAYPOINT", "WAYPOINT", "DWELL", "DWELL"])
    assert t.episodes == 4
    assert t.dwelled == {0, 1, 2, 3}


# The exact Hermes reorder, expressed on the surface the tracker actually consumes. The vehicle
# dwelled at 0,1,2 but NEVER reached waypoint 3 (no WAYPOINT(3)->DWELL(3) episode). DDS keeps the
# state topic ordered, so every DWELL(2) sample — even a delayed/duplicate one that arrives after
# current_waypoint already advanced to 3 on the *other* topic — is still part of waypoint 2's single
# episode. Because the tracker ignores current_waypoint entirely, waypoint 3 is NOT counted.
def test_reorder_cannot_false_count_unreached_final_waypoint():
    t = DwellTracker()
    _feed(t, ["WAYPOINT", "DWELL", "DWELL"])  # waypoint 0 dwelled
    _feed(t, ["WAYPOINT", "DWELL", "DWELL"])  # waypoint 1 dwelled
    # waypoint 2 dwelled, with extra trailing DWELL(2) samples (the delayed/duplicate ones whose
    # cross-topic peers reported current_waypoint=3 before they arrived):
    _feed(t, ["WAYPOINT", "DWELL", "DWELL", "DWELL", "DWELL"])
    assert t.episodes == 3
    assert t.dwelled == {0, 1, 2}  # waypoint 3 is NOT counted — it never had its own dwell episode


# Consecutive DWELL samples (the node republishes DWELL every tick for the whole hold) are one
# episode, not one per sample.
def test_consecutive_dwell_samples_are_a_single_episode():
    t = DwellTracker()
    _feed(t, ["WAYPOINT"] + ["DWELL"] * 20)
    assert t.episodes == 1


# Distinct legs are separated by the intervening non-DWELL (WAYPOINT) state, so two DWELL runs split
# by a WAYPOINT are two episodes.
def test_dwell_runs_split_by_waypoint_are_distinct_episodes():
    t = DwellTracker()
    _feed(t, ["DWELL", "DWELL", "WAYPOINT", "DWELL", "DWELL"])
    assert t.episodes == 2


# Non-DWELL states never start an episode, regardless of how many are seen.
@pytest.mark.parametrize("state", ["TAKEOFF", "HOVER", "WAYPOINT", "RTH", "LANDING", "ABORT"])
def test_non_dwell_states_do_not_count(state):
    t = DwellTracker()
    _feed(t, [state, state])
    assert t.dwelled == set()
