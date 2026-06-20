"""Unit tests for the pure dwell tracker (Hermes High — duration-aware, race-free dwell oracle).

Layer-A: ROS-free, deterministic. The patrol acceptance harness's dwell verdict is exercised with no
live bridge. The tracker lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(mirrors ``test_home_settle_tracker`` and how ``verify_patrol.py`` reaches that directory).

The tracker counts DWELL *episodes* in the (per-topic-ordered) mission_state stream and never reads
current_waypoint — so a cross-topic reorder cannot influence the count — and credits a waypoint only
once its episode has spanned the configured ``dwell_s``. These cases drive the timestamped state
stream alone, matching how the watcher feeds the tracker.
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from dwell_tracker import DwellTracker  # noqa: E402  (import after the path bootstrap)

DWELL_S = 3.0  # required dwell per waypoint in these cases (the shipped patrol uses 3.0)


def _feed(t: DwellTracker, states: list[str], *, dt: float = 1.0, start: float = 0.0) -> None:
    """Replay a mission_state stream at a fixed cadence ``dt`` (s); the k-th sample at ``start+k*dt``."""
    for k, s in enumerate(states):
        t.on_state(s, start + k * dt)


def _feed_at(t: DwellTracker, samples: list[tuple[str, float]]) -> None:
    """Replay an explicitly-timestamped mission_state stream (for sparse / gapped cadences)."""
    for state, ts in samples:
        t.on_state(state, ts)


def _one_waypoint(required: float = DWELL_S) -> DwellTracker:
    return DwellTracker(dwell_required_s=(required,))


# A leg held for its full dwell — WAYPOINT(0) then DWELL(0) republished across several ticks, then the
# next state — counts waypoint 0 (entry->exit span 3 s >= 3 s).
def test_full_dwell_counts_the_waypoint():
    t = _one_waypoint()
    _feed(t, ["WAYPOINT", "DWELL", "DWELL", "DWELL", "RTH"])  # DWELL@1..3, exit(RTH)@4 -> span 3.0
    assert t.dwelled == {0}


# The Hermes regression: a DWELL that is entered but NOT held for the configured dwell_s must not
# count — even though the machine entered DWELL (the old rising-edge oracle would have false-passed it).
def test_short_dwell_does_not_count_even_though_entered():
    t = _one_waypoint()
    _feed(t, ["WAYPOINT", "DWELL", "WAYPOINT"])  # DWELL@1, exit@2 -> span 1.0 < 3.0
    assert t.episodes == 1  # the episode WAS entered (waypoint reached)...
    assert t.dwelled == set()  # ...but not held long enough, so it is NOT credited


# A single transient DWELL sample (the one-sample false-pass Hermes called out) spans ~0 s -> rejected.
def test_single_dwell_sample_does_not_count():
    t = _one_waypoint()
    t.on_state("WAYPOINT", 0.0)
    t.on_state("DWELL", 1.0)
    t.on_state("RTH", 1.0)  # leaves at the same instant it entered -> span 0
    assert t.dwelled == set()


# Crediting also happens incrementally: a DWELL stream that reaches the threshold mid-episode (before
# any falling edge — e.g. observation captured right up to the hold completing) still counts. This
# covers the final waypoint, whose episode the watcher observes up to RTH.
def test_incremental_credit_without_falling_edge():
    t = _one_waypoint()
    _feed(
        t, ["WAYPOINT", "DWELL", "DWELL", "DWELL", "DWELL"]
    )  # DWELL@1..4: at @4 span 3.0 -> credit
    assert t.dwelled == {0}


# Per-waypoint requirements are honored independently: waypoint 0 (needs 1 s) is held ~3 s and counts;
# waypoint 1 (needs 5 s) is held only ~2 s and does not.
def test_per_waypoint_required_dwell_is_independent():
    t = DwellTracker(dwell_required_s=(1.0, 5.0))
    _feed(
        t,
        [
            "WAYPOINT",
            "DWELL",
            "DWELL",
            "DWELL",  # ep0: enter@1, exit(WAYPOINT)@4 -> span 3 >= 1
            "WAYPOINT",
            "DWELL",
            "DWELL",
            "RTH",  # ep1: enter@5, exit(RTH)@7 -> span 2 < 5
        ],
    )
    assert t.episodes == 2
    assert t.dwelled == {0}  # waypoint 1 was reached but held too briefly


# Full nominal patrol: four legs each held for the configured dwell count every waypoint once, the
# final waypoint included (its episode ends at RTH).
def test_full_patrol_counts_every_waypoint():
    t = DwellTracker(dwell_required_s=(DWELL_S,) * 4)
    states: list[str] = []
    for _ in range(4):
        states += ["WAYPOINT", "DWELL", "DWELL", "DWELL", "DWELL"]  # each leg spans >= 3 s
    states += ["RTH"]
    _feed(t, states)
    assert t.dwelled == {0, 1, 2, 3}


# The Hermes cross-topic reorder, on the surface the tracker consumes: the vehicle dwelled (long
# enough) at 0,1,2 but NEVER reached waypoint 3. Extra trailing DWELL(2) samples (the delayed/duplicate
# ones whose cross-topic peers already reported current_waypoint=3) stay part of waypoint 2's single
# episode. Because the tracker ignores current_waypoint, waypoint 3 is not counted.
def test_reorder_cannot_false_count_unreached_final_waypoint():
    t = DwellTracker(dwell_required_s=(DWELL_S,) * 4)
    _feed(
        t,
        [
            "WAYPOINT",
            "DWELL",
            "DWELL",
            "DWELL",
            "DWELL",  # wp0 held
            "WAYPOINT",
            "DWELL",
            "DWELL",
            "DWELL",
            "DWELL",  # wp1 held
            "WAYPOINT",
            "DWELL",
            "DWELL",
            "DWELL",
            "DWELL",
            "DWELL",
            "DWELL",  # wp2 held + trailing
        ],
    )
    assert t.episodes == 3
    assert t.dwelled == {0, 1, 2}  # waypoint 3 never had its own dwell episode


# Consecutive DWELL samples (the node republishes DWELL every tick for the whole hold) are one episode.
def test_consecutive_dwell_samples_are_a_single_episode():
    t = _one_waypoint()
    _feed(t, ["WAYPOINT"] + ["DWELL"] * 20)  # 20 samples at dt=1 span 19 s >> 3 s
    assert t.episodes == 1
    assert t.dwelled == {0}


# Two DWELL runs split by an intervening WAYPOINT are two distinct episodes (dwell_s 0 -> both count).
def test_dwell_runs_split_by_waypoint_are_distinct_episodes():
    t = DwellTracker(dwell_required_s=(0.0, 0.0))
    _feed(t, ["DWELL", "DWELL", "WAYPOINT", "DWELL", "DWELL"])
    assert t.episodes == 2
    assert t.dwelled == {0, 1}


# Non-DWELL states never start an episode, regardless of how many are seen.
@pytest.mark.parametrize("state", ["TAKEOFF", "HOVER", "WAYPOINT", "RTH", "LANDING", "ABORT"])
def test_non_dwell_states_do_not_count(state):
    t = _one_waypoint()
    _feed(t, [state, state])
    assert t.episodes == 0
    assert t.dwelled == set()


# Hermes Medium: an observation gap wider than max_gap_s between two DWELL samples must NOT be credited
# as continuous dwell. Two DWELL samples 10 s apart span 10 s of wall clock but prove no continuous
# hold — the gap restarts the episode clock, so the waypoint is reached (an episode) but not credited.
def test_observation_gap_is_not_credited_as_dwell():
    t = _one_waypoint()  # max_gap_s defaults to 1.0
    _feed_at(t, [("WAYPOINT", 0.0), ("DWELL", 1.0), ("DWELL", 11.0)])  # 10 s silence mid-episode
    assert t.episodes == 1  # the waypoint WAS reached...
    assert t.dwelled == set()  # ...but the gap broke the hold; the post-gap span is 0 s


# No regression: a sparse-but-dense-enough cadence (every gap <= max_gap_s) still credits normally.
# Gaps exactly equal to max_gap_s do not break (the break is strict `>`), mirroring HomeSettleTracker.
def test_gap_at_or_below_threshold_still_credits():
    t = _one_waypoint()  # max_gap_s 1.0
    _feed_at(t, [("WAYPOINT", 0.0), ("DWELL", 1.0), ("DWELL", 2.0), ("DWELL", 3.0), ("DWELL", 4.0)])
    assert t.dwelled == {0}  # gaps all 1.0 == max_gap_s; span 3.0 >= 3.0


# A gap mid-episode restarts the clock: a full dwell_s must be observed continuously AFTER the resume,
# not measured from the original entry across the blackout.
def test_gap_mid_episode_requires_a_fresh_full_span_after_resume():
    t = _one_waypoint()
    _feed_at(t, [("WAYPOINT", 0.0), ("DWELL", 1.0), ("DWELL", 2.0), ("DWELL", 12.0)])  # gap@12
    assert t.dwelled == set()  # post-gap span 0 s — entry-across-blackout (11 s) is NOT credited
    _feed_at(t, [("DWELL", 13.0), ("DWELL", 14.0), ("DWELL", 15.0)])  # 3 s held since the restart
    assert t.dwelled == {0}


# The max_gap_s threshold is honored as configured: a wider tolerance keeps a sparse stream as one hold.
def test_configured_max_gap_s_widens_the_tolerated_silence():
    t = DwellTracker(dwell_required_s=(DWELL_S,), max_gap_s=5.0)
    _feed_at(t, [("WAYPOINT", 0.0), ("DWELL", 1.0), ("DWELL", 4.0), ("DWELL", 7.0)])  # gaps 3 <= 5
    assert t.dwelled == {0}  # never broken; span 6.0 >= 3.0
