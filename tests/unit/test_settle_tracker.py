"""Unit tests for the pure settled-hover tracker (review #2).

Layer-A: ROS-free, deterministic, clock injected via ``update(z, now_s)`` — the acceptance harness's
settled-hover decision is exercised with no live bridge. The tracker lives beside the rclpy-importing
``mission_acceptance`` module under ``tests/integration``; it imports nothing heavy, so this Layer-A
test pulls it in via a path insert (mirrors how ``verify_mission.py`` reaches that directory).
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from settle_tracker import SettleTracker  # noqa: E402  (import after the path bootstrap)

TARGET = -5.0  # NED down: 5 m AGL
TOL = 0.5
MAX_GAP = 1.0


def _tracker() -> SettleTracker:
    return SettleTracker(target_z_ned=TARGET, tolerance_m=TOL, max_gap_s=MAX_GAP)


def _feed(tracker: SettleTracker, samples: list[tuple[float, float]]) -> None:
    """Feed ``(z, now_s)`` samples in order."""
    for z, now_s in samples:
        tracker.update(z, now_s)


# Continuous in-band, densely sampled: the hold accrues to the full observed span.
def test_continuous_in_band_accrues_full_hold():
    t = _tracker()
    _feed(t, [(TARGET, s * 0.5) for s in range(7)])  # 0.0 .. 3.0 s, every 0.5 s
    assert t.max_hold_s == pytest.approx(3.0)


# Leaving the band resets the window — a transient overshoot/dip can't accumulate a passing hover.
def test_leaving_band_resets_window():
    t = _tracker()
    _feed(t, [(TARGET, 0.0), (TARGET, 0.5), (TARGET + 5.0, 1.0), (TARGET, 1.5), (TARGET, 2.0)])
    assert t.max_hold_s == pytest.approx(0.5)  # 0.0->0.5 then reset; 1.5->2.0 after re-entry


# Review #2: a sample gap wider than max_gap_s breaks the window even when both endpoints are in-band
# — a telemetry blackout spanning the band must NOT count as continuous settled-hover time.
def test_observation_gap_breaks_window():
    t = _tracker()
    _feed(t, [(TARGET, 0.0), (TARGET, 8.0), (TARGET, 8.5), (TARGET, 9.0)])
    # The 8 s silence does not count; only the densely-sampled 8.0->9.0 window does.
    assert t.max_hold_s == pytest.approx(1.0)


# A gap exactly at max_gap_s is still continuous (boundary is inclusive: > breaks, == holds).
def test_gap_at_boundary_does_not_break():
    t = _tracker()
    _feed(t, [(TARGET, 0.0), (TARGET, MAX_GAP), (TARGET, 2 * MAX_GAP)])
    assert t.max_hold_s == pytest.approx(2 * MAX_GAP)


# The band edges are inclusive (±tolerance counts as settled); just outside does not.
@pytest.mark.parametrize(
    ("z", "expected_hold"),
    [(TARGET + TOL, 0.5), (TARGET - TOL, 0.5), (TARGET + TOL + 0.01, 0.0)],
    ids=["upper_edge", "lower_edge", "just_outside"],
)
def test_band_edges_inclusive(z, expected_hold):
    t = _tracker()
    _feed(t, [(z, 0.0), (z, 0.5)])
    assert t.max_hold_s == pytest.approx(expected_hold)
