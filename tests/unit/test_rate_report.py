"""Layer-A unit tests for the RTF-robust live-bag rate analyzer (docset 05, M8 / LR-8).

The witness (``verify_live_bag.py``, the ROS lane) reads a live bag and feeds these ROS-free pure
functions. Two behaviours are under test:

  * the TRUE rate is recovered from a topic's own timestamps de-duplicated to unique stamps — so a
    2x-duplicated, RTF-sagged live stream still measures at its real publish rate; and
  * the consistency guard hard-fails a demonstrably untrustworthy bag — either ``info_count`` ≠
    stream rows, or a sim-stamp ``dup_factor`` over the threshold — instead of a false rate PASS.

Numbers are the real ones measured from the offending bag
(``patrol_20260701T151327Z_20260701_151328``) and the checked-in reference bag.
"""

from __future__ import annotations

import pytest
from rate_report import (
    TopicSample,
    consistency_verdict,
    dup_factor,
    evaluate_true_rates,
    observed_true_rates,
    unique_stamp_rate,
)
from replay_assertions import AssertionSpec

_CAMERA = "/drone/camera/image_raw/compressed"
_STATE = "/patrol/mission_state"


def _ramp(n: int, hz: float, *, dup: int = 1, start_ns: int = 1_000_000_000) -> list[int]:
    """``n`` unique stamps at ``hz`` (ns), each repeated ``dup`` times (simulates duplicate delivery)."""
    step = int(1e9 / hz)
    return [start_ns + i * step for i in range(n) for _ in range(dup)]


def _sample(
    topic: str, info: int, rows: int, rate_stamps: list[int], sim_stamps: list[int] | None = None
) -> TopicSample:
    return TopicSample(
        topic,
        info_count=info,
        reader_rows=rows,
        rate_stamps_ns=rate_stamps,
        sim_stamps_ns=sim_stamps or [],
    )


# unique_stamp_rate collapses duplication and is RTF-invariant (span comes from the stamps).
@pytest.mark.parametrize(
    ("n", "hz", "dup"),
    [
        (150, 15.0, 1),  # clean 15 Hz
        (150, 15.0, 2),  # each frame delivered twice → still 15 Hz true rate
        (100, 10.0, 3),  # 10 Hz, tripled
    ],
)
def test_unique_stamp_rate_recovers_true_rate(n: int, hz: float, dup: int) -> None:
    assert unique_stamp_rate(_ramp(n, hz, dup=dup)) == pytest.approx(hz, rel=0.02)


@pytest.mark.parametrize(
    "stamps",
    [
        [],  # nothing
        [5],  # a single stamp has no span
        [5, 5, 5],  # all identical → no span
    ],
)
def test_unique_stamp_rate_no_span_is_zero(stamps: list[int]) -> None:
    assert unique_stamp_rate(stamps) == 0.0


@pytest.mark.parametrize(
    ("rows", "unique", "expected"),
    [
        (303, 303, 1.0),  # reference camera — clean
        (9335, 4537, pytest.approx(2.06, abs=0.01)),  # offending camera — 2x duplication
        (10, 0, 0.0),  # no unique stamps → guard against div-by-zero
    ],
)
def test_dup_factor(rows: int, unique: int, expected: float) -> None:
    assert dup_factor(rows, unique) == expected


# --- consistency_verdict: the hard-fail guard -------------------------------------------------


def test_clean_bag_passes_consistency() -> None:
    # reference camera: info == rows, sim-dup 1.0
    stamps = _ramp(303, 15.0)
    ok, reasons = consistency_verdict([_sample(_CAMERA, 303, 303, stamps, stamps)])
    assert ok is True
    assert reasons == []


def test_info_count_mismatch_fails_consistency() -> None:
    # ros2 bag info says 9037 but the stream yielded 9335 rows → inconsistent/non-finalized bag.
    # On the real offending bag every over-read row has a distinct log_time, so this info!=rows
    # signal — not dup_factor — is what catches it.
    ok, reasons = consistency_verdict([_sample(_CAMERA, 9037, 9335, _ramp(9335, 11.95))])
    assert ok is False
    assert any("!=" in r and _CAMERA in r for r in reasons)


def test_sim_dup_factor_over_threshold_fails_consistency() -> None:
    # info matches rows but each rendered frame's sim-stamp is duplicated → sim dup_factor ~2 > 1.2.
    sample = _sample(_CAMERA, 300, 300, _ramp(300, 30.0), _ramp(150, 15.0, dup=2))
    ok, reasons = consistency_verdict([sample])
    assert ok is False
    assert any("dup_factor" in r for r in reasons)


def test_sim_dup_factor_within_threshold_passes() -> None:
    # a single boundary collision (151 rows / 150 unique = 1.007) stays under 1.2.
    sim = _ramp(150, 15.0)
    sim.append(sim[-1])  # one accidental repeat
    ok, _ = consistency_verdict([_sample(_CAMERA, 151, 151, sim, sim)])
    assert ok is True


def test_no_sim_stamps_skips_dup_guard() -> None:
    # header-less/px4/tf types pass no sim stamps → dup guard is not applied (info==rows still passes).
    sample = _sample(_STATE, 200, 200, _ramp(200, 10.0))
    assert sample.sim_dup_factor is None
    ok, reasons = consistency_verdict([sample])
    assert ok is True
    assert reasons == []


# --- evaluate_true_rates: reuse the band check on RTF-robust rates -----------------------------


def _specs() -> list[AssertionSpec]:
    return [
        AssertionSpec(topic=_CAMERA, min_count=1, expected_hz=15.0, tol=0.40),
        AssertionSpec(topic=_STATE, min_count=1, expected_hz=10.0, tol=0.40),
    ]


def test_observed_true_rates_uses_unique_span() -> None:
    # 300 rows but 150 unique 15 Hz rate-stamps → the observed rate must be 15 Hz, not 30.
    sample = _sample(_CAMERA, 300, 300, _ramp(150, 15.0, dup=2))
    assert observed_true_rates([sample])[0].hz == pytest.approx(15.0, rel=0.02)


@pytest.mark.parametrize(
    ("camera_hz", "state_hz", "passes"),
    [
        (15.0, 10.0, True),  # both in band → PASS
        (30.0, 10.0, False),  # camera doubled (if we had NOT de-duped) → out of band → FAIL
        (15.0, 4.0, False),  # mission_state too slow → out of band → FAIL
    ],
)
def test_evaluate_true_rates(camera_hz: float, state_hz: float, passes: bool) -> None:
    samples = [
        _sample(_CAMERA, 150, 150, _ramp(150, camera_hz)),
        _sample(_STATE, 100, 100, _ramp(100, state_hz)),
    ]
    assert evaluate_true_rates(_specs(), samples).passed is passes


def test_dropped_topic_still_fails() -> None:
    # An asserted topic with no samples → absent → the comparator must fail it (deliberate-break AC).
    result = evaluate_true_rates(_specs(), [_sample(_CAMERA, 150, 150, _ramp(150, 15.0))])
    assert result.passed is False
    assert any(_STATE in f for f in result.failures)
