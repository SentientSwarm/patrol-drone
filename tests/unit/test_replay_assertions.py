"""Layer-A unit tests for the ROS-free replay assertion comparator (docset 05, M8 / T8.8, SWM-80).

The replay regression test (LR-5) plays a reference bag and counts messages per topic over a window,
then asserts the result against a curated subset + rate band (design §4.2.5, OQ-5). The *comparison*
is pure data — observed counts/rates vs the expected spec — so it lives in
`replay_assertions.evaluate` and is unit-tested here, host- and ROS-free. The `ros2 bag play` +
subscribe half that produces the observed counts is the ROS driver (test_replay_regression, CI lane).

Covers:
  * TS-15 — all asserted topics present + rates within ±40% → PASS.
  * TS-16 — a missing asserted topic → FAIL (this is what makes the deliberate-break self-check bite).
  * TS-17 — a rated topic outside the ±40% band → FAIL.
"""

from __future__ import annotations

from pathlib import Path

from replay_assertions import AssertionSpec, ObservedTopic, evaluate, load_specs


def _spec() -> list[AssertionSpec]:
    # The design §4.2.5 curated subset (trimmed for the test): one count-only, two rated.
    return [
        AssertionSpec(topic="/patrol/current_waypoint", min_count=1),
        AssertionSpec(topic="/patrol/mission_state", min_count=1, expected_hz=10.0, tol=0.40),
        AssertionSpec(
            topic="/fmu/out/vehicle_local_position", min_count=1, expected_hz=50.0, tol=0.40
        ),
    ]


# TS-15: every asserted topic present, all rates within ±40% → PASS (no failures).
def test_all_present_and_rated_passes() -> None:
    observed = [
        ObservedTopic("/patrol/current_waypoint", count=14, duration_s=10.0),
        ObservedTopic("/patrol/mission_state", count=100, duration_s=10.0),  # 10 Hz, exact
        ObservedTopic(
            "/fmu/out/vehicle_local_position", count=600, duration_s=10.0
        ),  # 60 Hz, in band
    ]

    result = evaluate(_spec(), observed)

    assert result.passed is True
    assert result.failures == []


# TS-16: a missing asserted topic → FAIL, with that topic named in the failures.
def test_missing_topic_fails() -> None:
    observed = [
        ObservedTopic("/patrol/current_waypoint", count=14, duration_s=10.0),
        # /patrol/mission_state is ABSENT (the deliberate-break shape)
        ObservedTopic("/fmu/out/vehicle_local_position", count=600, duration_s=10.0),
    ]

    result = evaluate(_spec(), observed)

    assert result.passed is False
    assert any("/patrol/mission_state" in f for f in result.failures)


# TS-16: a present-but-zero-count asserted topic → FAIL (min_count not met).
def test_zero_count_topic_fails() -> None:
    observed = [
        ObservedTopic("/patrol/current_waypoint", count=0, duration_s=10.0),
        ObservedTopic("/patrol/mission_state", count=100, duration_s=10.0),
        ObservedTopic("/fmu/out/vehicle_local_position", count=600, duration_s=10.0),
    ]

    result = evaluate(_spec(), observed)

    assert result.passed is False
    assert any("/patrol/current_waypoint" in f for f in result.failures)


# TS-17: a rated topic outside the ±40% band → FAIL.
def test_rate_outside_band_fails() -> None:
    observed = [
        ObservedTopic("/patrol/current_waypoint", count=14, duration_s=10.0),
        ObservedTopic("/patrol/mission_state", count=100, duration_s=10.0),
        # expected 50 Hz ±40% = [30, 70]; 200/10 = 20 Hz → below band
        ObservedTopic("/fmu/out/vehicle_local_position", count=200, duration_s=10.0),
    ]

    result = evaluate(_spec(), observed)

    assert result.passed is False
    assert any("/fmu/out/vehicle_local_position" in f for f in result.failures)


# TS-17: a rated topic at the band edge passes (±40% is inclusive).
def test_rate_at_band_edge_passes() -> None:
    spec = [AssertionSpec(topic="/x", min_count=1, expected_hz=10.0, tol=0.40)]
    # 10 Hz * (1 - 0.40) = 6 Hz lower edge; 60 msgs / 10 s = 6 Hz exactly
    observed = [ObservedTopic("/x", count=60, duration_s=10.0)]

    result = evaluate(spec, observed)

    assert result.passed is True


# load_specs parses the assertions.yaml subset into AssertionSpec objects (count-only + rated).
def test_load_specs_from_yaml(tmp_path: Path) -> None:
    yaml_text = (
        "topics:\n"
        "  - topic: /patrol/current_waypoint\n"
        "    min_count: 1\n"
        "  - topic: /patrol/mission_state\n"
        "    min_count: 2\n"
        "    expected_hz: 10.0\n"
        "    tol: 0.4\n"
    )
    path = tmp_path / "assertions.yaml"
    path.write_text(yaml_text)

    specs = load_specs(path)

    assert specs == [
        AssertionSpec(topic="/patrol/current_waypoint", min_count=1),
        AssertionSpec(topic="/patrol/mission_state", min_count=2, expected_hz=10.0, tol=0.4),
    ]


# The shipped assertions.yaml loads and lists the design §4.2.5 curated subset.
def test_shipped_assertions_yaml_loads() -> None:
    shipped = Path(__file__).resolve().parents[1] / "replay" / "assertions.yaml"
    specs = load_specs(shipped)

    topics = {s.topic for s in specs}
    assert "/patrol/checkpoint_capture" in topics
    assert "/drone/camera/image_raw/compressed" in topics
    assert "/fmu/out/vehicle_local_position_v1" in topics
