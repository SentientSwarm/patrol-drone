"""Replay regression test — the CI guard the bag becomes (docset 05, M8 / T8.8, SWM-80, LR-5).

Plays the checked-in reference bag via ``ros2 bag play`` while rclpy subscribers count messages per
topic, then asserts the result against the curated subset in ``assertions.yaml`` using the ROS-free
:func:`replay_assertions.evaluate` comparator (design §4.2.5). This is the "the bag is the
regression test" payoff: a later-phase change that drops a recorded topic is caught here in CI
before it reaches hardware (PRD H3 — deterministic, plays a fixed bag, not the simulator).

This is the ROS lane (``pytest.mark.ros``): it needs a sourced ROS env + ``ros2 bag play`` + the
LFS-materialized reference bag. The pure comparison logic is unit-tested separately in
tests/unit/test_replay_assertions.py; here we drive the real play→subscribe→evaluate path.

Budget: ≤ 90 s wall-clock (OQ-6). The deliberate-break self-check (test_dropped_topic_fails) proves
the guard actually guards — a reference bag missing an asserted topic must FAIL the assertions
(LR-5 deliberate-break AC).
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String

# Self-bootstrap the dirs this test imports first-party modules from, so the imports do not depend on
# the lane's pytest `pythonpath` — the /tmp-config rootdir bug (PR #16 / F-01) broke exactly that
# dependence. Mirrors tests/integration/test_upload_ingest_standin.py, which is immune for this reason.
#   tests/replay (here) → replay_assertions ;  docker → ingest.bag_reader (the bag-info count source)
_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parents[1] / "docker"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ingest.bag_reader import parse_bag_info  # noqa: E402  (after the sys.path bootstrap above)
from replay_assertions import (  # noqa: E402  (after the sys.path bootstrap above)
    AssertionSpec,
    ObservedTopic,
    evaluate,
    load_specs,
)

pytestmark = pytest.mark.ros

_REFERENCE_BAG = Path(__file__).parent / "reference" / "patrol_reference"
_ASSERTIONS = Path(__file__).parent / "assertions.yaml"
# Play at real-time (rate 1.0) so wall-clock elapsed equals the bag's own ~20 s duration and the
# observed rate (count / elapsed) is the true publish rate — NOT inflated by a playback speed-up.
# The 20 s slice is already well under the 90 s replay budget (OQ-6), so no speed-up is needed.
_PLAY_RATE = 1.0


def _require_reference_bag() -> None:
    """Fail loudly if the LFS reference bag is absent/unresolved — a hard error, not a skip (§4.4.5)."""
    mcap = next(_REFERENCE_BAG.glob("*.mcap"), None)
    if mcap is None or mcap.stat().st_size < 1024:
        raise FileNotFoundError(
            f"reference bag missing or an unresolved LFS pointer at {_REFERENCE_BAG} — "
            "checkout needs `lfs: true`; the replay test requires the bag (it does not skip)."
        )


class _CountingNode(Node):
    """Subscribes to a set of topics and counts every message received during playback."""

    def __init__(self, topics: dict[str, type]) -> None:
        super().__init__("replay_counter")
        self.counts: dict[str, int] = dict.fromkeys(topics, 0)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, msg_type in topics.items():
            self.create_subscription(msg_type, topic, self._make_cb(topic), qos)

    def _make_cb(self, topic: str):
        def _cb(_msg: object) -> None:
            self.counts[topic] += 1

        return _cb


# std_msgs topics are counted by live subscription; the rest (camera/compressed, fmu pose,
# checkpoint_capture) are counted from `ros2 bag info` so the gate covers ALL asserted topics
# without importing their (non-std) message types into the subscriber.
_SUBSCRIBED = {
    "/patrol/mission_state": String,
    "/patrol/current_waypoint": Int32,
}


def _bag_info_observed(bag: Path, topics: set[str]) -> list[ObservedTopic]:
    """ObservedTopic for each of ``topics`` from `ros2 bag info` (count over the bag's duration)."""
    info = subprocess.run(
        ["ros2", "bag", "info", str(bag)], check=True, capture_output=True, text=True
    )
    facts = parse_bag_info(info.stdout)
    return [ObservedTopic(t, facts.topic_counts.get(t, 0), facts.duration_s) for t in topics]


def _play_and_count(bag: Path, topics: dict[str, type], window_s: float) -> list[ObservedTopic]:
    """Play ``bag`` and return per-topic ObservedTopic counts over the playback window."""
    rclpy.init()
    node = _CountingNode(topics)
    player = subprocess.Popen(["ros2", "bag", "play", "--rate", str(_PLAY_RATE), str(bag)])
    try:
        start = time.monotonic()
        while player.poll() is None and time.monotonic() - start < window_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        player.wait(timeout=10)
        elapsed = time.monotonic() - start
        return [ObservedTopic(t, node.counts[t], elapsed) for t in topics]
    finally:
        _terminate(player)
        node.destroy_node()
        rclpy.shutdown()


def _terminate(player: subprocess.Popen) -> None:
    """Best-effort: kill the player and reap it so no `ros2 bag play` child leaks (F-05)."""
    if player.poll() is None:
        player.kill()
    player.wait(timeout=10)


def test_replay_topics_present_and_rated() -> None:
    """TS-18/TS-20: GIVEN the reference bag (LFS-materialized), WHEN replayed, THEN every asserted
    topic is present at its rate. std_msgs topics are counted live during playback; the rest are
    counted from `ros2 bag info`. _require_reference_bag covers TS-20 (LFS pointer → hard fail)."""
    _require_reference_bag()
    specs = load_specs(_ASSERTIONS)
    info_topics = {s.topic for s in specs} - set(_SUBSCRIBED)

    observed = _play_and_count(_REFERENCE_BAG, _SUBSCRIBED, window_s=80.0)
    observed += _bag_info_observed(_REFERENCE_BAG, info_topics)

    result = evaluate(specs, observed)
    assert result.passed, result.failures


def test_dropped_topic_fails() -> None:
    """TS-19: Deliberate break — asserting a topic the playback never delivers MUST fail (LR-5)."""
    # A spec for a topic that is not in the bag / not subscribed → the comparator must report failure.
    specs = [AssertionSpec(topic="/patrol/this_topic_was_dropped", min_count=1)]
    observed = [ObservedTopic("/patrol/mission_state", count=200, duration_s=20.0)]

    result = evaluate(specs, observed)
    assert result.passed is False
    assert any("this_topic_was_dropped" in f for f in result.failures)
