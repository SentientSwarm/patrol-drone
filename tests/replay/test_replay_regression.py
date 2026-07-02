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

import re
import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Int32, String

# Self-bootstrap the dir this test imports first-party modules from, so the imports do not depend on
# the lane's pytest `pythonpath` — the /tmp-config rootdir bug (PR #16 / F-01) broke exactly that
# dependence. Mirrors tests/integration/test_upload_ingest_standin.py, which is immune for this reason.
#   tests/replay (here) → replay_assertions
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

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
# "Topic: /name | Type: pkg/msg/Name | ..." from `ros2 bag info` — used to resolve a topic's type by
# name (so non-std types are counted from the stream without a hard-coded import).
_TOPIC_TYPE_RE = re.compile(r"Topic:\s*(\S+)\s*\|\s*Type:\s*(\S+)")


def _require_reference_bag() -> None:
    """Fail loudly if the LFS reference bag is absent/unresolved — a hard error, not a skip (§4.4.5)."""
    mcap = next(_REFERENCE_BAG.glob("*.mcap"), None)
    if mcap is None or mcap.stat().st_size < 1024:
        raise FileNotFoundError(
            f"reference bag missing or an unresolved LFS pointer at {_REFERENCE_BAG} — "
            "checkout needs `lfs: true`; the replay test requires the bag (it does not skip)."
        )


class _CountingNode(Node):
    """Subscribes to a set of topics and counts every message DELIVERED by playback.

    All asserted topics are counted from the played stream (never from `ros2 bag info`), so a topic
    dropped in playback is caught (F-04). Non-std types are resolved by name via ``get_message`` and
    subscribed ``raw=True`` (bytes, no deserialize) so the counter needs no hard-coded imports of
    ``sensor_msgs`` / ``px4_msgs`` / ``patrol_interfaces``.
    """

    def __init__(self, topic_types: dict[str, type]) -> None:
        super().__init__("replay_counter")
        self.counts: dict[str, int] = dict.fromkeys(topic_types, 0)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic, msg_type in topic_types.items():
            self.create_subscription(msg_type, topic, self._make_cb(topic), qos, raw=True)

    def _make_cb(self, topic: str):
        def _cb(_msg: object) -> None:
            self.counts[topic] += 1

        return _cb


# std_msgs topics whose type we import directly; every other asserted topic's type is resolved by
# name from `ros2 bag info` (see _resolve_types) so the counter imports no non-std message packages.
_STD_TYPES = {
    "/patrol/mission_state": String,
    "/patrol/current_waypoint": Int32,
}


def _resolve_types(bag: Path, topics: set[str]) -> dict[str, type]:
    """Map each asserted topic to its concrete message type, so ALL are counted from the stream.

    std_msgs types come from the direct imports; the rest are resolved by name (from `ros2 bag
    info`'s reported ``Type:``) via ``get_message`` — no hard-coded non-std imports needed.
    """
    info = subprocess.run(
        ["ros2", "bag", "info", str(bag)], check=True, capture_output=True, text=True
    )
    types_by_topic = dict(_TOPIC_TYPE_RE.findall(info.stdout))
    resolved: dict[str, type] = {}
    for topic in topics:
        if topic in _STD_TYPES:
            resolved[topic] = _STD_TYPES[topic]
        else:
            resolved[topic] = get_message(types_by_topic[topic])
    return resolved


def _play_and_count(
    bag: Path, topic_types: dict[str, type], window_s: float
) -> list[ObservedTopic]:
    """Play ``bag`` and return per-topic ObservedTopic counts over the playback window.

    Asserts ``ros2 bag play`` exits 0 — a player that errored out is a failure, not a silent pass.
    """
    rclpy.init()
    node = _CountingNode(topic_types)
    player = subprocess.Popen(["ros2", "bag", "play", "--rate", str(_PLAY_RATE), str(bag)])
    try:
        start = time.monotonic()
        while player.poll() is None and time.monotonic() - start < window_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        returncode = player.wait(timeout=10)
        elapsed = time.monotonic() - start
        assert returncode == 0, f"ros2 bag play exited {returncode}"
        return [ObservedTopic(t, node.counts[t], elapsed) for t in topic_types]
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
    topic is present at its rate — counted from the PLAYED STREAM (not `ros2 bag info`), so a topic
    dropped in playback fails the gate (F-04). _require_reference_bag covers TS-20 (LFS pointer → hard
    fail); _play_and_count asserts the player exited 0."""
    _require_reference_bag()
    specs = load_specs(_ASSERTIONS)
    topic_types = _resolve_types(_REFERENCE_BAG, {s.topic for s in specs})

    observed = _play_and_count(_REFERENCE_BAG, topic_types, window_s=80.0)

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
