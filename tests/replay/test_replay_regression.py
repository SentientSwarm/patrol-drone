"""Replay regression test — the CI guard the bag becomes (docset 05, M8 / T8.8, SWM-80, LR-5).

Plays the checked-in reference bag via ``ros2 bag play`` while rclpy subscribers count messages per
topic, then asserts the result against the curated subset in ``assertions.yaml`` using the ROS-free
:func:`replay_assertions.evaluate` comparator (design §4.2.5). This is the "the bag is the
regression test" payoff: a later-phase change that drops a recorded topic is caught here in CI
before it reaches hardware (PRD H3 — deterministic, plays a fixed bag, not the simulator).

This is the ROS lane (``pytest.mark.ros``): it needs a sourced ROS env + ``ros2 bag play`` + the
LFS-materialized reference bag + the message packages for every asserted topic (std_msgs and
sensor_msgs from apt; px4_msgs and patrol_interfaces from the lane's cached colcon overlay), since
every topic is counted by live subscription (Mira High, review 4728294643). The pure comparison
logic is unit-tested separately in tests/unit/test_replay_assertions.py; here we drive the real
play→subscribe→evaluate path.

Budget: ≤ 90 s wall-clock (OQ-6; ~20 s live play + a ~5 s 4x-rate deliberate-break play). The
deliberate-break self-check (test_dropped_topic_fails_end_to_end) proves the guard actually guards
— an asserted topic withheld from playback must FAIL the assertions (LR-5 deliberate-break AC).
"""

from __future__ import annotations

import subprocess
import sys
import time
from pathlib import Path

import pytest
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

# Self-bootstrap the dir this test imports first-party modules from, so the import does not depend on
# the lane's pytest `pythonpath` — the /tmp-config rootdir bug (PR #16 / F-01) broke exactly that
# dependence. Mirrors tests/integration/test_upload_ingest_standin.py, which is immune for this reason.
#   tests/replay (here) → replay_assertions
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))

from replay_assertions import (  # noqa: E402  (after the sys.path bootstrap above)
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


# EVERY asserted topic is counted by LIVE subscription during playback (Mira High, review
# 4728294643) — no static `ros2 bag info` fallback: a bag-info count proves the bag CONTAINS a
# topic, not that playback DELIVERED it. The non-std message classes (px4_msgs/sensor_msgs/
# patrol_interfaces) are resolved by name at runtime from the bag's own metadata, so
# assertions.yaml stays the single source of asserted topics; the replay lane provides the message
# packages (apt type-support + the cached px4_msgs/patrol_interfaces colcon overlay) and numpy —
# the get_message prerequisites (see the repo's Jazzy-rclpy note).
def _recorded_topic_types(bag: Path) -> dict[str, str]:
    """topic -> ROS type string, read from the bag's own metadata.yaml (the recorded truth)."""
    meta = yaml.safe_load((bag / "metadata.yaml").read_text())
    entries = meta["rosbag2_bagfile_information"]["topics_with_message_count"]
    return {e["topic_metadata"]["name"]: e["topic_metadata"]["type"] for e in entries}


def _subscribed_types(bag: Path, topics: set[str]) -> dict[str, type]:
    """Resolve each asserted topic's message class by name — every topic is counted LIVE at
    playback; a topic absent from the bag or an uninstalled message package fails loudly here
    (KeyError / import error), never silently downgrades to a static count."""
    types = _recorded_topic_types(bag)
    return {t: get_message(types[t]) for t in topics}


def _play_and_count(
    bag: Path,
    topics: dict[str, type],
    window_s: float,
    *,
    rate: float = _PLAY_RATE,
    play_topics: list[str] | None = None,
) -> list[ObservedTopic]:
    """Play ``bag`` and return per-topic ObservedTopic counts over the playback window.

    Asserts ``ros2 bag play`` exits 0 — a player that errored out is a failure, not a silent pass
    (F-04). ``play_topics`` narrows what the PLAYER publishes (``--topics``) while the subscriber
    set stays ``topics`` — the deliberate-break test uses it to withhold one asserted topic from
    playback and prove the full play→count→evaluate path fails.
    """
    rclpy.init()
    node = _CountingNode(topics)
    argv = ["ros2", "bag", "play", "--rate", str(rate), str(bag)]
    if play_topics is not None:
        argv += ["--topics", *play_topics]
    player = subprocess.Popen(argv)
    try:
        start = time.monotonic()
        while player.poll() is None and time.monotonic() - start < window_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        returncode = player.wait(timeout=10)
        elapsed = time.monotonic() - start
        assert returncode == 0, f"ros2 bag play exited {returncode}"
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
    topic is present at its rate — ALL topics counted by live subscription during playback (Mira
    High, review 4728294643). _require_reference_bag covers TS-20 (LFS pointer → hard fail);
    _play_and_count asserts the player exited 0 (F-04)."""
    _require_reference_bag()
    specs = load_specs(_ASSERTIONS)
    types = _subscribed_types(_REFERENCE_BAG, {s.topic for s in specs})

    observed = _play_and_count(_REFERENCE_BAG, types, window_s=80.0)

    result = evaluate(specs, observed)
    assert result.passed, result.failures


def test_dropped_topic_fails_end_to_end() -> None:
    """TS-19: Deliberate break — play the reference bag with one asserted topic WITHHELD from
    playback (``--topics`` keeps the rest); the full play→subscribe→count→evaluate path MUST fail
    on exactly that topic (LR-5, Mira High: end-to-end, not comparator-only — the comparator-level
    break stays covered by tests/unit/test_replay_assertions.py). 4x rate keeps this second play
    ~5 s (OQ-6 budget); rate-band noise on OTHER topics at 4x is irrelevant — only the dropped
    topic's presence failure is asserted."""
    _require_reference_bag()
    specs = load_specs(_ASSERTIONS)
    dropped = "/patrol/mission_state"
    kept = [s.topic for s in specs if s.topic != dropped]
    types = _subscribed_types(_REFERENCE_BAG, {s.topic for s in specs})

    observed = _play_and_count(_REFERENCE_BAG, types, window_s=30.0, rate=4.0, play_topics=kept)

    result = evaluate(specs, observed)
    assert result.passed is False
    assert any(dropped in f for f in result.failures)
