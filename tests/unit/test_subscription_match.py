"""Layer-A cover for the DDS-matching predicate behind the SITL abort-delivery wait (High #1).

The predicate lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(the same bootstrap ``test_dwell_tracker`` uses).

Why this exists: the wait it backs used to ask ``publisher.get_subscription_count() > 0``. Since
F-09 the acceptance watcher subscribes to /patrol/abort too, and the watcher shares a process with
the publisher while the mission node is separate — so the count reached 1 from the watcher, the
volatile abort published into nothing, and the scenario failed as an opaque timeout. The
``watcher present / mission absent -> False`` case below IS that bug.

The wait's *other* half broke the same way one round later: the count clause stayed pinned to a
literal 1, which one of two required subscribers satisfies on its own. Both halves now live in
``subscription_match``, and ``subscriptions_ready`` is covered below —
``round-2-bug-IS-F-01-watcher-unmatched`` is that second bug.
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from subscription_match import (  # noqa: E402  (import after the path bootstrap)
    WATCHER_NODE_NAME,
    has_subscriber,
    subscriptions_ready,
)

MISSION = "patrol_mission"
WATCHER = WATCHER_NODE_NAME  # one literal, shared with the watcher's own node_name default


class _Endpoint:
    """The one attribute this predicate reads off an rclpy ``TopicEndpointInfo``."""

    def __init__(self, node_name: str) -> None:
        self.node_name = node_name


@pytest.mark.parametrize(
    ("node_names", "expected"),
    [
        pytest.param([], False, id="no-endpoints-at-all"),
        pytest.param([WATCHER], False, id="watcher-present-mission-absent-IS-the-bug"),
        pytest.param([MISSION], True, id="mission-alone"),
        pytest.param([WATCHER, MISSION], True, id="both-present"),
        pytest.param([MISSION, WATCHER], True, id="both-present-reverse-discovery-order"),
        pytest.param(["patrol_mission_node"], False, id="near-miss-name-is-not-a-match"),
        pytest.param(["patrol_missio"], False, id="prefix-of-the-name-is-not-a-match"),
    ],
)
def test_has_subscriber_matches_only_the_named_node(node_names, expected):
    endpoints = [_Endpoint(n) for n in node_names]
    assert has_subscriber(endpoints, MISSION) is expected


# An endpoint object without a node_name must not raise — the predicate degrades to "not a match"
# rather than crashing the wait loop on an unexpected rclpy shape.
def test_has_subscriber_tolerates_an_endpoint_without_a_node_name():
    assert has_subscriber([object()], MISSION) is False


# The full readiness rule: every required node discovered AND the publisher matched at least that
# many subscriptions. Each half has broken once, so both directions are pinned here — this lane
# cannot run before merge, which makes this the only gate the fix gets.
@pytest.mark.parametrize(
    ("required", "present", "matched_count", "expected"),
    [
        pytest.param((MISSION,), [WATCHER], 1, False, id="round-1-bug-count-1-but-wrong-identity"),
        pytest.param(
            (MISSION, WATCHER), [MISSION], 1, False, id="round-2-bug-IS-F-01-watcher-unmatched"
        ),
        pytest.param(
            (MISSION, WATCHER),
            [MISSION, WATCHER],
            1,
            False,
            id="both-discovered-but-only-one-matched",
        ),
        pytest.param(
            (MISSION, WATCHER), [MISSION, WATCHER], 2, True, id="both-required-both-matched"
        ),
        pytest.param(
            (MISSION,), [MISSION], 1, True, id="low-battery-shape-one-subscriber-suffices"
        ),
        pytest.param(
            (MISSION,), [MISSION, WATCHER], 2, True, id="extra-observer-must-not-break-the-wait"
        ),
        pytest.param((), [MISSION], 1, False, id="no-required-nodes-is-not-readiness"),
    ],
)
def test_subscriptions_ready_requires_every_named_node_and_a_matching_count(
    required, present, matched_count, expected
):
    endpoints = [_Endpoint(n) for n in present]
    assert subscriptions_ready(endpoints, required, matched_count) is expected
