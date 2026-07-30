"""Layer-A cover for the DDS-matching predicate behind the SITL abort-delivery wait (High #1).

The predicate lives beside the rclpy-importing ``patrol_acceptance`` module under
``tests/integration``; it imports nothing heavy, so this Layer-A test pulls it in via a path insert
(the same bootstrap ``test_dwell_tracker`` uses).

Why this exists: the wait it backs used to ask ``publisher.get_subscription_count() > 0``. Since
F-09 the acceptance watcher subscribes to /patrol/abort too, and the watcher shares a process with
the publisher while the mission node is separate — so the count reached 1 from the watcher, the
volatile abort published into nothing, and the scenario failed as an opaque timeout. The
``watcher present / mission absent -> False`` case below IS that bug.
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from subscription_match import has_subscriber  # noqa: E402  (import after the path bootstrap)

MISSION = "patrol_mission"
WATCHER = "patrol_acceptance_watcher"


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
