"""Whether a named node holds a discovered subscriber on a topic — the DDS-matching predicate.

Split out of ``patrol_acceptance`` (which imports rclpy at module scope) so the decision is
Layer-A testable, matching ``dwell_tracker`` / ``home_settle_tracker``. Takes an iterable of
endpoint-info objects — i.e. whatever ``Node.get_subscriptions_info_by_topic`` returned — so the
test can feed plain stubs and never needs a live graph.

Counting subscribers is what this replaces, and counting is what broke: since F-09 the acceptance
watcher itself subscribes to /patrol/abort, so a bare ``get_subscription_count() > 0`` is satisfied
by the watcher (same process as the publisher, so it matches first) while the mission node is still
undiscovered. Identity, not population, is the question a delivery guarantee actually asks.

The *whole* rule lives here, count clause included — ``subscriptions_ready`` below. An earlier split
kept identity here and left the count in the rclpy-importing caller, and the count is the half that
then broke a second time (pinned to a literal 1, so one of two required subscribers satisfied it).
Both halves belong on the Layer-A side of the line: the lane that exercises them cannot run before
merge, so a unit test over this module is the only gate either half gets.
"""

from collections.abc import Iterable

# The acceptance watcher's ROS node name. It lives here, beside the predicate that has to name it,
# because this is the only module under tests/integration that imports no rclpy — so the Layer-B
# watcher, the call site that must wait for it, and the Layer-A test can all share ONE literal.
# (Its sibling MISSION_NODE_NAME belongs to patrol_mission.topics: that one is the mission node's
# own contract surface; this one is test scaffolding and does not belong in the production package.)
WATCHER_NODE_NAME = "patrol_acceptance_watcher"


def has_subscriber(endpoints, node_name: str) -> bool:
    """Whether any endpoint in ``endpoints`` belongs to ``node_name``."""
    return any(getattr(e, "node_name", None) == node_name for e in endpoints)


def subscriptions_ready(endpoints, node_names: Iterable[str], matched_count: int) -> bool:
    """Whether EVERY required node is discovered AND the publisher has matched that many subs.

    The count half is not redundant with the identity half, and both have now broken once:

    * count alone (round 1, High #1) is satisfied by whichever subscriber matched first — since F-09
      the in-process watcher — while the mission node is still undiscovered.
    * a count *pinned to 1* (the state this replaces) is satisfied while only one of two required
      subscribers has matched, so a volatile sample can reach the mission and not the watcher.

    So the count scales with the required set instead of being a literal: graph presence says
    discovery reported an endpoint, ``matched_count`` says this publisher actually matched that
    many — and neither implies the other.
    """
    names = tuple(node_names)
    if not names:
        return False  # nothing named is not readiness — a caller must say WHO must be matched
    return matched_count >= len(names) and all(has_subscriber(endpoints, n) for n in names)
