"""Whether a named node holds a discovered subscriber on a topic — the DDS-matching predicate.

Split out of ``patrol_acceptance`` (which imports rclpy at module scope) so the decision is
Layer-A testable, matching ``dwell_tracker`` / ``home_settle_tracker``. Takes an iterable of
endpoint-info objects — i.e. whatever ``Node.get_subscriptions_info_by_topic`` returned — so the
test can feed plain stubs and never needs a live graph.

Counting subscribers is what this replaces, and counting is what broke: since F-09 the acceptance
watcher itself subscribes to /patrol/abort, so a bare ``get_subscription_count() > 0`` is satisfied
by the watcher (same process as the publisher, so it matches first) while the mission node is still
undiscovered. Identity, not population, is the question a delivery guarantee actually asks.
"""


def has_subscriber(endpoints, node_name: str) -> bool:
    """Whether any endpoint in ``endpoints`` belongs to ``node_name``."""
    return any(getattr(e, "node_name", None) == node_name for e in endpoints)
