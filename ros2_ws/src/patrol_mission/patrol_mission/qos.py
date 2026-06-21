"""Shared QoS profiles for the patrol stack (design §4.4.2).

One definition per messaging surface, imported by the node *and* the acceptance harness, so a
publisher and its subscriber are guaranteed-compatible and the profile is not re-derived per file
(the CodeScene duplication trap on M-milestone test PRs). Imports rclpy, so this is Layer-B — not
part of the ROS-free unit-coverage gate.

The /patrol/* surfaces have different needs (Hermes Medium), so they get distinct profiles
rather than one shared one:
  * observable state (mission_state/current_waypoint) is *latched* (transient-local) so a late
    04/05 subscriber sees the latest sample;
  * the /patrol/dwell capture event is a discrete live event — reliable + volatile, keep-last with a
    route-covering depth so every checkpoint is delivered once and never coalesced to "latest";
  * the inbound /patrol/abort *command* is volatile, so a plain ``ros2 topic pub`` (the documented
    manual abort) is QoS-compatible. The abort "sticks" through RTH via the state machine's
    _NON_ABORTABLE latch — not via topic durability.
"""

from __future__ import annotations

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def px4_qos() -> QoSProfile:
    """The /fmu/* QoS PX4's uXRCE-DDS bridge uses (best-effort + transient-local, depth 1)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def patrol_state_qos() -> QoSProfile:
    """The /patrol/{mission_state,current_waypoint} QoS (reliable + transient-local, depth 1).

    Transient-local depth-1 latches the observable mission surface so a late subscriber (04/05
    starting after the node) immediately sees the latest state/waypoint sample.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


_PATROL_EVENT_DEPTH = 16  # covers a full patrol's checkpoint events without coalescing to "latest"


def patrol_event_qos() -> QoSProfile:
    """The /patrol/dwell QoS (reliable + volatile, keep-last depth covering a route).

    A *discrete live event* (one message per DWELL entry), not latched observable state: every
    checkpoint event must reach a connected subscriber exactly once, so it is keep-last with a
    route-covering depth rather than the depth-1 latch the state surface uses (depth 1 would coalesce
    rapid events to only the latest). Volatile for the same reason the abort command is — it is a
    momentary event, not state a late subscriber should be handed; 04/05 are expected up before the
    patrol starts. This is the atomic OQ-7 capture trigger (Hermes High).
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=_PATROL_EVENT_DEPTH,
    )


def patrol_abort_qos() -> QoSProfile:
    """The /patrol/abort QoS (reliable + volatile, depth 1).

    The abort is an inbound *command*, not latched observable state, so it is volatile: a plain
    ``ros2 topic pub -1`` (which waits for the node's subscription before publishing) is then
    QoS-compatible and delivered. The abort still "sticks" through the whole return home — that is
    the state machine's _NON_ABORTABLE latch after receipt, not topic durability.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
