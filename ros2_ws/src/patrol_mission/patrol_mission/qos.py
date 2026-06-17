"""Shared QoS profiles for the patrol stack (design §4.4.2).

One definition each for the two messaging surfaces, imported by the node *and* the acceptance
harness, so a publisher and its subscriber are guaranteed-compatible and the profile is not
re-derived per file (the CodeScene duplication trap on M-milestone test PRs). Imports rclpy, so
this is Layer-B — not part of the ROS-free unit-coverage gate.
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


def patrol_qos() -> QoSProfile:
    """The /patrol/* QoS (reliable + transient-local, depth 1).

    Transient-local depth-1 lets a late subscriber (04/05 starting after the node) see the latest
    mission_state/current_waypoint/abort; reliable delivery means a connected subscriber never drops
    the single observable ABORT sample before it routes to RTH.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
