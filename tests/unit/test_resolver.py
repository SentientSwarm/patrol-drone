"""Layer-A unit tests for patrol_perception.checkpoint_resolver (M6.A, T A.3).

Covers TS-3 (checkpoint_id sourced from the config map, not a constant — AC-4), TS-4 (unmapped
tag_id is rejected, no fabricated id — AC-4), TS-5 (family mismatch rejected). ROS-free: the
resolver consumes a *duck-typed* apriltag detection (``.id`` / ``.family`` / ``.decision_margin``)
so no apriltag_msgs/rclpy import is needed (AC-5). PCAP-7 confidence passthrough uses
``decision_margin`` (Research D2).
"""

from dataclasses import dataclass

import pytest
from patrol_perception.checkpoint_config import CheckpointEntry
from patrol_perception.checkpoint_resolver import (
    CheckpointResolver,
    CheckpointResolverError,
)


@dataclass
class _Detection:
    """Stand-in for apriltag_msgs/msg/AprilTagDetection (duck-typed)."""

    id: int
    family: str = "tag36h11"
    decision_margin: float | None = None
    hamming: int | None = None


_ENTRIES = {
    0: CheckpointEntry("cp_north", 0, "tag36h11", (12.0, 8.0, 1.5)),
    1: CheckpointEntry("cp_east", 1, "tag36h11", (18.0, -6.0, 1.5)),
}


def test_resolves_checkpoint_id_from_config_map():
    """TS-3: checkpoint_id comes from the loaded map keyed by tag_id, not a constant."""
    resolver = CheckpointResolver(_ENTRIES)

    cid_a, meta_a = resolver.resolve(_Detection(id=0))
    cid_b, meta_b = resolver.resolve(_Detection(id=1))

    assert cid_a == "cp_north"
    assert cid_b == "cp_east"
    assert meta_a["tag_id"] == "0"
    assert meta_b["tag_id"] == "1"


def test_passes_through_detection_confidence():
    """TS-3/PCAP-7: decision_margin is surfaced as detection_confidence metadata (Research D2)."""
    resolver = CheckpointResolver(_ENTRIES)

    _, meta = resolver.resolve(_Detection(id=1, decision_margin=42.5, hamming=1))

    assert meta["detection_confidence"] == "42.5"
    assert meta["tag_hamming"] == "1"


def test_confidence_omitted_when_absent():
    """A detection with no decision_margin still resolves; the key is simply absent."""
    resolver = CheckpointResolver(_ENTRIES)

    _, meta = resolver.resolve(_Detection(id=0))

    assert "detection_confidence" not in meta


def test_rejects_unmapped_tag_id():
    """TS-4: an unmapped tag_id raises (no fabricated checkpoint_id, AC-4)."""
    resolver = CheckpointResolver(_ENTRIES)

    with pytest.raises(CheckpointResolverError, match="unmapped tag_id"):
        resolver.resolve(_Detection(id=99))


def test_rejects_family_mismatch():
    """TS-5: a detection whose family differs from the config row is rejected."""
    resolver = CheckpointResolver(_ENTRIES)

    with pytest.raises(CheckpointResolverError, match="family"):
        resolver.resolve(_Detection(id=0, family="tag25h9"))
