"""tag_id -> checkpoint_id resolution (M6.A, T A.3 — design §4.2.5, PCAP-2/PCAP-7).

Maps an AprilTag detection to the semantic ``checkpoint_id`` from 03's config (built by
:class:`~patrol_perception.checkpoint_config.CheckpointConfigLoader`). Guards enforce that the
id is honestly identified — an unmapped tag_id or a tag-family mismatch raises rather than
fabricating a checkpoint_id (AC-4). The caller (CaptureCoordinator) treats the raise as a
skip and does NOT latch the visit, so a re-trigger can retry (design §4.2.8/§4.4.5).

ROS-free: the detection is *duck-typed* (``.id`` int, ``.family`` str, optional
``.decision_margin`` / ``.hamming``), matching apriltag_msgs/msg/AprilTagDetection (Research D3)
without importing it — so the resolver is unit-testable with a plain stand-in (AC-5). PCAP-7
detection confidence is sourced from ``decision_margin`` (Research D2).
"""

from __future__ import annotations

from typing import Any

from patrol_perception.checkpoint_config import CheckpointEntry


class CheckpointResolverError(ValueError):
    """Raised when a detection cannot be honestly resolved to a configured checkpoint."""


class CheckpointResolver:
    """Resolves an apriltag detection to ``(checkpoint_id, metadata)`` (design §4.2.5)."""

    def __init__(self, entries: dict[int, CheckpointEntry]):
        self._entries = entries

    def resolve(self, detection: Any) -> tuple[str, dict[str, str]]:
        entry = self._entries.get(detection.id)
        if entry is None:
            raise CheckpointResolverError(
                f"unmapped tag_id {detection.id} (not present in the checkpoint config)"
            )
        if not _families_match(detection.family, entry.tag_family):
            raise CheckpointResolverError(
                f"tag family mismatch for tag_id {detection.id}: "
                f"detection '{detection.family}' != config '{entry.tag_family}'"
            )
        return entry.checkpoint_id, _detection_metadata(detection)


def _families_match(detected: str, configured: str) -> bool:
    """True iff two AprilTag family labels name the same family.

    apriltag_ros emits the bare family token its ``family`` parameter was set to (e.g. ``"36h11"``),
    while 03's ``checkpoints.yaml`` uses the conventional ``"tag"``-prefixed form (``"tag36h11"``).
    Both name the same family, so the guard normalizes the optional ``tag`` prefix before comparing —
    a real mismatch (e.g. ``36h11`` vs ``25h9``) still fails loud (AC-4: no silent frame/tag bug).
    """
    return _normalize_family(detected) == _normalize_family(configured)


def _normalize_family(family: str) -> str:
    """Strip the optional conventional ``tag`` prefix so ``36h11`` and ``tag36h11`` compare equal."""
    return family[3:] if family.startswith("tag") else family


def _detection_metadata(detection: Any) -> dict[str, str]:
    """Build the PCAP-7 confidence/quality metadata from a detection (all values stringly typed)."""
    metadata: dict[str, str] = {"tag_id": str(detection.id)}
    confidence = getattr(detection, "decision_margin", None)
    if confidence is not None:
        metadata["detection_confidence"] = str(confidence)
    hamming = getattr(detection, "hamming", None)
    if hamming is not None:
        metadata["tag_hamming"] = str(hamming)
    return metadata
