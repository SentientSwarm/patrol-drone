"""Trigger -> one-capture-per-visit orchestration + the AC-6 latch (M6.B, T B.3 — design §4.2.8).

CaptureCoordinator is the node-layer orchestrator, but its logic is collaborator-injected and
ROS-free so the latch + ADR-A gate are unit-tested directly (AC-5/AC-6). On 02's trigger it gates
on a tag in view (ADR-A), samples the latest frame+pose (ADR-B), resolves the checkpoint_id from
the detection (PCAP-2, never fabricated — AC-4), builds + publishes the CheckpointCapture, and
ONLY THEN latches the visit token. Any skip (no tag, no frame, no pose, unmapped tag_id) leaves the
token unlatched so a re-trigger for the same visit can retry (§4.2.8 latch-only-on-success).

The CaptureWriter (on-disk persistence) is M6.C; in M6.B ``writer`` is ``None`` and the coordinator
publishes only. The detection is duck-typed (apriltag_msgs/msg/AprilTagDetection, T B.1).
"""

from __future__ import annotations

import logging
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from typing import Any

from patrol_perception.capture_builder import CaptureRecord
from patrol_perception.checkpoint_resolver import CheckpointResolverError

_log = logging.getLogger("patrol_perception.coordinator")


@dataclass(frozen=True)
class CapturePipeline:
    """The capture pipeline's injected collaborators, grouped so the coordinator constructor stays
    small (one params object, not seven positional collaborators). All duck-typed (M6 seams)."""

    frame_sampler: Any
    pose_sampler: Any
    detection_buffer: Any
    resolver: Any
    builder: Any
    publisher: Any
    writer: Any | None


class CaptureCoordinator:
    """Orchestrates trigger -> sample -> resolve -> publish/write -> latch (one per visit)."""

    def __init__(
        self,
        *,
        pipeline: CapturePipeline,
        clock: Callable[[], tuple[int, int]],
        mission_id: str = "",
    ) -> None:
        self._frame_sampler = pipeline.frame_sampler
        self._pose_sampler = pipeline.pose_sampler
        self._detection_buffer = pipeline.detection_buffer
        self._resolver = pipeline.resolver
        self._builder = pipeline.builder
        self._publisher = pipeline.publisher
        self._writer = pipeline.writer
        self._clock = clock
        self._mission_id = mission_id
        self._latched_token: Any = _UNSET

    def on_trigger(self, visit_token: Any) -> None:
        """Capture once for ``visit_token``; idempotent within a visit, retryable across skips."""
        if visit_token == self._latched_token:
            return  # AC-6: this visit already captured — no duplicate.

        detection = self._first_detection()
        if detection is None:
            _log.info("skip visit %s: no tag in view (ADR-A gate)", visit_token)
            return  # not latched -> a re-trigger retries once a tag is in view

        sample = self._sample_world()
        if sample is None:
            return  # no frame and/or pose buffered yet — skip, stay retryable

        image_bytes, pose = sample
        try:
            checkpoint_id, metadata = self._resolver.resolve(detection)
        except CheckpointResolverError as exc:
            _log.info("skip visit %s: unresolved detection (%s)", visit_token, exc)
            return  # AC-4: no fabricated checkpoint_id; not latched -> retryable

        # T C.3 / PCAP-6: merge run + visit context with the resolver's detection metadata. The
        # /patrol/dwell trigger value IS the dwelled waypoint index, so visit_token == waypoint_index.
        metadata = {
            **metadata,
            "mission_id": self._mission_id,
            "waypoint_index": str(visit_token),
        }
        self._emit(checkpoint_id, metadata, pose, image_bytes)
        # Latch after the publish (the topic is the bag's source of truth, §4.4.5): a write failure
        # is a degraded success, not a skip — re-triggering must NOT re-publish for this visit (AC-6).
        self._latched_token = visit_token

    def _first_detection(self) -> Any | None:
        """The tag-in-view for this visit: the first buffered detection, or None if none in view."""
        detections: Sequence[Any] | None = self._detection_buffer.latest()
        if not detections:
            return None
        return detections[0]

    def _sample_world(self) -> tuple[bytes, Any] | None:
        """Latest encoded frame + ENU pose (ADR-B). None if either is unavailable (skip, retryable)."""
        frame = self._frame_sampler.take_latest()
        pose = self._pose_sampler.sample()
        if frame is None or pose is None:
            return None
        _image_msg, image_bytes = frame
        return image_bytes, pose

    def _emit(self, checkpoint_id: str, metadata: dict, pose: Any, image_bytes: bytes) -> None:
        """Build the CaptureRecord, persist (M6.C) if a writer is wired, then publish (PCAP-3).

        Per design §4.4.5: if the writer raises (``output_root`` unwritable / disk full), log
        ``capture_write_failed`` and still publish with an empty ``image_path`` and an
        ``image_write_status="failed"`` metadata flag — the topic is the bag's source of truth, so a
        persistence failure must NOT suppress the capture message (continue patrol).
        """
        sec, nanosec = self._clock()
        record = CaptureRecord(
            stamp_sec=sec,
            stamp_nanosec=nanosec,
            frame_id=pose.frame_id,
            checkpoint_id=checkpoint_id,
            position=pose.position,
            orientation=pose.orientation,
            image_path="",  # set below on a successful write; left empty on a write failure
            metadata={**metadata, "image_write_status": "ok"},
        )
        record = self._persist(record, image_bytes)
        self._publisher.publish(self._builder.build_message(record))

    def _persist(self, record: CaptureRecord, image_bytes: bytes) -> CaptureRecord:
        """Write the image+sidecar if a writer is wired; on OSError, degrade per §4.4.5 (publish
        anyway with an empty path + ``image_write_status="failed"``). No writer wired -> unchanged."""
        if self._writer is None:
            return record
        try:
            return replace(record, image_path=self._writer.write(record, image_bytes))
        except OSError as exc:
            _log.warning("capture_write_failed for %s: %s", record.checkpoint_id, exc)
            return replace(record, metadata={**record.metadata, "image_write_status": "failed"})


_UNSET = object()
