"""CheckpointCapture message + sidecar construction (M6.A, T A.4 — design §4.2.4).

One ``CaptureRecord`` is the single source for two surfaces (PCAP-6 single shape):
the published ``patrol_interfaces/msg/CheckpointCapture`` and the on-disk JSON sidecar.

The message is assembled through an injected (duck-typed) message-factory seam so this module
stays ROS-free and unit-testable with a plain stand-in (AC-5) — the real rosidl-backed factory is
wired by the node in M6.B. ``build_sidecar`` is pure stdlib. Together they are the AC-3/PCAP-6
unit surface: built from plain ``CaptureRecord`` instances, no ROS spin-up, sub-second.

The ``factory`` passed to :class:`CheckpointCaptureBuilder` must provide four methods (the M6.B
rosidl factory and the unit-test stand-in both implement these):

* ``new_capture() -> CheckpointCapture`` — an empty message to populate
* ``make_header(sec, nanosec, frame_id) -> std_msgs/Header``
* ``make_pose_stamped(rec: CaptureRecord) -> geometry_msgs/PoseStamped``
* ``make_key_value(key, value) -> diagnostic_msgs/KeyValue``
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CaptureRecord:
    """Plain-data source for one capture; ROS-free (no message/rclpy types)."""

    stamp_sec: int
    stamp_nanosec: int
    frame_id: str  # world/ENU frame name (ADR-B)
    checkpoint_id: str  # resolved from the tag (PCAP-2)
    position: tuple[float, float, float]  # x, y, z in world/ENU meters
    orientation: tuple[float, float, float, float]  # x, y, z, w
    image_path: str  # path written by CaptureWriter (PCAP-5)
    metadata: dict[str, str]  # single KV source for message + sidecar (PCAP-6)


class CheckpointCaptureBuilder:
    """Builds the CheckpointCapture message and its JSON sidecar from a CaptureRecord.

    ``factory`` is duck-typed (see the module docstring for the four methods it must provide):
    M6.B injects a rosidl-backed factory; unit tests inject a SimpleNamespace stand-in (AC-5).
    """

    def __init__(self, factory: Any):
        self._factory = factory

    def build_message(self, rec: CaptureRecord) -> Any:
        """Populate every CheckpointCapture field from ``rec`` (AC-3). Guard: checkpoint_id
        non-empty (AC-4 no-fabrication corollary). The pose carries ``rec.frame_id`` by
        construction, satisfying the design §4.2.4 ``pose.frame_id == rec.frame_id`` guard."""
        if not rec.checkpoint_id:
            raise ValueError("checkpoint_id must be non-empty to publish a CheckpointCapture")
        factory = self._factory
        msg = factory.new_capture()
        msg.header = factory.make_header(rec.stamp_sec, rec.stamp_nanosec, rec.frame_id)
        msg.checkpoint_id = rec.checkpoint_id
        msg.pose = factory.make_pose_stamped(rec)
        msg.image_path = rec.image_path
        msg.metadata = [factory.make_key_value(key, value) for key, value in rec.metadata.items()]
        return msg

    @staticmethod
    def build_sidecar(rec: CaptureRecord) -> dict[str, Any]:
        """Return the JSON-serializable sidecar carrying the SAME checkpoint_id / pose / stamp /
        metadata as the message, plus the image basename (design §4.2.4, PCAP-3 consistency).

        Static/pure: a function of ``rec`` only (no factory), so CaptureWriter can build the sidecar
        without constructing a builder, while the published message still uses the same record."""
        px, py, pz = rec.position
        ox, oy, oz, ow = rec.orientation
        return {
            "checkpoint_id": rec.checkpoint_id,
            "stamp": {"sec": rec.stamp_sec, "nanosec": rec.stamp_nanosec},
            "frame_id": rec.frame_id,
            "pose": {
                "position": {"x": px, "y": py, "z": pz},
                "orientation": {"x": ox, "y": oy, "z": oz, "w": ow},
            },
            "image": os.path.basename(rec.image_path),
            "metadata": dict(rec.metadata),
        }
