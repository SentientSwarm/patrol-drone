"""Layer-A unit tests for patrol_perception.capture_builder (M6.A, T A.4).

Covers TS-1 (build_message populates every field — AC-3) and TS-2 (build_sidecar carries the
identical KV set + pose/checkpoint/stamp as the message — PCAP-6 consistency, AC-2/AC-3).
ROS-free: build_message assembles the message through an injected message-factory seam, so the
unit test passes a SimpleNamespace-backed fake — no rosidl/rclpy import, sub-second (AC-5). The
real rosidl-backed factory lands with the node in M6.B.
"""

import json
from types import SimpleNamespace

import pytest
from patrol_perception.capture_builder import CaptureRecord, CheckpointCaptureBuilder


class _FakeFactory:
    """A ROS-free stand-in for the rosidl message factory (M6.B provides the real one)."""

    def new_capture(self):
        return SimpleNamespace()

    def make_header(self, sec, nanosec, frame_id):
        return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec), frame_id=frame_id)

    def make_pose_stamped(self, sec, nanosec, frame_id, position, orientation):
        return SimpleNamespace(
            header=self.make_header(sec, nanosec, frame_id),
            pose=SimpleNamespace(
                position=SimpleNamespace(x=position[0], y=position[1], z=position[2]),
                orientation=SimpleNamespace(
                    x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3]
                ),
            ),
        )

    def make_key_value(self, key, value):
        return SimpleNamespace(key=key, value=value)


def _record(**overrides) -> CaptureRecord:
    base = {
        "stamp_sec": 100,
        "stamp_nanosec": 500,
        "frame_id": "map",
        "checkpoint_id": "cp_north",
        "position": (12.0, 8.0, 1.5),
        "orientation": (0.0, 0.0, 0.0, 1.0),
        "image_path": "/runs/run1/000_cp_north.png",
        "metadata": {"tag_id": "0", "detection_confidence": "42.5"},
    }
    base.update(overrides)
    return CaptureRecord(**base)


def _kv_dict(msg) -> dict[str, str]:
    return {kv.key: kv.value for kv in msg.metadata}


def test_build_message_populates_every_field():
    """TS-1 (AC-3): every CheckpointCapture field is populated from the CaptureRecord."""
    builder = CheckpointCaptureBuilder(_FakeFactory())

    msg = builder.build_message(_record())

    assert msg.checkpoint_id == "cp_north"
    assert msg.image_path == "/runs/run1/000_cp_north.png"
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (100, 500)
    assert msg.header.frame_id == "map"
    # Guard (design §4.2.4): pose carries the same world/ENU frame as the record.
    assert msg.pose.header.frame_id == "map"
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z) == (
        12.0,
        8.0,
        1.5,
    )
    assert msg.pose.pose.orientation.w == 1.0
    assert _kv_dict(msg) == {"tag_id": "0", "detection_confidence": "42.5"}


def test_build_message_rejects_empty_checkpoint_id():
    """Guard: an empty checkpoint_id must never be published (AC-4 no-fabrication corollary)."""
    builder = CheckpointCaptureBuilder(_FakeFactory())

    with pytest.raises(ValueError, match="checkpoint_id"):
        builder.build_message(_record(checkpoint_id=""))


def test_build_sidecar_matches_message():
    """TS-2 (PCAP-6): the sidecar carries the same checkpoint/pose/stamp/metadata as the message."""
    builder = CheckpointCaptureBuilder(_FakeFactory())
    rec = _record()

    msg = builder.build_message(rec)
    sidecar = builder.build_sidecar(rec)

    assert sidecar["checkpoint_id"] == msg.checkpoint_id
    assert sidecar["frame_id"] == "map"
    assert sidecar["stamp"] == {"sec": 100, "nanosec": 500}
    assert sidecar["pose"]["position"] == {"x": 12.0, "y": 8.0, "z": 1.5}
    assert sidecar["pose"]["orientation"] == {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    # Sidecar references the image by basename (design §4.2.4).
    assert sidecar["image"] == "000_cp_north.png"
    # The KV set is identical across both surfaces (the PCAP-6 single-shape guarantee).
    assert sidecar["metadata"] == _kv_dict(msg)


def test_build_sidecar_is_json_serializable():
    """TS-2: the sidecar round-trips through JSON (it is written to disk by CaptureWriter)."""
    builder = CheckpointCaptureBuilder(_FakeFactory())

    sidecar = builder.build_sidecar(_record())

    assert json.loads(json.dumps(sidecar)) == sidecar
