"""Unit tests for the perception samplers' ROS-free logic (M6.B, T B.2 — design §4.2.7).

Layer-A: no rclpy, no Gazebo, no PX4. The samplers own rclpy subscriptions in the live node,
but their *logic* — latest-frame buffering, cv_bridge-seam encoding, and NED->ENU pose stamping
via the single MC-7 site (frames.to_enu_from_ned) — is exercised here with plain stand-ins (AC-5).
"""

from types import SimpleNamespace

import pytest
from patrol_perception.samplers import FrameSampler, LatestBuffer, PoseSampler

# --- LatestBuffer: keep-latest, return-or-None (shared by both samplers) ---


def test_latest_buffer_starts_empty():
    assert LatestBuffer().latest() is None


def test_latest_buffer_returns_most_recent():
    buf = LatestBuffer()
    buf.update("a")
    buf.update("b")
    assert buf.latest() == "b"


# --- FrameSampler: buffer the latest Image, encode via the injected cv_bridge seam ---


def test_frame_sampler_no_frame_returns_none():
    # No frame buffered yet -> take_latest yields None (coordinator treats this as a skip).
    assert FrameSampler(encoder=_fake_encoder).take_latest() is None


def test_frame_sampler_encodes_latest_frame():
    sampler = FrameSampler(encoder=_fake_encoder)
    sampler.update("frame-1")
    sampler.update("frame-2")
    result = sampler.take_latest()
    # Returns (image_msg, encoded_bytes) for the MOST RECENT frame (ADR-B latest-frame).
    assert result == ("frame-2", b"encoded:frame-2")


# --- PoseSampler: buffer latest NED pose, return it in world/ENU with explicit frame_id ---


def _ned_pose(x, y, z):
    # Minimal VehicleLocalPosition stand-in: x,y,z in NED + a quaternion.
    return SimpleNamespace(x=x, y=y, z=z, q=[0.0, 0.0, 0.0, 1.0])


def test_pose_sampler_no_pose_returns_none():
    assert PoseSampler(world_frame="map", ekf_origin_ned=(0.0, 0.0, 0.0)).sample() is None


def test_pose_sampler_converts_ned_to_enu_and_stamps_frame():
    sampler = PoseSampler(world_frame="patrol_world", ekf_origin_ned=(0.0, 0.0, 0.0))
    sampler.update(_ned_pose(2.0, 1.0, -3.0))  # NED -> ENU (1, 2, 3)
    sample = sampler.sample()
    assert sample.frame_id == "patrol_world"
    assert sample.position == pytest.approx((1.0, 2.0, 3.0))
    assert sample.orientation == (0.0, 0.0, 0.0, 1.0)


def test_pose_sampler_applies_ekf_origin_offset():
    sampler = PoseSampler(world_frame="patrol_world", ekf_origin_ned=(10.0, 20.0, 30.0))
    sampler.update(_ned_pose(12.0, 21.0, 27.0))  # rel NED (2,1,-3) -> ENU (1,2,3)
    assert sampler.sample().position == pytest.approx((1.0, 2.0, 3.0))


def _fake_encoder(image_msg):
    """Stand-in for the cv_bridge/OpenCV encode seam: deterministic, ROS-free."""
    return f"encoded:{image_msg}".encode()
