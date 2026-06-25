"""Unit tests for the perception samplers' ROS-free logic (M6.B, T B.2 — design §4.2.7).

Layer-A: no rclpy, no Gazebo, no PX4. The samplers own rclpy subscriptions in the live node,
but their *logic* — latest-frame buffering, cv_bridge-seam encoding, and NED->ENU pose stamping
via the single MC-7 site (frames.to_enu_from_ned) — is exercised here with plain stand-ins (AC-5).
"""

import math
from types import SimpleNamespace

import pytest
from patrol_perception.samplers import FrameSampler, LatestBuffer, PoseSample, PoseSampler

# The samplers take the same (sec, nanosec) clock seam the CaptureCoordinator does (ADR-B freshness
# window): receipt time is stamped on each update so the coordinator can age-check the buffer. A
# fixed clock keeps the timestamp deterministic in unit tests (no wall-clock flakiness).
_RX = (100, 0)


def _clock_at(stamp=_RX):
    """A fixed (sec, nanosec) clock; shared so the constructions don't copy-paste a lambda."""
    return lambda: stamp


# --- LatestBuffer: keep-latest + receipt timestamp (shared by both samplers) ---


def test_latest_buffer_starts_empty():
    buf: LatestBuffer[str] = LatestBuffer(_clock_at())
    assert buf.latest() is None
    assert buf.latest_at() is None  # no timestamp before the first update


def test_latest_buffer_returns_most_recent():
    buf: LatestBuffer[str] = LatestBuffer(_clock_at())
    buf.update("a")
    buf.update("b")
    assert buf.latest() == "b"


def test_latest_buffer_records_receipt_timestamp():
    # latest_at() surfaces (value, received_at) so the coordinator can enforce the freshness window.
    buf: LatestBuffer[str] = LatestBuffer(_clock_at((7, 42)))
    buf.update("v")
    assert buf.latest_at() == ("v", (7, 42))


# --- FrameSampler: buffer the latest Image, encode via the injected cv_bridge seam ---


def test_frame_sampler_no_frame_returns_none():
    # No frame buffered yet -> take_latest yields None (coordinator treats this as a skip).
    assert FrameSampler(encoder=_fake_encoder, clock=_clock_at()).take_latest() is None


def test_frame_sampler_encodes_latest_frame():
    sampler = FrameSampler(encoder=_fake_encoder, clock=_clock_at())
    sampler.update("frame-1")
    sampler.update("frame-2")
    result = sampler.take_latest()
    # Returns (image_msg, encoded_bytes, received_at) for the MOST RECENT frame (ADR-B latest-frame).
    assert result == ("frame-2", b"encoded:frame-2", _RX)


def test_frame_sampler_surfaces_receipt_timestamp():
    # The receipt time flows out of take_latest() so the coordinator can age-check the frame.
    sampler = FrameSampler(encoder=_fake_encoder, clock=_clock_at((9, 5)))
    sampler.update("frame")
    result = sampler.take_latest()
    assert result is not None
    assert result[2] == (9, 5)


# --- PoseSampler: buffer latest NED pose, return it in world/ENU with explicit frame_id ---


def _ned_pose(x, y, z, heading=0.0):
    # Minimal VehicleLocalPosition stand-in: matches the REAL message field set — x,y,z in NED plus
    # a scalar `heading` (NED Euler yaw). VehicleLocalPosition carries NO quaternion field, so the
    # sampler must derive orientation from `heading`, not a fabricated `.q` (would have caught the
    # AttributeError that a `.q` stand-in masked).
    return SimpleNamespace(x=x, y=y, z=z, heading=heading)


def _enu_sampler(ekf_origin_ned=(0.0, 0.0, 0.0), clock=None) -> PoseSampler:
    """A PoseSampler wired to the world/ENU frame + a fixed clock (shared to avoid copy-paste)."""
    return PoseSampler(
        world_frame="patrol_world",
        ekf_origin_ned=ekf_origin_ned,
        clock=clock or _clock_at(),
    )


def _require_sample(sampler: PoseSampler) -> PoseSample:
    """Sample and assert a pose was produced (narrows the tuple|None for the assertions below)."""
    result = sampler.sample()
    assert result is not None
    pose_sample, _received_at = result
    return pose_sample


def test_pose_sampler_no_pose_returns_none():
    assert (
        PoseSampler(world_frame="map", ekf_origin_ned=(0.0, 0.0, 0.0), clock=_clock_at()).sample()
        is None
    )


def test_pose_sampler_converts_ned_to_enu_and_stamps_frame():
    sampler = _enu_sampler()
    sampler.update(_ned_pose(2.0, 1.0, -3.0))  # NED -> ENU (1, 2, 3)
    sample = _require_sample(sampler)
    assert sample.frame_id == "patrol_world"
    assert sample.position == pytest.approx((1.0, 2.0, 3.0))
    # heading 0 (NED, facing North) -> ENU yaw pi/2 (facing North in ENU) -> quaternion about +Up.
    assert sample.orientation == pytest.approx(
        (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    )


def test_pose_sampler_orientation_tracks_ned_heading():
    # A non-trivial NED heading must produce the corresponding ENU yaw quaternion (honest ENU
    # orientation, not the raw NED value mislabeled ENU). NED yaw pi/2 (facing East) -> ENU yaw 0.
    sampler = _enu_sampler()
    sampler.update(_ned_pose(0.0, 0.0, 0.0, heading=math.pi / 2))
    # ENU yaw 0 -> identity quaternion (facing East in ENU).
    assert _require_sample(sampler).orientation == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_pose_sampler_applies_ekf_origin_offset():
    sampler = _enu_sampler(ekf_origin_ned=(10.0, 20.0, 30.0))
    sampler.update(_ned_pose(12.0, 21.0, 27.0))  # rel NED (2,1,-3) -> ENU (1,2,3)
    assert _require_sample(sampler).position == pytest.approx((1.0, 2.0, 3.0))


def test_pose_sampler_surfaces_receipt_timestamp():
    # sample() returns (PoseSample, received_at) so the coordinator can enforce max_pose_age_s.
    sampler = _enu_sampler(clock=_clock_at((11, 3)))
    sampler.update(_ned_pose(0.0, 0.0, 0.0))
    result = sampler.sample()
    assert result is not None
    assert result[1] == (11, 3)


def _fake_encoder(image_msg):
    """Stand-in for the cv_bridge/OpenCV encode seam: deterministic, ROS-free."""
    return f"encoded:{image_msg}".encode()
