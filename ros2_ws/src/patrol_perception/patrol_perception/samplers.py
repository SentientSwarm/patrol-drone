"""Latest-frame camera + ground-truth pose sampling (M6.B, T B.2 — design §4.2.7, ADR-B).

The samplers own rclpy subscriptions in the live PerceptionNode, but the logic here is ROS-free
and unit-tested (AC-5): keep-latest buffering, cv_bridge-seam encoding, and NED->ENU pose stamping.
Per ADR-B (Phase 1 uses ground-truth sim pose, quasi-static hover) both samplers return their
most-recent buffered value on trigger — no message_filters time-sync.

NED->ENU goes through the SINGLE MC-7 conversion site (``patrol_mission.frames.to_enu_from_ned``);
this module adds no second conversion site (Tenet 4). The encode step is an injected callable so
the cv_bridge/OpenCV dependency stays at the node boundary and out of the unit path.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from patrol_mission.frames import (
    Point,
    Quaternion,
    enu_quaternion_from_ned_heading,
    to_enu_from_ned,
)


class LatestBuffer[T]:
    """Holds only the most-recent pushed value; ``latest()`` is ``None`` until the first update.

    Shared by both samplers so the keep-latest semantics live in one place (ADR-B latest-frame),
    rather than being re-implemented per sampler.
    """

    def __init__(self) -> None:
        self._value: T | None = None

    def update(self, value: T) -> None:
        self._value = value

    def latest(self) -> T | None:
        return self._value


class FrameSampler:
    """Buffers the latest ``sensor_msgs/Image`` and encodes it on demand (PCAP-1).

    ``encoder`` is the cv_bridge/OpenCV seam (Image -> PNG/JPEG bytes); injected so the unit path
    needs neither. The live node passes a real cv_bridge-backed encoder.
    """

    def __init__(self, encoder: Callable[[Any], bytes]) -> None:
        self._encoder = encoder
        self._buffer: LatestBuffer[Any] = LatestBuffer()

    def update(self, image_msg: Any) -> None:
        self._buffer.update(image_msg)

    def take_latest(self) -> tuple[Any, bytes] | None:
        """Return ``(image_msg, encoded_bytes)`` for the most recent frame, or ``None`` if no
        frame has arrived yet (the coordinator treats ``None`` as a skip, leaving the visit
        retryable per §4.2.8)."""
        image_msg = self._buffer.latest()
        if image_msg is None:
            return None
        return image_msg, self._encoder(image_msg)


@dataclass(frozen=True)
class PoseSample:
    """A capture pose in world/ENU with an explicit frame_id (ADR-B, Tenet 5).

    Both position AND orientation are world/ENU: the orientation is a yaw-only ENU quaternion derived
    from the PX4 NED ``heading`` at the single MC-7 site, so a downstream consumer that trusts
    ``frame_id`` reads the whole pose in one frame (no mixed NED/ENU pose, design §4.2.4).
    """

    position: Point  # x, y, z in world/ENU meters
    orientation: Quaternion  # x, y, z, w in world/ENU (yaw-only, from NED heading)
    frame_id: str


class PoseSampler:
    """Buffers the latest PX4 ``VehicleLocalPosition`` (NED) and returns it in world/ENU (PCAP-1).

    On ``sample()`` the buffered NED position is converted to world/ENU through the single MC-7 site,
    and the NED ``heading`` (the only attitude PX4 ``VehicleLocalPosition`` carries — there is no
    quaternion field) is converted to a yaw-only world/ENU quaternion at the same MC-7 boundary
    (:func:`~patrol_mission.frames.enu_quaternion_from_ned_heading`). Roll/pitch are dropped: a
    checkpoint visit is a quasi-static hover, so an honest yaw-only ENU orientation is sufficient and
    a full NED/FRD->ENU/FLU attitude transform is deferred to Phase 3+ with VIO (ADR-B), rather than
    fabricating precision the ground-truth-hover path does not require. The whole pose is stamped with
    ``world_frame`` and is genuinely single-frame (no orientation mislabeled ENU).
    """

    def __init__(self, world_frame: str, ekf_origin_ned: Point) -> None:
        self._world_frame = world_frame
        self._ekf_origin_ned = ekf_origin_ned
        self._buffer: LatestBuffer[Any] = LatestBuffer()

    def update(self, pose_ned: Any) -> None:
        self._buffer.update(pose_ned)

    def sample(self) -> PoseSample | None:
        """Return the latest pose in world/ENU, or ``None`` if no pose has arrived yet."""
        pose_ned = self._buffer.latest()
        if pose_ned is None:
            return None
        position = to_enu_from_ned((pose_ned.x, pose_ned.y, pose_ned.z), self._ekf_origin_ned)
        orientation = enu_quaternion_from_ned_heading(pose_ned.heading)
        return PoseSample(
            position=position,
            orientation=orientation,
            frame_id=self._world_frame,
        )
