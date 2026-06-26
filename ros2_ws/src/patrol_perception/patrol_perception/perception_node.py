"""PerceptionNode — the rclpy entrypoint that wires the capture pipeline (M6.B, T B.4 — §4.2.9).

This is the only module in the package that imports rclpy. It declares the six bind-at-config
parameters (§4.2.9), constructs the subscriptions (camera, trigger, pose, /tag_detections), builds
the ROS-free core + samplers + coordinator, and owns the CapturePublisher on
``/patrol/checkpoint_capture``. All domain logic lives in the (unit-tested, ROS-free) collaborators;
this file is intentionally thin plumbing, verified by ``colcon build`` (AC-7) and the downstream
SITL integration test (AC-2/AC-4/AC-6), not by the Layer-A unit suite.

QoS (§4.4): publisher is RELIABLE depth-1 so 05's recorder reliably captures each low-rate event;
camera + detections subscriptions use sensor-data (best-effort) to match the streaming sources;
pose uses the PX4 bridge profile (best-effort + transient-local).
"""

from __future__ import annotations

from datetime import UTC, datetime

import cv2
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from cv_bridge import CvBridge
from diagnostic_msgs.msg import KeyValue
from geometry_msgs.msg import Point as PointMsg
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from patrol_mission.qos import patrol_event_qos
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header, Int32

from patrol_interfaces.msg import CheckpointCapture
from patrol_perception.capture_builder import CaptureRecord, CheckpointCaptureBuilder
from patrol_perception.capture_writer import CaptureWriter
from patrol_perception.checkpoint_config import CheckpointConfigLoader
from patrol_perception.checkpoint_resolver import CheckpointResolver
from patrol_perception.coordinator import (
    CaptureCoordinator,
    CapturePipeline,
    FreshnessWindows,
)
from patrol_perception.samplers import FrameSampler, LatestBuffer, PoseSampler

# PX4 VehicleLocalPosition is already expressed relative to the EKF origin (the mission node treats
# it the same way), so no additional origin offset is applied in this SITL setup.
_EKF_ORIGIN_NED = (0.0, 0.0, 0.0)


def _capture_publisher_qos() -> QoSProfile:
    """Publisher-only QoS for /patrol/checkpoint_capture: reliable, depth-1 so 05's recorder
    reliably gets every low-rate capture event (§4.4). NOT for the /patrol/dwell trigger subscriber
    — that uses the route-covering patrol_event_qos() so rapid events aren't coalesced (F-01)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _px4_pose_qos() -> QoSProfile:
    """The /fmu/out QoS the PX4 uXRCE-DDS bridge uses (best-effort + transient-local, depth 1)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class _RosCaptureMessageFactory:
    """The rosidl-backed factory the CheckpointCaptureBuilder duck-types against (M6.A seam)."""

    def new_capture(self) -> CheckpointCapture:
        return CheckpointCapture()

    def make_header(self, sec: int, nanosec: int, frame_id: str) -> Header:
        header = Header()
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        header.frame_id = frame_id
        return header

    def make_pose_stamped(self, rec: CaptureRecord) -> PoseStamped:
        ps = PoseStamped()
        ps.header = self.make_header(rec.stamp_sec, rec.stamp_nanosec, rec.frame_id)
        px, py, pz = rec.position
        ox, oy, oz, ow = rec.orientation
        ps.pose = Pose(
            position=PointMsg(x=px, y=py, z=pz),
            orientation=Quaternion(x=ox, y=oy, z=oz, w=ow),
        )
        return ps

    def make_key_value(self, key: str, value: str):

        return KeyValue(key=key, value=value)


class _CapturePublisher:
    """Owns the /patrol/checkpoint_capture publisher (the only ROS-out of the pipeline, PCAP-3)."""

    def __init__(self, node: Node) -> None:
        self._pub = node.create_publisher(
            CheckpointCapture, "/patrol/checkpoint_capture", _capture_publisher_qos()
        )

    def publish(self, msg: CheckpointCapture) -> None:
        self._pub.publish(msg)


class PerceptionNode(Node):
    """Wires params -> subscriptions -> coordinator; hosts no domain logic itself."""

    def __init__(self) -> None:
        super().__init__("patrol_perception")
        camera_topic = self._required_param("camera_topic")
        trigger_topic = str(self.declare_parameter("trigger_topic", "/patrol/dwell").value)
        detections_topic = str(self.declare_parameter("detections_topic", "/tag_detections").value)
        config_path = str(
            self.declare_parameter("checkpoint_config_path", "sim/config/checkpoints.yaml").value
        )
        world_frame = str(self.declare_parameter("world_frame", "patrol_world").value)
        # output_root defaults under a CWD-relative "captures" dir (05 may override to align bags);
        # run_id is a UTC timestamp so each patrol writes to its own <output_root>/<run_id>/ (§4.2.6).
        # A launch-provided run_id (mission_patrol forwards the same id to 05's recorder) wins so
        # captures and the bag share one identity (F-01 / OQ-4); empty -> this node mints its own.
        output_root = str(self.declare_parameter("output_root", "").value) or "captures"
        run_id = str(self.declare_parameter("run_id", "").value) or datetime.now(tz=UTC).strftime(
            "%Y%m%dT%H%M%SZ"
        )

        # ADR-B freshness windows (seconds): a buffered detection/frame/pose older than its window is
        # treated as stale and skipped like an absent buffer (§4.4.5). Defaults are set above the
        # slowest expected inter-message gap so only a genuine stall trips the gate (15 Hz camera ->
        # ~0.067 s/frame; detector/pose are slower), keeping the happy-path traversal unaffected.
        max_detection_age_s = float(self.declare_parameter("max_detection_age_s", 1.0).value)
        max_frame_age_s = float(self.declare_parameter("max_frame_age_s", 0.5).value)
        max_pose_age_s = float(self.declare_parameter("max_pose_age_s", 1.0).value)

        self._bridge = CvBridge()
        self._frame_sampler = FrameSampler(encoder=self._encode_image, clock=self._now)
        self._pose_sampler = PoseSampler(
            world_frame=world_frame, ekf_origin_ned=_EKF_ORIGIN_NED, clock=self._now
        )
        self._detection_buffer: LatestBuffer = LatestBuffer(self._now)

        entries = CheckpointConfigLoader().load(config_path)
        pipeline = CapturePipeline(
            frame_sampler=self._frame_sampler,
            pose_sampler=self._pose_sampler,
            detection_buffer=self._detection_buffer,
            resolver=CheckpointResolver(entries),
            builder=CheckpointCaptureBuilder(_RosCaptureMessageFactory()),
            publisher=_CapturePublisher(self),
            writer=CaptureWriter(output_root=output_root, run_id=run_id),
        )
        self._coordinator = CaptureCoordinator(
            pipeline=pipeline,
            clock=self._now,
            freshness=FreshnessWindows(
                detection_s=max_detection_age_s,
                frame_s=max_frame_age_s,
                pose_s=max_pose_age_s,
            ),
            # mission_id == run_id == <run dir name> (the UTC timestamp from L130): the settled OQ-4
            # alignment. 05 correlates captures<->bag by this run id; the "mission_id" capture-metadata
            # key is intentionally this run timestamp, not a separate mission identifier.
            mission_id=run_id,
        )

        self.create_subscription(Image, camera_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray, detections_topic, self._on_detections, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._pose_sampler.update,
            _px4_pose_qos(),
        )
        self.create_subscription(Int32, trigger_topic, self._on_trigger, patrol_event_qos())
        self.get_logger().info(
            f"patrol_perception up: camera={camera_topic} trigger={trigger_topic} "
            f"detections={detections_topic} checkpoints={len(entries)} world_frame={world_frame}"
        )

    def _required_param(self, name: str) -> str:
        value = str(self.declare_parameter(name, "").value)
        if not value:
            raise ValueError(f"required node parameter '{name}' is unset")
        return value

    def _encode_image(self, image_msg: Image) -> bytes:
        """cv_bridge/OpenCV encode seam: sensor_msgs/Image -> PNG bytes (VP-2)."""

        frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".png", frame)
        if not ok:
            raise RuntimeError("cv2.imencode failed to encode the camera frame to PNG")
        encoded: bytes = buf.tobytes()
        return encoded

    def _now(self) -> tuple[int, int]:
        now = self.get_clock().now().to_msg()
        return now.sec, now.nanosec

    def _on_image(self, msg: Image) -> None:
        self._frame_sampler.update(msg)

    def _on_detections(self, msg: AprilTagDetectionArray) -> None:
        self._detection_buffer.update(msg.detections)

    def _on_trigger(self, msg: Int32) -> None:
        self._coordinator.on_trigger(visit_token=msg.data)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
