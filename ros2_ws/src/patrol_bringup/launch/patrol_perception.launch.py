"""Bring up the perception capture chain: camera_info bridge -> apriltag detector -> capture node.

M6.B (T B.4, design §4.2.9). This wires the live detect-publish path on top of M5's camera bridge:

    /drone/camera/camera_info  (gz -> ROS via ros_gz_bridge; apriltag needs intrinsics)
    apriltag_node              (image_rect + camera_info -> /tag_detections, AprilTagDetectionArray)
    perception_node            (camera + /tag_detections + /patrol/dwell + pose -> CheckpointCapture)

This launch is wiring only — it starts no simulator and does not bridge the raw image (that is
camera_bridge.launch.py, M5). Run alongside the patrol stack; the end-to-end traversal (one capture
per checkpoint) is the nightly/manual SITL integration test (AC-2/AC-4/AC-6), not this launch.

    ros2 launch patrol_bringup patrol_perception.launch.py \
        checkpoint_config_path:=/abs/path/to/sim/config/checkpoints.yaml
    ros2 launch patrol_bringup patrol_perception.launch.py \
        checkpoint_config_path:=/abs/path/to/checkpoints.yaml camera_topic:=/drone/camera/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# M5's bridged RGB image (camera_bridge.launch.py); apriltag + perception both subscribe here.
CAMERA_IMAGE_TOPIC = "/drone/camera/image_raw"
# gz publishes camera_info as a sibling of the image topic; ros_gz_bridge maps it 1:1 into ROS.
CAMERA_INFO_TOPIC = "/drone/camera/camera_info"
# apriltag_node detection output; perception_node binds its detections_topic to this.
DETECTIONS_TOPIC = "/tag_detections"


def _camera_info_bridge() -> Node:
    """Bridge the gz camera_info (apriltag needs intrinsics; ros_gz_image bridges only the image)."""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="camera_info_bridge",
        output="screen",
        arguments=[f"{CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        # All camera-pipeline nodes run on the Gazebo sim clock so the bridged image and camera_info
        # share one time base; apriltag's exact-time message_filters sync needs matching stamps
        # (without this it pairs 0 of N and emits no detections — the M6 capture chain stalls).
        parameters=[{"use_sim_time": True}],
    )


def _apriltag_node() -> Node:
    """Off-the-shelf tag36h11 detector (VP-1). apriltag_ros emits the bare family token it is given
    here ('36h11') in AprilTagDetection.family; the checkpoint config's tag_family is the conventional
    prefixed form ('tag36h11'). CheckpointResolver normalizes the 'tag' prefix so the two compare
    equal (_families_match) — see checkpoint_resolver.py."""
    return Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        remappings=[
            ("image_rect", CAMERA_IMAGE_TOPIC),
            ("camera_info", CAMERA_INFO_TOPIC),
            ("detections", DETECTIONS_TOPIC),
        ],
        parameters=[{"family": "36h11", "max_hamming": 0, "use_sim_time": True}],
    )


def _perception_node() -> Node:
    return Node(
        package="patrol_perception",
        executable="perception_node",
        name="patrol_perception",
        output="screen",
        parameters=[
            {
                "camera_topic": LaunchConfiguration("camera_topic"),
                "trigger_topic": LaunchConfiguration("trigger_topic"),
                "detections_topic": DETECTIONS_TOPIC,
                "checkpoint_config_path": LaunchConfiguration("checkpoint_config_path"),
                "world_frame": LaunchConfiguration("world_frame"),
                "output_root": LaunchConfiguration("output_root"),
                # run_id: empty -> the node mints its own UTC token; mission_patrol forwards the same
                # id it gives 05's recorder so captures and the bag correlate (F-01 / OQ-4).
                "run_id": LaunchConfiguration("run_id"),
                "max_detection_age_s": LaunchConfiguration("max_detection_age_s"),
                "max_frame_age_s": LaunchConfiguration("max_frame_age_s"),
                "max_pose_age_s": LaunchConfiguration("max_pose_age_s"),
                # Sim clock: the perception node's ADR-B freshness gate compares message stamps to
                # "now", so it must read the same Gazebo clock the bridged camera/detections carry.
                "use_sim_time": True,
            }
        ],
    )


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_topic", default_value=CAMERA_IMAGE_TOPIC),
            DeclareLaunchArgument("trigger_topic", default_value="/patrol/dwell"),
            DeclareLaunchArgument(
                "checkpoint_config_path",
                default_value="",
                description="absolute path to 03's checkpoints.yaml (the installed package cannot "
                "resolve a CWD-relative path); required for standalone launch, forwarded by "
                "mission_patrol.launch.py",
            ),
            DeclareLaunchArgument("world_frame", default_value="patrol_world"),
            # output_root: where captures land (<output_root>/<run_id>/). Empty -> the node's CWD
            # "captures" default; mission_patrol forwards 05's bag/run dir here so 04↔05 align (OQ-4).
            DeclareLaunchArgument("output_root", default_value=""),
            # run_id: the shared correlation id (<output_root>/<run_id>/ + the bag's mission-id
            # segment). Empty -> the node mints its own; mission_patrol forwards one shared id (F-01).
            DeclareLaunchArgument("run_id", default_value=""),
            # ADR-B freshness windows (seconds): defaults mirror perception_node's, set above the
            # slowest inter-message gap per stream so only a genuine stall trips the gate (§4.2.9).
            DeclareLaunchArgument("max_detection_age_s", default_value="1.0"),
            DeclareLaunchArgument("max_frame_age_s", default_value="0.5"),
            DeclareLaunchArgument("max_pose_age_s", default_value="1.0"),
            _camera_info_bridge(),
            _apriltag_node(),
            _perception_node(),
        ]
    )
