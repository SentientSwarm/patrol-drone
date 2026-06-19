"""Bridge the gz_x500_patrol RGB camera into ROS 2 image topics (M5, SIM-3 / AC-4).

The camera sensor in sim/px4_sitl_overrides/gz_x500_patrol/model.sdf publishes a ``gz.msgs.Image`` on
the Gazebo transport topic ``/drone/camera/image_raw``. ``ros_gz_image image_bridge`` republishes it
into ROS 2 on the SAME name as ``sensor_msgs/Image`` and — via ``image_transport`` — a companion
``sensor_msgs/CompressedImage`` on ``<topic>/compressed``. That gives the exact 04/05 contract
(design §4.4.4) from one bridge process:

    /drone/camera/image_raw              sensor_msgs/Image            (04-perception subscribes)
    /drone/camera/image_raw/compressed   sensor_msgs/CompressedImage  (05-logging records)

Wiring only — this starts no simulator. PX4 SITL + Gazebo (patrol_world + gz_x500_patrol) are brought
up separately (PX4 is not a ROS node); see sim/README.md "Running the patrol world". The compressed
companion requires ``image_transport_plugins`` (``compressed_image_transport``) in the environment.

    ros2 launch patrol_bringup camera_bridge.launch.py
    ros2 launch patrol_bringup camera_bridge.launch.py gz_image_topic:=/drone/camera/image_raw
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_IMAGE_TOPIC = (
    "/drone/camera/image_raw"  # gz transport topic == ROS base topic (04/05 contract)
)


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_image_topic",
                default_value=CAMERA_IMAGE_TOPIC,
                description="Gazebo camera image topic to bridge; the ROS image topic takes the same "
                "name, with a /compressed companion added by image_transport (the 04/05 contract).",
            ),
            Node(
                package="ros_gz_image",
                executable="image_bridge",
                name="camera_image_bridge",
                output="screen",
                arguments=[LaunchConfiguration("gz_image_topic")],
            ),
        ]
    )
