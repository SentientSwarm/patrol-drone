"""Launch the basic mission: arm, takeoff to 5 m, hover 10 s, land (MC-1, AC-1).

Wiring only — starts PatrolMissionNode with the checked-in mission_basic.yaml.
No logic lives here (design §4.2.7).

    ros2 launch patrol_bringup mission_basic.launch.py
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("patrol_bringup"), "config", "mission_basic.yaml"]
    )
    return LaunchDescription(
        [
            Node(
                package="patrol_mission",
                executable="patrol_mission",
                name="patrol_mission",
                output="screen",
                parameters=[{"mission_yaml": mission_yaml}],
            ),
        ]
    )
