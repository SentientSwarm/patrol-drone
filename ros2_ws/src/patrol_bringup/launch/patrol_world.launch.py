"""Run the M4 patrol over the M5 camera-equipped patrol world (ROS side; M5 T3.1, SIM-5 glue).

Wires the stage together so the same ``sim/config/checkpoints.yaml`` drives BOTH the world's AprilTag
marker placement (via sim/tools/compose_world.py) AND the patrol waypoints (02's
``mission_patrol.launch.py`` resolves ``checkpoint_id`` against it) — one source of truth (Tenet 2).
It starts the camera image bridge (camera_bridge.launch.py) and 02's patrol node.

This launches the ROS side only. PX4 SITL + Gazebo (loading patrol_world.sdf and spawning
gz_x500_patrol) are brought up separately — PX4 is not a ROS node; see sim/README.md "Running the
patrol world" for the env (GZ_SIM_RESOURCE_PATH, PX4_GZ_WORLD, PX4_GZ_MODEL_NAME). The full
end-to-end SITL traversal (AC-3/SIM-5) is exercised in the nightly tier.

``checkpoints_yaml`` is required and must be absolute — the checkpoints file is 03-owned with no
CWD-relative default (OQ-2; same contract as mission_patrol.launch.py). Point it at the repo's
sim/config/checkpoints.yaml.

    ros2 launch patrol_bringup patrol_world.launch.py checkpoints_yaml:=/abs/sim/config/checkpoints.yaml
    ros2 launch patrol_bringup patrol_world.launch.py checkpoints_yaml:=... record:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(
    launch_file: str, arguments: dict[str, LaunchConfiguration]
) -> IncludeLaunchDescription:
    source = PathJoinSubstitution([FindPackageShare("patrol_bringup"), "launch", launch_file])
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(source),
        launch_arguments=list(arguments.items()),
    )


def generate_launch_description() -> LaunchDescription:
    checkpoints_yaml = LaunchConfiguration("checkpoints_yaml")
    record = LaunchConfiguration("record")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "checkpoints_yaml",
                description="absolute path to 03's sim/config/checkpoints.yaml (OQ-2; required, no "
                "CWD-relative default) — drives both the world markers and the patrol waypoints",
            ),
            DeclareLaunchArgument(
                "record",
                default_value="false",
                description="forwarded to mission_patrol: include 05's recorder if installed",
            ),
            _include("camera_bridge.launch.py", {}),
            _include(
                "mission_patrol.launch.py",
                {"checkpoints_yaml": checkpoints_yaml, "record": record},
            ),
        ]
    )
