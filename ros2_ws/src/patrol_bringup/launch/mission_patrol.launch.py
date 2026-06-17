"""Launch the patrol mission: arm, takeoff, patrol 4+ waypoints with dwell, RTH, land (MC-2, AC-2).

Wiring only (design §4.2.7) — starts PatrolMissionNode with the checked-in patrol_mission.yaml and
optionally includes 05's recorder.

The recorder include is **resilient**: 05 (``patrol_logging``) is a later docset and may be absent.
When it is, the include is skipped with a warning rather than failing the launch (design §4.4.5:
"recorder absent -> mission flies, no bag, non-critical"). When 05 lands, ``record:=true`` (the
default) auto-attaches it; ``record:=false`` disables it explicitly even when present.

    ros2 launch patrol_bringup mission_patrol.launch.py
    ros2 launch patrol_bringup mission_patrol.launch.py record:=false
    ros2 launch patrol_bringup mission_patrol.launch.py checkpoints_yaml:=/abs/path/checkpoints.yaml
"""

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_RECORDER_PKG = "patrol_logging"  # 05-logging; owns launch/record.launch.py


def _maybe_record(context: LaunchContext) -> list[IncludeLaunchDescription]:
    """Include 05's recorder iff ``record:=true`` AND ``patrol_logging`` is installed.

    Resolved at launch time (not description-build time) so an absent 05 is a skip-with-warning,
    not a hard ``PackageNotFoundError`` that would ground the whole patrol launch before 05 lands.
    """
    if LaunchConfiguration("record").perform(context) != "true":
        return []
    try:
        get_package_share_directory(_RECORDER_PKG)
    except PackageNotFoundError:
        get_logger("mission_patrol").warning(
            f"{_RECORDER_PKG} (05 recorder) not found — flying the patrol without recording. "
            "Land 05 and re-run with record:=true to capture a bag."
        )
        return []
    record_launch = PathJoinSubstitution(
        [FindPackageShare(_RECORDER_PKG), "launch", "record.launch.py"]
    )
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(record_launch))]


def generate_launch_description() -> LaunchDescription:
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("patrol_bringup"), "config", "patrol_mission.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "record",
                default_value="true",
                description="include 05's recorder if the patrol_logging package is installed",
            ),
            DeclareLaunchArgument(
                "checkpoints_yaml",
                default_value="sim/config/checkpoints.yaml",
                description="path to 03's checkpoint-positions YAML (OQ-2)",
            ),
            Node(
                package="patrol_mission",
                executable="patrol_mission",
                name="patrol_mission",
                output="screen",
                parameters=[
                    {
                        "mission_yaml": mission_yaml,
                        "checkpoints_yaml": LaunchConfiguration("checkpoints_yaml"),
                    }
                ],
            ),
            OpaqueFunction(function=_maybe_record),
        ]
    )
