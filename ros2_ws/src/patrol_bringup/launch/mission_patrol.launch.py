"""Launch the patrol mission: arm, takeoff, patrol 4+ waypoints with dwell, RTH, land (MC-2, AC-2).

Wiring only (design §4.2.7) — starts PatrolMissionNode with the checked-in patrol_mission.yaml and
optionally includes 05's recorder.

``checkpoints_yaml`` has no in-package default: the checkpoints file is 03's deliverable (OQ-2), so
its path is supplied explicitly rather than via a CWD-relative default that would resolve
differently depending on where the launch ran from (Hermes Medium). The shipped patrol_mission.yaml
uses checkpoint_id waypoints, so an absolute ``checkpoints_yaml:=`` is required (the UAT runner
``scripts/run_sitl_mission.sh --patrol`` passes it for you).

The recorder include is **resilient**: 05 (``patrol_logging``) is a later docset and may be absent.
``record`` defaults to ``false`` until 05 lands in-tree, so the launch never auto-includes a package
that merely happens to be named ``patrol_logging`` on the path (Hermes Medium). Pass ``record:=true``
to attach it once 05 is installed — when present it auto-attaches, when absent the include is skipped
with a warning rather than failing the launch (design §4.4.5: "recorder absent -> mission flies").

    ros2 launch patrol_bringup mission_patrol.launch.py checkpoints_yaml:=/abs/path/checkpoints.yaml
    ros2 launch patrol_bringup mission_patrol.launch.py checkpoints_yaml:=... record:=true
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
_PERCEPTION_PKG = "patrol_perception"  # 04-perception; the capture node + apriltag deps (VP-1)


def _maybe_perception(context: LaunchContext) -> list[IncludeLaunchDescription]:
    """Include 04's perception chain (patrol_perception.launch.py) iff ``perception:=true`` AND
    ``patrol_perception`` is installed. Resolved at launch time so an environment without the
    apriltag/perception deps skips with a warning rather than grounding the patrol (design §4.4.5).
    """
    if LaunchConfiguration("perception").perform(context) != "true":
        return []
    try:
        get_package_share_directory(_PERCEPTION_PKG)
    except PackageNotFoundError:
        get_logger("mission_patrol").warning(
            f"{_PERCEPTION_PKG} (04 perception) not found — flying the patrol without capture."
        )
        return []
    perception_launch = PathJoinSubstitution(
        [FindPackageShare("patrol_bringup"), "launch", "patrol_perception.launch.py"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch),
            launch_arguments={
                "checkpoint_config_path": LaunchConfiguration("checkpoints_yaml"),
                # forward the capture output root so 04's artifacts co-locate with 05's run/bag dir
                # when set (OQ-4 alignment); empty falls back to the perception node's CWD default.
                "output_root": LaunchConfiguration("output_root"),
            }.items(),
        )
    ]


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
                default_value="false",
                description="include 05's recorder (patrol_logging) if installed; defaults false "
                "until 05 lands in-tree, pass record:=true to attach it",
            ),
            DeclareLaunchArgument(
                "checkpoints_yaml",
                default_value="",
                description="absolute path to 03's checkpoint-positions YAML (OQ-2); required when "
                "the mission uses checkpoint_id waypoints (no CWD-relative default)",
            ),
            DeclareLaunchArgument(
                "perception",
                default_value="true",
                description="include 04's perception capture chain (patrol_perception.launch.py) "
                "if installed; skipped with a warning when the package/apriltag deps are absent",
            ),
            DeclareLaunchArgument(
                "output_root",
                default_value="",
                description="root dir for 04's on-disk captures (<output_root>/<run_id>/); set it to "
                "05's bag/run dir to align artifacts (OQ-4), empty -> the node's CWD 'captures' default",
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
            OpaqueFunction(function=_maybe_perception),
            OpaqueFunction(function=_maybe_record),
        ]
    )
