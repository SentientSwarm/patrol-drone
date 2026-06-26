"""Launch the patrol mission: arm, takeoff, patrol 4+ waypoints with dwell, RTH, land (MC-2, AC-2).

Wiring only (design §4.2.7) — starts PatrolMissionNode with the checked-in patrol_mission.yaml and
optionally includes 05's recorder.

``checkpoints_yaml`` has no in-package default: the checkpoints file is 03's deliverable (OQ-2), so
its path is supplied explicitly rather than via a CWD-relative default that would resolve
differently depending on where the launch ran from (Hermes Medium). The shipped patrol_mission.yaml
uses checkpoint_id waypoints, so an absolute ``checkpoints_yaml:=`` is required (the UAT runner
``scripts/run_sitl_mission.sh --patrol`` passes it for you).

The recorder include is **resilient**: now that 05 (``patrol_logging``) has landed, ``record``
defaults to ``true`` so every mission run produces one MCAP bag (the M7 discipline). The include is
still guarded — if ``patrol_logging`` is not built into the environment (or resolves to a different
install prefix than this package, F-04), the launch logs a warning and flies the patrol without
recording rather than failing (design §4.4.5: "recorder absent -> mission flies"). Pass
``record:=false`` to fly without recording.

A single ``run_id`` is minted once here and forwarded to BOTH 04's perception capture and 05's
recorder (as the bag's mission-id segment), so a checkpoint capture correlates to the bag that
recorded it (F-01 / OQ-4). Pass ``run_id:=<id>`` to set it explicitly; empty mints a UTC token.

    ros2 launch patrol_bringup mission_patrol.launch.py checkpoints_yaml:=/abs/path/checkpoints.yaml
    ros2 launch patrol_bringup mission_patrol.launch.py checkpoints_yaml:=... record:=false
"""

from datetime import UTC, datetime
from pathlib import Path

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
_BRINGUP_PKG = (
    "patrol_bringup"  # owns this launch; the trusted prefix the recorder must co-install in
)


def _resolve_run_id(context: LaunchContext) -> list:
    """Mint one shared run id and stash it on the context so both includes read the same value (F-01).

    Runs before ``_maybe_record``/``_maybe_perception`` for EVERY patrol launch (incl. record:=false),
    so it must NOT import ``patrol_logging`` — ``patrol_bringup`` doesn't depend on it and the recorder
    include is resilient to its absence (design §4.4.5); a module-level import would ground the patrol
    when 05 isn't built. An operator-supplied ``run_id:=`` passes through; an empty default mints a UTC
    token. The format mirrors perception's run-dir name AND ``patrol_logging.recorder._RUN_ID_FMT`` (the
    bag's mission-id segment) so the two correlate — keep the literals in sync if either changes.
    """
    configured = LaunchConfiguration("run_id").perform(context)
    context.launch_configurations["run_id"] = configured or datetime.now(UTC).strftime(
        "%Y%m%dT%H%M%SZ"
    )
    return []


def _same_install_prefix(pkg_a: str, pkg_b: str) -> bool:
    """True iff both packages resolve under the same install prefix (``<prefix>/share/<pkg>``) (F-04).

    With ``record:=true`` the default, ``_maybe_record`` auto-includes and *executes*
    ``patrol_logging``'s launch file resolved by package name; pinning it to the same prefix as the
    trusted ``patrol_bringup`` keeps a stray overlay package from shadowing it. ``share`` dir is
    ``<prefix>/share/<pkg>`` so the install prefix is two parents up.
    """
    prefix_a = Path(get_package_share_directory(pkg_a)).parents[1]
    prefix_b = Path(get_package_share_directory(pkg_b)).parents[1]
    return prefix_a == prefix_b


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
                # the shared run id (resolved in _resolve_run_id) — perception tags captures with it
                # and 05's recorder takes the same value as mission_id, so the two correlate (F-01).
                "run_id": LaunchConfiguration("run_id"),
                # forward the ADR-B freshness windows so a slower detector / noisier sim can retune
                # them from the top-level patrol launch (defaults preserved in patrol_perception).
                "max_detection_age_s": LaunchConfiguration("max_detection_age_s"),
                "max_frame_age_s": LaunchConfiguration("max_frame_age_s"),
                "max_pose_age_s": LaunchConfiguration("max_pose_age_s"),
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
    if not _same_install_prefix(_RECORDER_PKG, _BRINGUP_PKG):
        get_logger("mission_patrol").warning(
            f"{_RECORDER_PKG} resolves to a different install prefix than {_BRINGUP_PKG} — "
            "skipping the recorder include to avoid running an overlay's record.launch.py."
        )
        return []
    record_launch = PathJoinSubstitution(
        [FindPackageShare(_RECORDER_PKG), "launch", "record.launch.py"]
    )
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("patrol_bringup"), "config", "patrol_mission.yaml"]
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(record_launch),
            launch_arguments={
                # the shared run id (resolved in _resolve_run_id) becomes the bag's mission-id
                # segment, matching perception's run-dir name so captures correlate to the bag (F-01).
                "mission_id": LaunchConfiguration("run_id"),
                # correlate the bag with the mission that produced it (sidecar mission_config_ref)
                "mission_config_ref": mission_yaml,
                # co-locate the bag with 04's captures when output_root is set (OQ-4 alignment);
                # empty -> the recorder's ~/patrol_bags default.
                "output_dir": LaunchConfiguration("output_root"),
            }.items(),
        )
    ]


def _declare_arguments() -> list[DeclareLaunchArgument]:
    """The launch's CLI surface, split out so ``generate_launch_description`` stays a short assembler."""
    return [
        DeclareLaunchArgument(
            "record",
            default_value="true",
            description="include 05's recorder (patrol_logging) so every mission run produces "
            "one MCAP bag (M7 discipline); resilient — skips with a warning if patrol_logging "
            "is not built. Pass record:=false to fly without recording",
        ),
        DeclareLaunchArgument(
            "run_id",
            default_value="",
            description="shared correlation id forwarded to BOTH 04 (capture run dir) and 05 "
            "(bag mission-id segment) so captures correlate to their bag (F-01); empty -> one "
            "is minted here (UTC token) in _resolve_run_id",
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
        DeclareLaunchArgument(
            "max_detection_age_s",
            default_value="1.0",
            description="ADR-B freshness window for /tag_detections (s); forwarded to perception",
        ),
        DeclareLaunchArgument(
            "max_frame_age_s",
            default_value="0.5",
            description="ADR-B freshness window for the camera frame (s); forwarded to perception",
        ),
        DeclareLaunchArgument(
            "max_pose_age_s",
            default_value="1.0",
            description="ADR-B freshness window for the /fmu/out pose (s); forwarded to perception",
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    mission_yaml = PathJoinSubstitution(
        [FindPackageShare("patrol_bringup"), "config", "patrol_mission.yaml"]
    )
    mission_node = Node(
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
    )
    return LaunchDescription(
        [
            *_declare_arguments(),
            mission_node,
            # Resolve the one shared run id BEFORE the two includes so both read the same value.
            OpaqueFunction(function=_resolve_run_id),
            OpaqueFunction(function=_maybe_perception),
            OpaqueFunction(function=_maybe_record),
        ]
    )
