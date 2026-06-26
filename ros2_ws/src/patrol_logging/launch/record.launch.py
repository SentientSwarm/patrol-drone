"""Includable recorder fragment — records one MCAP bag per mission run (docset 05, M7).

02's ``mission_patrol.launch.py`` includes this (``record:=true``); it does not run standalone in the
normal flow, but can be launched directly for a bare recording. It is the launch/process plumbing
around the ROS-free recorder core (``patrol_logging.recorder``): it parses ``recorded_topics.yaml``,
asks the core to build the ``ros2 bag record --storage mcap`` argv and the JSON sidecar, runs the
recorder as an ``ExecuteProcess`` (so the launch system SIGINT-finalizes the MCAP at shutdown — no
manual stop), and writes ``<bag>.meta.json`` once the recorder exits.

Exactly one bag, named ``patrol_<missionId>_<timestamp>.mcap``, lands in ``output_dir`` per run — no
operator command beyond the launch (LR-1). The timestamp is captured once here so the bag name and
the sidecar agree.

    ros2 launch patrol_logging record.launch.py
    ros2 launch patrol_logging record.launch.py mission_id:=alpha output_dir:=/data/patrol_bags
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from patrol_logging.recorder import (
    RecordingRun,
    bag_name,
    build_record_argv,
    build_sidecar,
    write_sidecar,
)

_PKG = "patrol_logging"
_DEFAULT_OUTPUT_DIR = str(Path.home() / "patrol_bags")


def _default_topics_yaml() -> str:
    return str(Path(get_package_share_directory(_PKG)) / "config" / "recorded_topics.yaml")


def _load_topic_selection(topics_yaml: str) -> tuple[list[str], list[str]]:
    """Parse recorded_topics.yaml -> (topics, regexes). Fail loud if it has neither."""
    data = yaml.safe_load(Path(topics_yaml).read_text()) or {}
    topics = list(data.get("topics") or [])
    regexes = list(data.get("regexes") or [])
    if not topics and not regexes:
        raise ValueError(
            f"{topics_yaml} selects no topics — a recording with nothing to record is a "
            "configuration error (expected non-empty 'topics' and/or 'regexes')"
        )
    return topics, regexes


def _launch_recorder(context: LaunchContext) -> list[ExecuteProcess | RegisterEventHandler]:
    mission_id = LaunchConfiguration("mission_id").perform(context)
    # An empty output_dir (e.g. a top-level launch forwarding an unset output_root) falls back to
    # the known default rather than resolving to CWD.
    output_dir_arg = LaunchConfiguration("output_dir").perform(context) or _DEFAULT_OUTPUT_DIR
    output_dir = Path(output_dir_arg).expanduser()
    mission_config_ref = LaunchConfiguration("mission_config_ref").perform(context)
    topics_yaml = LaunchConfiguration("recorded_topics").perform(context)

    output_dir.mkdir(parents=True, exist_ok=True)
    topics, regexes = _load_topic_selection(topics_yaml)

    started = datetime.now(UTC)
    basename = bag_name(mission_id, started)
    argv = build_record_argv(
        output_dir=output_dir, bag_basename=basename, topics=topics, regexes=regexes
    )
    run = RecordingRun(
        mission_id=mission_id,
        bag_filename=f"{basename}.mcap",
        started=started,
        mission_config_ref=mission_config_ref,
    )

    get_logger("patrol_record").info(f"recording bag {basename}.mcap into {output_dir}")
    recorder = ExecuteProcess(cmd=argv, name="patrol_bag_record", output="screen")

    def _write_sidecar_on_exit(_event: object, _context: LaunchContext) -> None:
        """OnProcessExit callback: finalize the JSON sidecar once the recorder has stopped."""
        sidecar = build_sidecar(run, datetime.now(UTC), topics + regexes)
        sidecar_path = output_dir / f"{basename}.mcap.meta.json"
        write_sidecar(sidecar_path, sidecar)
        get_logger("patrol_record").info(f"wrote bag sidecar {sidecar_path}")

    return [
        recorder,
        RegisterEventHandler(OnProcessExit(target_action=recorder, on_exit=_write_sidecar_on_exit)),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mission_id",
                default_value="patrol",
                description="mission identifier baked into the bag name "
                "patrol_<missionId>_<timestamp>.mcap and the sidecar (correlates bag -> mission)",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value=_DEFAULT_OUTPUT_DIR,
                description="known output directory the bag + <bag>.meta.json land in "
                "(created if absent); defaults to ~/patrol_bags",
            ),
            DeclareLaunchArgument(
                "mission_config_ref",
                default_value="",
                description="path/ref to the mission YAML that produced this run, recorded in the "
                "sidecar for correlation (LR-2)",
            ),
            DeclareLaunchArgument(
                "recorded_topics",
                default_value=_default_topics_yaml(),
                description="path to recorded_topics.yaml (the broad topic set; topics + regexes)",
            ),
            OpaqueFunction(function=_launch_recorder),
        ]
    )
