"""Shared patrol launch description for the SITL integration scenarios (test consolidation, SWM-33).

Both nightly patrol scenarios — the nominal/external-abort suite (``test_mission_patrol.py``) and
the low-battery abort scenario (``test_mission_patrol_low_battery.py``) — bring the mission node up
the same way: ``mission_patrol.launch.py`` with an explicit ``record`` flag and the absolute
checkpoints path. Extracted here so the launch wiring lives in exactly one place and the two
scenario files can't drift (CodeScene: no duplicated launch block across the test files).

Isolated from :mod:`patrol_acceptance` on purpose: this module imports ``launch``/``launch_ros``,
which the host-side verifier (``scripts/verify_patrol.py``, which also imports patrol_acceptance)
does not need — keeping the launch dependency out of the shared acceptance module.
"""

from __future__ import annotations

from pathlib import Path

import launch_pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Absolute path to the interim checkpoints file, computed from this module's location so it resolves
# regardless of the launched node's working directory (parents[2] is the repo root on the host and
# /opt in the nightly container, where `docker cp sim /opt/sim` places it). Passed explicitly to the
# launch so the patrol's checkpoint_id waypoints resolve without depending on CWD.
CHECKPOINTS_YAML = str(Path(__file__).resolve().parents[2] / "sim" / "config" / "checkpoints.yaml")


def patrol_launch_description(record: str) -> LaunchDescription:
    """The ``mission_patrol.launch.py`` include + ReadyToTest, with ``record`` passed explicitly.

    ``record`` is a launch-argument string ("true"/"false"): the launch default is "false" until 05
    is present, so a scenario that wants the recorder passes "true" (which also exercises the
    resilient-include skip when 05 is absent in CI, TS-I3); a scenario that wants determinism passes
    "false".
    """
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("patrol_bringup"), "launch", "mission_patrol.launch.py"]
                    )
                ),
                launch_arguments={
                    "record": record,
                    "checkpoints_yaml": CHECKPOINTS_YAML,
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )
