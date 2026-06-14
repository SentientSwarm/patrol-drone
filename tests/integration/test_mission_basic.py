"""SITL integration test for the basic mission (AC-1, AC-5, MC-10).

Spins the mission node via ``mission_basic.launch.py`` against a **real** PX4 SITL drone (Gazebo
Harmonic) reachable over the uXRCE-DDS bridge, and asserts the observable progression of the basic
mission:

    arm  ->  climb to ~5 m AGL  ->  hover  ->  land + disarm

The simulator is never mocked (tests/README): if a test needs flight dynamics it uses real SITL.
PX4 SITL + the Micro XRCE-DDS Agent are brought up by the nightly job *before* pytest runs; this
test launches only the mission node and observes ``/fmu/out/*``.

The PASS/FAIL definition itself lives in :mod:`mission_acceptance`, shared verbatim with the host
verifier (``scripts/verify_mission.py``) so the two can't drift — this test only wires the launch
and turns each shared :class:`~mission_acceptance.Check` into an assertion.

Runs in the nightly SITL tier only — never a required per-PR check (OQ-5). Marked ``ros`` so the
Layer-A unit runner (which has no ROS) skips it.
"""

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from mission_acceptance import (
    MissionAcceptanceWatcher,
    evaluate,
    load_thresholds,
    spin_until_complete,
)

pytestmark = pytest.mark.ros


@launch_pytest.fixture
def mission_launch() -> LaunchDescription:
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("patrol_bringup"), "launch", "mission_basic.launch.py"]
                    )
                )
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=mission_launch)
def test_basic_mission_arms_climbs_and_lands() -> None:
    rclpy.init()
    thresholds = load_thresholds()
    watcher = MissionAcceptanceWatcher(thresholds)
    try:
        spin_until_complete(watcher, thresholds)
        for check in evaluate(watcher, thresholds):
            assert check.passed, f"{check.name}: {check.detail}"
    finally:
        watcher.destroy_node()
        rclpy.shutdown()
