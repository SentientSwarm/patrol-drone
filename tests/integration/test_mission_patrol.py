"""SITL integration test for the multi-waypoint patrol (AC-2, AC-6, MC-10).

Spins the mission node via ``mission_patrol.launch.py`` against a **real** PX4 SITL drone (Gazebo
Harmonic) over the uXRCE-DDS bridge, and asserts the observable patrol on ``/patrol/*``:

  * Nominal (AC-2): arm -> takeoff -> visit every configured waypoint (with dwell) -> RTH -> land.
    Run with ``record:=true`` passed explicitly (the launch default is ``false`` until 05 lands):
    since 05 (patrol_logging) is absent in CI, this also exercises the launch's resilient-include
    skip (TS-I3) — the patrol must still come up.
  * External abort (AC-6): an abort published on ``/patrol/abort`` mid-patrol drives an observable
    ABORT -> RTH (return home) then disarm. Run with ``record:=false`` for determinism.

The simulator is never mocked (tests/README): real SITL + the agent are brought up by the nightly
job before pytest runs; this test launches only the mission node and observes ``/patrol/*`` +
``/fmu/out/vehicle_status``. The PASS/FAIL definition is shared verbatim with the host verifier via
:mod:`patrol_acceptance` so the two can't drift.

Nightly SITL tier only — never a required per-PR check (OQ-5). Marked ``ros`` so the Layer-A unit
runner (no ROS) skips it.
"""

from pathlib import Path

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from patrol_acceptance import (
    PatrolWatcher,
    evaluate_nominal,
    expected_waypoint_count,
    spin_until,
    wait_for_subscription,
)
from patrol_mission.qos import patrol_abort_qos
from std_msgs.msg import Bool

from patrol_mission import topics

pytestmark = pytest.mark.ros

# How long to wait for the patrol to get underway before injecting the abort (arm + climb + reach
# the first leg). Well under the patrol timeout; the abort then drives the (shorter) return home.
_UNDERWAY_TIMEOUT_S = 150.0

# Absolute path to the interim checkpoints file, computed from this test's location so it resolves
# regardless of the launched node's working directory (parents[2] is the repo root on the host and
# /opt in the nightly container, where `docker cp sim /opt/sim` places it). Passed explicitly to the
# launch so the patrol's checkpoint_id waypoints resolve without depending on CWD.
_CHECKPOINTS_YAML = str(Path(__file__).resolve().parents[2] / "sim" / "config" / "checkpoints.yaml")


def _patrol_launch(record: str) -> LaunchDescription:
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
                    "checkpoints_yaml": _CHECKPOINTS_YAML,
                }.items(),
            ),
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@launch_pytest.fixture
def patrol_launch() -> LaunchDescription:
    # Pass record:=true explicitly (launch default is false); 05 absent in CI -> resilient skip
    # (TS-I3), patrol still comes up.
    return _patrol_launch("true")


@launch_pytest.fixture
def patrol_launch_no_record() -> LaunchDescription:
    return _patrol_launch("false")


@pytest.mark.launch(fixture=patrol_launch)
def test_patrol_visits_all_waypoints_then_returns_home() -> None:
    rclpy.init()
    expected = expected_waypoint_count()
    watcher = PatrolWatcher(expected)
    try:
        spin_until(watcher, lambda w: w.nominal_complete)
        for check in evaluate_nominal(watcher, expected):
            assert check.passed, f"{check.name}: {check.detail}"
    finally:
        watcher.destroy_node()
        rclpy.shutdown()


@pytest.mark.launch(fixture=patrol_launch_no_record)
def test_external_abort_mid_patrol_drives_observable_rth() -> None:
    rclpy.init()
    watcher = PatrolWatcher(expected_waypoint_count())
    injector = rclpy.create_node("abort_injector")
    abort_pub = injector.create_publisher(Bool, topics.PATROL_ABORT, patrol_abort_qos())
    try:
        # Wait until the patrol is underway (armed + past takeoff / reached a waypoint)...
        spin_until(
            watcher,
            lambda w: w.was_armed and ("HOVER" in w.states_seen or bool(w.waypoints_visited)),
            timeout_s=_UNDERWAY_TIMEOUT_S,
        )
        assert watcher.was_armed, "patrol never armed — cannot exercise the mid-patrol abort"

        # ...then raise the external abort. patrol_abort_qos is reliable + *volatile*, so a sample
        # published before the node's subscriber is discovered would be dropped on the floor. Wait
        # for DDS matching first (Hermes Medium) rather than assume the patrol being underway implies
        # it. Once delivered, the abort "sticks" through RTH via the state machine's latch.
        assert wait_for_subscription(injector, abort_pub), (
            "node's /patrol/abort subscriber was not discovered; the volatile abort would be dropped"
        )
        msg = Bool()
        msg.data = True
        abort_pub.publish(msg)

        # The mission must transition to an observable ABORT, then RTH, settle at home, then disarm.
        spin_until(
            watcher,
            lambda w: w.abort_then_rth and w.settled_near_home and w.disarmed_after_arm,
        )
        assert watcher.abort_then_rth, (
            f"no observable ABORT->RTH; states seen: {watcher.states_seen}"
        )
        assert watcher.settled_near_home, (
            f"abort-driven RTH did not settle within {watcher.home_tol_m} m of home_ned "
            f"{watcher.home_ned}; closest approach {watcher.min_home_distance_m:.2f} m"
        )
        assert watcher.disarmed_after_arm, (
            "vehicle did not disarm after the abort-driven return home"
        )
    finally:
        injector.destroy_node()
        watcher.destroy_node()
        rclpy.shutdown()
