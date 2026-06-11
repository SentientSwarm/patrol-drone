"""SITL integration test for the basic mission (AC-1, AC-5, MC-10).

Spins the mission node via ``mission_basic.launch.py`` against a **real** PX4
SITL drone (Gazebo Harmonic) reachable over the uXRCE-DDS bridge, and asserts the
observable progression of the basic mission:

    arm  ->  climb to ~5 m AGL  ->  hover  ->  land + disarm

The simulator is never mocked (tests/README): if a test needs flight dynamics it
uses real SITL. PX4 SITL + the Micro XRCE-DDS Agent are brought up by the nightly
job *before* pytest runs; this test launches only the mission node and observes
``/fmu/out/*``. ``/patrol/*`` mission topics arrive in M4, so M1 asserts on the
PX4 telemetry surface directly.

Runs in the nightly SITL tier only — never a required per-PR check (OQ-5). Marked
``ros`` so the Layer-A unit runner (which has no ROS) skips it.
"""

import time

import launch_pytest
import pytest
import rclpy
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

pytestmark = pytest.mark.ros

TAKEOFF_ALT_M = 5.0
ALT_REACHED_NED_Z = -4.5  # within 0.5 m of the -5 m takeoff setpoint counts as "reached"
MISSION_TIMEOUT_S = 120.0


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


class _TelemetryWatcher(Node):
    """Records the arm/altitude/disarm milestones observed on /fmu/out/*."""

    def __init__(self) -> None:
        super().__init__("mission_basic_test_watcher")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.was_armed = False
        self.reached_altitude = False
        self.disarmed_after_arm = False
        # PX4 v1.17 advertises `_v1`-suffixed topic names (01-platform design §4.2.4).
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._on_pos, qos
        )

    def _on_status(self, msg: VehicleStatus) -> None:
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        if armed:
            self.was_armed = True
        elif self.was_armed and self.reached_altitude:
            self.disarmed_after_arm = True

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        if msg.z <= ALT_REACHED_NED_Z:
            self.reached_altitude = True

    @property
    def mission_complete(self) -> bool:
        return self.was_armed and self.reached_altitude and self.disarmed_after_arm


@pytest.mark.launch(fixture=mission_launch)
def test_basic_mission_arms_climbs_and_lands() -> None:
    rclpy.init()
    watcher = _TelemetryWatcher()
    try:
        deadline = time.monotonic() + MISSION_TIMEOUT_S
        while time.monotonic() < deadline and not watcher.mission_complete:
            rclpy.spin_once(watcher, timeout_sec=0.5)

        assert watcher.was_armed, "drone never armed"
        assert watcher.reached_altitude, f"drone never reached ~{TAKEOFF_ALT_M} m AGL"
        assert watcher.disarmed_after_arm, "drone never disarmed (landing) after the mission"
    finally:
        watcher.destroy_node()
        rclpy.shutdown()
