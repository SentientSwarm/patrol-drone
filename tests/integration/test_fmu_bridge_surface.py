"""SITL integration test: the platform /fmu/* bridge surface + telemetry liveness (01, AC-2/AC-3).

Grows 01-platform e2e coverage (SWM-15) beyond the two canonical 02 missions: it turns the nightly
job's *shell* bridge-up gate into an asserted pytest over the platform contract every downstream
docset consumes — the ``/fmu/out/*`` surface is live and ``vehicle_local_position`` streams steadily
over the uXRCE-DDS bridge (01 DoD AC-2 "PX4 topics returned" / AC-3 "steady rate").

Unlike the mission scenarios this launches nothing: the nightly brings PX4 SITL + the agent up (and
waits for the bridge) before pytest runs, so this test only *observes* the already-live surface with
the same px4_qos the mission node subscribes with. It therefore has no ``launch_pytest`` fixture.

Nightly SITL tier only — never a required per-PR check (OQ-5). Marked ``ros`` so the Layer-A unit
runner (no ROS) skips it. **Live-run status: the precise ~50 Hz rate (AC-3) is confirmed by the
live/manual exit-checklist run; this gate asserts only a generous liveness floor so it can't flake
(SWM-31 owns the measured figures).**
"""

import time

import pytest
import rclpy
from patrol_mission.qos import px4_qos
from px4_msgs.msg import VehicleLocalPosition

from patrol_mission import topics

pytestmark = pytest.mark.ros

# The key /fmu/out/* topics downstream docsets depend on: 02 drives offboard off position/status and
# aborts on battery; 05 records the surface. Names come from patrol_mission.topics (one source of
# truth, the _v1-suffixed PX4 v1.17 outputs), never re-hardcoded here.
_REQUIRED_FMU_TOPICS = (topics.VEHICLE_LOCAL_POSITION, topics.VEHICLE_STATUS, topics.BATTERY_STATUS)

# Liveness window + a deliberately generous floor. Nominal is ~50 Hz (AC-3), but headless
# software-render at RTF < 1 lowers the wall-clock rate, so assert only that telemetry streams
# steadily (>= ~5 Hz sustained), never a precise rate — the precise figure is the live-run's job
# (SWM-31). The nightly waits for the bridge before this runs, so telemetry is already flowing.
_WINDOW_S = 10.0
_MIN_MESSAGES = 50  # >= ~5 Hz over the window — liveness, not a precise-rate assertion


def test_fmu_bridge_surface_and_liveness() -> None:
    rclpy.init()
    node = rclpy.create_node("fmu_bridge_probe")
    received: list[int] = []
    node.create_subscription(
        VehicleLocalPosition,
        topics.VEHICLE_LOCAL_POSITION,
        lambda _msg: received.append(1),
        px4_qos(),
    )
    try:
        deadline = time.monotonic() + _WINDOW_S
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)

        # AC-2: the /fmu/out/* surface carries the topics the downstream docsets consume. Query the
        # PUBLISHER side: get_topic_names_and_types() returns every topic with ANY endpoint, and a
        # node sees its own — so this probe's own subscription above put VEHICLE_LOCAL_POSITION in
        # the set whether or not PX4 published it, and that one presence claim was self-satisfying.
        # Applied to all three, not just the self-satisfied one: the other two are correct today only
        # by the accident of having no local endpoint, and a maintainer adding a VEHICLE_STATUS
        # subscription to this probe would silently reintroduce the same weakness.
        missing = [t for t in _REQUIRED_FMU_TOPICS if not node.get_publishers_info_by_topic(t)]
        assert not missing, f"/fmu/* bridge surface has no PUBLISHER for: {missing}"

        # AC-3: vehicle_local_position streamed steadily over the window (liveness floor).
        assert len(received) >= _MIN_MESSAGES, (
            f"vehicle_local_position delivered {len(received)} msgs in {_WINDOW_S:.0f}s "
            f"(need >= {_MIN_MESSAGES}); the bridge is up but telemetry is not streaming steadily"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()
