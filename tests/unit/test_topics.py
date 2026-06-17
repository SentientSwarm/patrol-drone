"""Layer-A regression guard for the PX4 v1.17 /fmu/* topic-name contract (design §4.2.6).

PX4 v1.17's uXRCE-DDS bridge suffixes a topic with ``_v{N}`` ONLY when the underlying message
declares ``MESSAGE_VERSION >= 1``; unversioned / ``MESSAGE_VERSION 0`` messages keep the bare name.
So the node's OUTPUT topics (VehicleStatus + VehicleLocalPosition, both v1) end in ``_v1``, while
its offboard-control INPUT trio must NOT: OffboardControlMode is unversioned and
TrajectorySetpoint/VehicleCommand are v0, so PX4 subscribes on the bare names. The node imports
rclpy/px4_msgs and can't run on the ROS-free Layer-A runner, so an earlier revision that suffixed
the inputs too was invisible here — PX4 silently ignored the offboard stream and the drone never
left LOITER; only a live SITL run caught it. This test pins the per-surface rule so a re-introduced
(or dropped) suffix fails in <5 s instead of at night against SITL.
"""

from patrol_mission import topics

# The contract split by version surface: outputs are MESSAGE_VERSION>=1 (suffixed); the offboard
# input trio is unversioned / v0 (bare). See the module docstring for why the asymmetry matters.
_VERSIONED_OUT = (topics.VEHICLE_LOCAL_POSITION, topics.VEHICLE_STATUS, topics.BATTERY_STATUS)
_UNVERSIONED_IN = (topics.OFFBOARD_CONTROL_MODE, topics.TRAJECTORY_SETPOINT, topics.VEHICLE_COMMAND)


# Versioned outputs (MESSAGE_VERSION>=1) MUST carry the _v1 suffix the bridge appends.
def test_versioned_outputs_carry_the_v1_suffix():
    for name in _VERSIONED_OUT:
        assert name.endswith("_v1"), f"{name!r} (MESSAGE_VERSION>=1) must keep its _v1 suffix"


# Unversioned / v0 inputs MUST be bare — re-adding _v1 here is the bug that grounded the drone
# (PX4 listens on the bare name, so a suffixed publication reaches no subscriber).
def test_unversioned_inputs_have_no_version_suffix():
    for name in _UNVERSIONED_IN:
        assert not name.endswith("_v1"), f"{name!r} is unversioned/v0 — must NOT carry a _v1 suffix"


# Each topic is on the /fmu/in or /fmu/out surface (no stray names).
def test_fmu_topics_are_on_the_fmu_surface():
    for name in topics.FMU_TOPICS:
        assert name.startswith(("/fmu/in/", "/fmu/out/")), f"{name!r} is not a /fmu/* topic"


# Pin the exact contract (out: position+status+battery, _v1; in: the offboard-control trio, bare).
# Catches an accidental rename as well as a wrong-surface suffix.
def test_topic_names_match_the_platform_contract():
    assert topics.VEHICLE_LOCAL_POSITION == "/fmu/out/vehicle_local_position_v1"
    assert topics.VEHICLE_STATUS == "/fmu/out/vehicle_status_v1"
    assert topics.BATTERY_STATUS == "/fmu/out/battery_status_v1"
    assert topics.OFFBOARD_CONTROL_MODE == "/fmu/in/offboard_control_mode"
    assert topics.TRAJECTORY_SETPOINT == "/fmu/in/trajectory_setpoint"
    assert topics.VEHICLE_COMMAND == "/fmu/in/vehicle_command"


# The /patrol/* surface (M4, OQ-3): std_msgs orchestration topics, distinct from /fmu/*.
def test_patrol_topics_are_on_the_patrol_surface():
    for name in topics.PATROL_TOPICS:
        assert name.startswith("/patrol/"), f"{name!r} is not a /patrol/* topic"


def test_patrol_topic_names_match_contract():
    assert topics.PATROL_MISSION_STATE == "/patrol/mission_state"
    assert topics.PATROL_CURRENT_WAYPOINT == "/patrol/current_waypoint"
    assert topics.PATROL_ABORT == "/patrol/abort"


# The aggregate tuple is exactly the six distinct names (no dupes, none missed).
def test_fmu_topics_aggregate_is_complete_and_unique():
    expected = {*_VERSIONED_OUT, *_UNVERSIONED_IN}
    assert set(topics.FMU_TOPICS) == expected
    assert len(topics.FMU_TOPICS) == len(expected)
