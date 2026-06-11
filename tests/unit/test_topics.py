"""Layer-A regression guard for the PX4 /fmu/* topic-name contract (design §4.2.6).

PX4 v1.17 advertises message-versioned topic names with a ``_v1`` suffix (01-platform
design §4.2.4: "M3/02 must subscribe to the `_v1` names"). The node imports rclpy/px4_msgs
and cannot be imported on the ROS-free Layer-A runner, so the unversioned-name bug that a
`/code-review` caught was invisible to the unit suite. Centralizing the names in the ROS-free
``patrol_mission.topics`` module lets this test pin the contract with no live bridge — a future
drop of the ``_v1`` suffix fails here in <5 s instead of silently at night against SITL.
"""

from patrol_mission import topics


# Every FMU topic the node talks to must carry the PX4 v1.17 _v1 version suffix.
def test_all_fmu_topics_are_version_suffixed():
    assert topics.FMU_TOPICS, "FMU_TOPICS must not be empty"
    for name in topics.FMU_TOPICS:
        assert name.endswith("_v1"), f"{name!r} is missing the PX4 v1.17 _v1 suffix"


# Each topic is on the /fmu/in or /fmu/out surface (no stray names).
def test_fmu_topics_are_on_the_fmu_surface():
    for name in topics.FMU_TOPICS:
        assert name.startswith(("/fmu/in/", "/fmu/out/")), f"{name!r} is not a /fmu/* topic"


# Pin the exact contract against 01-platform design §4.2.4 (out: position+status; in: the
# offboard control trio). Catches an accidental rename as well as a dropped suffix.
def test_topic_names_match_the_platform_contract():
    assert topics.VEHICLE_LOCAL_POSITION == "/fmu/out/vehicle_local_position_v1"
    assert topics.VEHICLE_STATUS == "/fmu/out/vehicle_status_v1"
    assert topics.OFFBOARD_CONTROL_MODE == "/fmu/in/offboard_control_mode_v1"
    assert topics.TRAJECTORY_SETPOINT == "/fmu/in/trajectory_setpoint_v1"
    assert topics.VEHICLE_COMMAND == "/fmu/in/vehicle_command_v1"


# The aggregate tuple is exactly the five distinct names (no dupes, none missed).
def test_fmu_topics_aggregate_is_complete_and_unique():
    expected = {
        topics.VEHICLE_LOCAL_POSITION,
        topics.VEHICLE_STATUS,
        topics.OFFBOARD_CONTROL_MODE,
        topics.TRAJECTORY_SETPOINT,
        topics.VEHICLE_COMMAND,
    }
    assert set(topics.FMU_TOPICS) == expected
    assert len(topics.FMU_TOPICS) == len(expected)
