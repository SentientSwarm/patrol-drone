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

from pathlib import Path

from patrol_mission import topics

REPO_ROOT = Path(__file__).resolve().parents[2]
_MISSION_LAUNCH = REPO_ROOT / "ros2_ws/src/patrol_bringup/launch/mission_patrol.launch.py"

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
    assert topics.PATROL_DWELL == "/patrol/dwell"
    assert topics.PATROL_ABORT == "/patrol/abort"
    assert topics.PATROL_ABORT_REASON == "/patrol/abort_reason"


# The aggregate tuple is exactly the five distinct /patrol/* names (no dupes, none missed) — the
# atomic /patrol/dwell capture trigger joins the orchestration surface (Hermes High), and the
# outbound /patrol/abort_reason carries the latched abort cause the state alone cannot express (F-09).
def test_patrol_topics_aggregate_is_complete_and_unique():
    expected = {
        topics.PATROL_MISSION_STATE,
        topics.PATROL_CURRENT_WAYPOINT,
        topics.PATROL_DWELL,
        topics.PATROL_ABORT,
        topics.PATROL_ABORT_REASON,
    }
    assert set(topics.PATROL_TOPICS) == expected
    assert len(topics.PATROL_TOPICS) == len(expected)


# The inbound command and the outbound cause are distinct topics — a regression that collapsed them
# would make the low-battery scenario's "no external abort was published" evidence meaningless.
def test_abort_command_and_abort_reason_are_distinct_topics():
    assert topics.PATROL_ABORT != topics.PATROL_ABORT_REASON


# The aggregate tuple is exactly the six distinct names (no dupes, none missed).
def test_fmu_topics_aggregate_is_complete_and_unique():
    expected = {*_VERSIONED_OUT, *_UNVERSIONED_IN}
    assert set(topics.FMU_TOPICS) == expected
    assert len(topics.FMU_TOPICS) == len(expected)


# named_topic backs `python -m patrol_mission.topics <NAME>` (Hermes Low): shell/CI resolve the
# canonical, version-sensitive name from this one source instead of re-hardcoding the _v1 literal.
def test_named_topic_resolves_known_constants():
    assert topics.named_topic("VEHICLE_STATUS") == topics.VEHICLE_STATUS
    assert topics.named_topic("PATROL_ABORT") == topics.PATROL_ABORT


# An unknown name returns None (the CLI turns that into a non-zero exit + usage on stderr).
def test_named_topic_unknown_returns_none():
    assert topics.named_topic("NOT_A_TOPIC") is None
    assert topics.named_topic("") is None


# The resolver map is exactly the /fmu/* and /patrol/* surfaces — auto-derived, so a newly added
# topic constant is reachable from the CLI with no second list to keep in sync.
def test_named_topic_map_covers_every_surface():
    assert set(topics._NAMED_TOPICS.values()) == {*topics.FMU_TOPICS, *topics.PATROL_TOPICS}


# The mission node's ROS node name is a contract too: the SITL acceptance helper waits for THIS
# node's subscriber by name (High #1), so a silent rename would turn a real abort-delivery failure
# into an opaque timeout. It is deliberately NOT a "/"-prefixed value, so it stays out of
# _NAMED_TOPICS above.
def test_mission_node_name_matches_contract():
    assert topics.MISSION_NODE_NAME == "patrol_mission"


def test_mission_node_name_is_a_valid_ros_node_name():
    name = topics.MISSION_NODE_NAME
    assert name, "a ROS node name must be non-empty"
    assert not name[0].isdigit(), "a ROS node name must not start with a digit"
    assert "/" not in name, "a node name is not a topic — it must carry no '/' separator"


# The launch file's `name=` is a __node remap and OVERRIDES the constructor, so it — not node.py — is
# what names the node in the SITL lane the acceptance helper waits on (High #1). Pinning node.py
# alone would leave the drift this constant guards against open in exactly the lane that consumes it.
# Scanned as text rather than imported: launch/launch_ros are not installed on the Layer-A runner
# (and pyproject excludes launch files from mypy for the same reason).
def test_launch_file_names_the_mission_node_from_the_shared_constant():
    assert f'name="{topics.MISSION_NODE_NAME}"' in _MISSION_LAUNCH.read_text()
