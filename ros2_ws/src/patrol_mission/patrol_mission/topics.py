"""Single source of truth for the PX4 ``/fmu/*`` topic names (design §4.2.6, 01-platform §4.2.4).

PX4 v1.17's uXRCE-DDS bridge suffixes a topic with ``_v{N}`` ONLY when the underlying message
declares ``MESSAGE_VERSION >= 1``; unversioned (or ``MESSAGE_VERSION = 0``) messages keep the bare
name. So the node's OUTPUT topics — VehicleStatus and VehicleLocalPosition, both ``MESSAGE_VERSION
1`` — are ``..._v1``, while the offboard-control INPUT trio is NOT: OffboardControlMode is
unversioned, and TrajectorySetpoint/VehicleCommand are ``MESSAGE_VERSION 0``. PX4 subscribes to the
BARE input names; publishing those to ``..._v1`` reaches no subscriber, so PX4 never sees the
offboard-control stream and refuses the offboard switch (the drone stays in LOITER, disarmed). An
earlier revision suffixed the inputs too and was only caught by a live SITL run.

These names are the contract the node publishes/subscribes against — kept here, ROS-free, so a
Layer-A unit test (``tests/unit/test_topics.py``) pins the per-surface rule without a live bridge,
and the node + the SITL integration test share one definition rather than duplicating literals.
"""

# /fmu/out — PX4 -> ROS 2 (subscribed by the node). All three are MESSAGE_VERSION 1 (verified in the
# vendored px4_msgs/msg/*.msg) -> the bridge adds _v1. BatteryStatus (M4, the low-battery abort)
# joins VehicleStatus + VehicleLocalPosition on the versioned-output surface.
VEHICLE_LOCAL_POSITION = "/fmu/out/vehicle_local_position_v1"
VEHICLE_STATUS = "/fmu/out/vehicle_status_v1"
BATTERY_STATUS = "/fmu/out/battery_status_v1"

# /fmu/in — ROS 2 -> PX4 (published by the node). The offboard-control trio is unversioned / v0,
# so PX4 v1.17 listens on the BARE names (NO _v1) — see module docstring; do not re-add the suffix.
OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode"
TRAJECTORY_SETPOINT = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND = "/fmu/in/vehicle_command"

# Aggregate of every /fmu/* topic the node talks to — the set the regression guard checks.
FMU_TOPICS = (
    VEHICLE_LOCAL_POSITION,
    VEHICLE_STATUS,
    BATTERY_STATUS,
    OFFBOARD_CONTROL_MODE,
    TRAJECTORY_SETPOINT,
    VEHICLE_COMMAND,
)

# /patrol/* — the mission-orchestration surface (M4, OQ-3). Plain std_msgs so MCAP records and
# Foxglove renders them with no custom-type plugin (design §4.4.2). mission_state + current_waypoint
# are the observable mission surface (and the DWELL+index capture trigger for 04, OQ-7); abort is the
# inbound external-abort signal (MC-6). These are NOT /fmu/* — PX4 never sees them.
PATROL_MISSION_STATE = "/patrol/mission_state"  # std_msgs/String — the returned MissionState name
PATROL_CURRENT_WAYPOINT = "/patrol/current_waypoint"  # std_msgs/Int32 — active index (-1 = none)
PATROL_ABORT = "/patrol/abort"  # std_msgs/Bool — inbound external-abort (MC-6)

PATROL_TOPICS = (
    PATROL_MISSION_STATE,
    PATROL_CURRENT_WAYPOINT,
    PATROL_ABORT,
)

# Map of public constant name -> topic, derived from this module's own constants (never a hand-kept
# second list). Lets a shell/CI step resolve a canonical topic name — incl. the version-sensitive
# ``_v1`` suffix — from this one source instead of re-hardcoding the literal (Hermes Low). See the
# ``python -m patrol_mission.topics <NAME>`` entry point below.
_NAMED_TOPICS = {
    name: value
    for name, value in list(globals().items())
    if name.isupper() and isinstance(value, str) and value.startswith("/")
}


def named_topic(name: str) -> str | None:
    """The canonical topic for a constant name (e.g. ``"VEHICLE_STATUS"``), or ``None`` if unknown."""
    return _NAMED_TOPICS.get(name)


if __name__ == "__main__":  # `python -m patrol_mission.topics VEHICLE_STATUS` -> the canonical name
    import sys

    _args = sys.argv[1:]
    _topic = named_topic(_args[0]) if len(_args) == 1 else None
    if _topic is None:
        _known = ", ".join(sorted(_NAMED_TOPICS))
        print(f"usage: python -m patrol_mission.topics <NAME>; known: {_known}", file=sys.stderr)
        raise SystemExit(2)
    print(_topic)
