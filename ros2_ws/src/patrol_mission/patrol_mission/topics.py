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

# /fmu/out — PX4 -> ROS 2 (subscribed by the node). Both are MESSAGE_VERSION 1 -> the bridge adds _v1.
VEHICLE_LOCAL_POSITION = "/fmu/out/vehicle_local_position_v1"
VEHICLE_STATUS = "/fmu/out/vehicle_status_v1"

# /fmu/in — ROS 2 -> PX4 (published by the node). The offboard-control trio is unversioned / v0,
# so PX4 v1.17 listens on the BARE names (NO _v1) — see module docstring; do not re-add the suffix.
OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode"
TRAJECTORY_SETPOINT = "/fmu/in/trajectory_setpoint"
VEHICLE_COMMAND = "/fmu/in/vehicle_command"

# Aggregate of every /fmu/* topic the node talks to — the set the regression guard checks.
FMU_TOPICS = (
    VEHICLE_LOCAL_POSITION,
    VEHICLE_STATUS,
    OFFBOARD_CONTROL_MODE,
    TRAJECTORY_SETPOINT,
    VEHICLE_COMMAND,
)
