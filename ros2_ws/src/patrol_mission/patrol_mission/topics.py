"""Single source of truth for the PX4 ``/fmu/*`` topic names (design §4.2.6, 01-platform §4.2.4).

PX4 v1.17 advertises message-versioned topic names with a ``_v1`` suffix; the unversioned names
from the vendored v1.16-era ``px4_ros_com`` example do NOT exist on the M2 bridge. These names are
the contract the node publishes/subscribes against — kept here, ROS-free, so a Layer-A unit test
(``tests/unit/test_topics.py``) can pin them without a live bridge, and the node + the SITL
integration test share one definition rather than duplicating string literals.
"""

# /fmu/out — PX4 -> ROS 2 (subscribed by the node)
VEHICLE_LOCAL_POSITION = "/fmu/out/vehicle_local_position_v1"
VEHICLE_STATUS = "/fmu/out/vehicle_status_v1"

# /fmu/in — ROS 2 -> PX4 (published by the node)
OFFBOARD_CONTROL_MODE = "/fmu/in/offboard_control_mode_v1"
TRAJECTORY_SETPOINT = "/fmu/in/trajectory_setpoint_v1"
VEHICLE_COMMAND = "/fmu/in/vehicle_command_v1"

# Aggregate of every /fmu/* topic the node talks to — the set the regression guard checks.
FMU_TOPICS = (
    VEHICLE_LOCAL_POSITION,
    VEHICLE_STATUS,
    OFFBOARD_CONTROL_MODE,
    TRAJECTORY_SETPOINT,
    VEHICLE_COMMAND,
)
