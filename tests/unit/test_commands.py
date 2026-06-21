"""Unit tests for the pure PX4 VehicleCommand builder (design §4.2.6, MC-1, INF-M1).

Layer-A: ROS-free. Locks the command *params*, *order*, and *warmup gating* that PX4 cares
about — the offboard-before-arm sequence, the DO_SET_MODE custom-offboard params (1.0/6.0), the
arm param (1.0), and that mode/arm are withheld until the offboard setpoint stream is
established (A-2) while ``land`` stays ungated. This is the command-sequence coverage the M3
review asked for, kept off the nightly/SITL tier.
"""

import pytest
from patrol_mission.commands import Px4Command, Px4CommandKind, build_vehicle_commands
from patrol_mission.state_machine import Command

_SET_OFFBOARD = Px4Command(Px4CommandKind.SET_OFFBOARD, param1=1.0, param2=6.0)
_ARM = Px4Command(Px4CommandKind.ARM, param1=1.0)
_LAND = Px4Command(Px4CommandKind.LAND)


@pytest.mark.parametrize(
    ("cmd", "warmup_elapsed", "offboard_requested", "expected"),
    [
        # Warmup not yet elapsed: mode + arm are withheld even when requested...
        (Command(set_offboard=True, arm=True), False, False, []),
        # ...but land is never gated on the offboard warmup.
        (Command(land=True), False, False, [_LAND]),
        # Warmup elapsed: each request maps to its command with the PX4 params.
        (Command(set_offboard=True), True, False, [_SET_OFFBOARD]),
        # Arm is withheld until offboard has been requested on a PRIOR tick...
        (Command(arm=True), True, False, []),
        # ...and issued once it has.
        (Command(arm=True), True, True, [_ARM]),
        # The KEY one-tick gap: the FIRST tick (offboard not yet requested) sends offboard ONLY —
        # arm is deferred so it can't race the mode switch (M3 review #2).
        (Command(set_offboard=True, arm=True), True, False, [_SET_OFFBOARD]),
        # Steady-state retry tick (offboard already requested): offboard + arm together.
        (Command(set_offboard=True, arm=True), True, True, [_SET_OFFBOARD, _ARM]),
        (Command(land=True), True, False, [_LAND]),
        # A no-op command (e.g. pure setpoint streaming) emits no VehicleCommands.
        (Command(setpoint_ned=(0.0, 0.0, -5.0)), True, True, []),
    ],
    ids=[
        "warmup_pending_withholds_mode_and_arm",
        "warmup_pending_still_lands",
        "set_offboard_params",
        "arm_withheld_until_offboard_requested",
        "arm_param_after_offboard_requested",
        "first_tick_offboard_only_arm_deferred",
        "steady_state_offboard_then_arm",
        "land_command",
        "setpoint_only_emits_nothing",
    ],
)
def test_build_vehicle_commands(cmd, warmup_elapsed, offboard_requested, expected):
    assert build_vehicle_commands(cmd, warmup_elapsed, offboard_requested) == expected
