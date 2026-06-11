"""Pure PX4 VehicleCommand sequencing (design §4.2.6, MC-1) — the rclpy-free half of `_issue`.

The node owns ROS *mechanism* but the *decision* of which ``VehicleCommand``s to emit, with
which params, in which order, and under the offboard-stream warmup gate (A-2) is pure data and
belongs here so it gets Layer-A unit coverage (INF-M1) instead of only nightly SITL.

This module deliberately imports **no** rclpy/px4_msgs: it speaks in :class:`Px4CommandKind`
symbols, and the node maps each kind to its ``px4_msgs.VehicleCommand.VEHICLE_CMD_*`` constant
at the single ROS-side site (so the numeric IDs live in exactly one place and cannot drift).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from patrol_mission.state_machine import Command


class Px4CommandKind(Enum):
    """The PX4 VehicleCommands the basic mission issues (mapped to MAVLink IDs node-side)."""

    SET_OFFBOARD = auto()  # VEHICLE_CMD_DO_SET_MODE -> custom PX4 offboard
    ARM = auto()  # VEHICLE_CMD_COMPONENT_ARM_DISARM
    LAND = auto()  # VEHICLE_CMD_NAV_LAND


@dataclass(frozen=True)
class Px4Command:
    """One VehicleCommand to send: a kind plus its MAVLink param payload."""

    kind: Px4CommandKind
    param1: float = 0.0
    param2: float = 0.0


def build_vehicle_commands(cmd: Command, warmup_elapsed: bool) -> list[Px4Command]:
    """The ordered VehicleCommands for a decision-layer ``Command`` this tick.

    Mode/arm are held until the offboard setpoint stream is established
    (``warmup_elapsed``, A-2) and offboard is requested **before** arm — PX4 rejects
    arming outside offboard (the proven px4_ros_com order). ``land`` is intentionally
    not warmup-gated: landing must be commandable regardless of the offboard warmup.
    """
    out: list[Px4Command] = []
    if warmup_elapsed:
        if cmd.set_offboard:
            out.append(Px4Command(Px4CommandKind.SET_OFFBOARD, param1=1.0, param2=6.0))
        if cmd.arm:
            out.append(Px4Command(Px4CommandKind.ARM, param1=1.0))
    if cmd.land:
        out.append(Px4Command(Px4CommandKind.LAND))
    return out
