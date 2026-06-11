"""PatrolMissionNode — the thin rclpy plumbing layer (design §4.2.6, MC-1, INF-M2).

This node owns every *mechanism* and **no decision logic** (a node that branched
on mission state would be a review-rejectable layer violation): it subscribes
``/fmu/out/*``, runs the offboard keepalive heartbeat (A-2), builds a
:class:`~patrol_mission.state_machine.Telemetry` from the latest cache, drives
``MissionStateMachine.tick()`` from a fixed 10 Hz timer, and translates the
returned :class:`~patrol_mission.state_machine.Command` into ``/fmu/in/*``.

The single ENU->NED conversion happens at exactly one site
(:func:`~patrol_mission.frames.to_ned_from_origin`, MC-7). For SITL the
``VehicleLocalPosition`` frame is already EKF-origin-relative NED, so the origin
offset is zero; M4 captures a live origin if needed.

This module imports rclpy + px4_msgs and is therefore **not** exercised by the
Layer-A unit suite; it is covered by ``colcon build`` and the SITL integration
test (T1.7 / AC-5).
"""

from __future__ import annotations

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from patrol_mission import topics
from patrol_mission.config import load_mission_config
from patrol_mission.frames import Point, to_ned_from_origin
from patrol_mission.state_machine import Command, MissionState, MissionStateMachine, Telemetry

# PX4 needs a continuous setpoint + offboard-mode stream established before it will
# accept the offboard-mode switch (A-2). Hold the stream this many ticks first.
_OFFBOARD_STREAM_WARMUP_TICKS = 10
_TIMER_PERIOD_S = 0.1  # 10 Hz (A-2 keepalive rate)
_EKF_ORIGIN_NED: Point = (0.0, 0.0, 0.0)  # SITL local position is already origin-relative NED


def _px4_qos() -> QoSProfile:
    """The /fmu/* QoS PX4's uXRCE-DDS bridge expects (matches px4_ros_com example)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class PatrolMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_mission")
        mission_yaml = str(self.declare_parameter("mission_yaml", "").value)
        if not mission_yaml:
            raise ValueError("parameter 'mission_yaml' is required (path to the mission YAML)")
        self._cfg = load_mission_config(mission_yaml)

        # Topic names are the PX4 v1.17 `_v1`-suffixed contract, defined once in
        # patrol_mission.topics (01-platform design §4.2.4) and pinned by a Layer-A test.
        qos = _px4_qos()
        self._pub_ctrl = self.create_publisher(
            OffboardControlMode, topics.OFFBOARD_CONTROL_MODE, qos
        )
        self._pub_sp = self.create_publisher(TrajectorySetpoint, topics.TRAJECTORY_SETPOINT, qos)
        self._pub_cmd = self.create_publisher(VehicleCommand, topics.VEHICLE_COMMAND, qos)
        self.create_subscription(
            VehicleLocalPosition, topics.VEHICLE_LOCAL_POSITION, self._on_pos, qos
        )
        self.create_subscription(VehicleStatus, topics.VEHICLE_STATUS, self._on_status, qos)

        self._pos = VehicleLocalPosition()
        self._status = VehicleStatus()
        self._state = MissionState.IDLE
        self._warmup = 0

        home_ned = to_ned_from_origin(
            self._cfg.home_position, self._cfg.home_frame, _EKF_ORIGIN_NED
        )
        waypoints_ned = [
            to_ned_from_origin(w.position_enu, w.frame, _EKF_ORIGIN_NED)
            for w in self._cfg.waypoints
        ]
        self._sm = MissionStateMachine(self._cfg, waypoints_ned, home_ned)

        self.create_timer(_TIMER_PERIOD_S, self._on_tick)
        self.get_logger().info(f"patrol_mission up; mission={mission_yaml}")

    # --- subscriptions ------------------------------------------------------

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        self._pos = msg

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg

    # --- 10 Hz control loop -------------------------------------------------

    def _on_tick(self) -> None:
        self._publish_keepalive()  # A-2: heartbeat every tick, before and during offboard
        telem = self._build_telemetry()
        self._state, cmd = self._sm.tick(self._state, telem)
        self._issue(cmd)
        if self._warmup < _OFFBOARD_STREAM_WARMUP_TICKS:
            self._warmup += 1

    def _build_telemetry(self) -> Telemetry:
        return Telemetry(
            now_s=self.get_clock().now().nanoseconds / 1e9,
            position_ned=(float(self._pos.x), float(self._pos.y), float(self._pos.z)),
            armed=self._status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            offboard_active=self._status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        )

    def _issue(self, cmd: Command) -> None:
        """Translate a decision-layer Command into /fmu/in/* messages."""
        if cmd.setpoint_ned is not None:
            self._publish_setpoint(cmd.setpoint_ned, cmd.yaw)
        # Only command mode/arming once the keepalive stream is established (A-2). Engage offboard
        # BEFORE arming — PX4 rejects arming outside offboard, so this is the proven
        # px4_ros_com offboard_control.py order (engage_offboard_mode() then arm()).
        if self._warmup >= _OFFBOARD_STREAM_WARMUP_TICKS:
            if cmd.set_offboard:
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            if cmd.arm:
                self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        if cmd.land:
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    # --- /fmu/in publishers -------------------------------------------------

    def _publish_keepalive(self) -> None:
        msg = OffboardControlMode()
        msg.position = True
        msg.timestamp = self._now_us()
        self._pub_ctrl.publish(msg)

    def _publish_setpoint(self, ned: Point, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        msg.yaw = float(yaw)
        msg.timestamp = self._now_us()
        self._pub_sp.publish(msg)

    def _send_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self._now_us()
        self._pub_cmd.publish(msg)

    def _now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = PatrolMissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
