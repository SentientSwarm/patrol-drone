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
from patrol_mission.commands import Px4CommandKind, build_vehicle_commands
from patrol_mission.config import load_mission_config
from patrol_mission.frames import Point, to_ned_from_origin
from patrol_mission.state_machine import (
    Command,
    MissionState,
    MissionStateMachine,
    Telemetry,
    local_position_usable,
    telemetry_fresh,
)

# PX4 needs a continuous setpoint + offboard-mode stream established before it will
# accept the offboard-mode switch (A-2). Hold the stream this many ticks first.
_OFFBOARD_STREAM_WARMUP_TICKS = 10
_TIMER_PERIOD_S = 0.1  # 10 Hz (A-2 keepalive rate)
_EKF_ORIGIN_NED: Point = (0.0, 0.0, 0.0)  # SITL local position is already origin-relative NED
# Max age (s) of a cached /fmu/out/* sample before the node stops advancing the mission on it
# (Hermes Medium). Sized well above the slowest /fmu/out cadence (vehicle_status) so normal jitter
# never trips it; its job is to catch a STOPPED stream (age grows unbounded), not to police latency.
_TELEMETRY_TIMEOUT_S = 2.0

# The ONE site that binds the pure Px4CommandKind symbols (patrol_mission.commands) to their
# px4_msgs MAVLink IDs. Referencing the VehicleCommand.* constants directly means the IDs can't
# drift; the pure builder stays rclpy-free and Layer-A testable.
_VEHICLE_CMD_ID: dict[Px4CommandKind, int] = {
    Px4CommandKind.SET_OFFBOARD: VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    Px4CommandKind.ARM: VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    Px4CommandKind.LAND: VehicleCommand.VEHICLE_CMD_NAV_LAND,
}


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

        # Init to None (not a default-constructed message): a default VehicleStatus reports
        # disarmed-at-origin, which is indistinguishable from "no telemetry has arrived yet".
        # _on_tick gates state-machine progression behind receipt of BOTH streams so the node
        # never publishes arm/offboard intent on stale defaults before observability is confirmed.
        self._pos: VehicleLocalPosition | None = None
        self._status: VehicleStatus | None = None
        # Receipt time (node clock, s) of the latest sample on each stream — None until first arrival.
        # _on_tick uses these to refuse to advance the mission on a frozen fix (Hermes Medium).
        self._pos_rx_s: float | None = None
        self._status_rx_s: float | None = None
        self._state = MissionState.IDLE
        self._warmup = 0

        home_ned = to_ned_from_origin(
            self._cfg.home_position, self._cfg.home_frame, _EKF_ORIGIN_NED
        )
        waypoints_ned = [
            to_ned_from_origin(w.position, w.frame, _EKF_ORIGIN_NED) for w in self._cfg.waypoints
        ]
        self._sm = MissionStateMachine(self._cfg, waypoints_ned, home_ned)

        self.create_timer(_TIMER_PERIOD_S, self._on_tick)
        self.get_logger().info(f"patrol_mission up; mission={mission_yaml}")

    # --- subscriptions ------------------------------------------------------

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        self._pos = msg
        self._pos_rx_s = self._clock_s()

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg
        self._status_rx_s = self._clock_s()

    # --- 10 Hz control loop -------------------------------------------------

    def _on_tick(self) -> None:
        self._publish_keepalive()  # A-2: heartbeat every tick, even before telemetry/offboard
        pos, status = self._pos, self._status
        if pos is None or status is None:
            return  # no telemetry yet — keep the heartbeat alive but do not progress/arm on defaults
        if not local_position_usable(pos.xy_valid, pos.z_valid):
            return  # EKF position estimate not yet valid — heartbeat stays alive, don't arm on it
        now_s = self._clock_s()
        if self._telemetry_stale(now_s):
            # A /fmu/out stream stopped after a valid sample: keep the heartbeat alive (so PX4's own
            # failsafe governs) but do NOT advance the mission on the frozen fix (Hermes Medium).
            self.get_logger().warning(
                "stale /fmu/out telemetry — pausing mission progression", throttle_duration_sec=1.0
            )
            return
        telem = self._build_telemetry(pos, status, now_s)
        self._state, cmd = self._sm.tick(self._state, telem)
        self._issue(cmd)
        if self._warmup < _OFFBOARD_STREAM_WARMUP_TICKS:
            self._warmup += 1

    def _telemetry_stale(self, now_s: float) -> bool:
        """True if either required /fmu/out stream's latest sample is older than the freshness timeout."""
        # Each rx time is set in the same callback as its message, so once _on_tick has confirmed
        # both _pos and _status are non-None neither rx is None here; the `is None` arm is the
        # type-narrowing guard (rx_s is float | None) and a belt-and-braces "unknown age == stale".
        for rx_s in (self._pos_rx_s, self._status_rx_s):
            if rx_s is None or not telemetry_fresh(now_s - rx_s, _TELEMETRY_TIMEOUT_S):
                return True
        return False

    def _build_telemetry(
        self, pos: VehicleLocalPosition, status: VehicleStatus, now_s: float
    ) -> Telemetry:
        return Telemetry(
            now_s=now_s,
            position_ned=(float(pos.x), float(pos.y), float(pos.z)),
            armed=status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            offboard_active=status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
        )

    def _clock_s(self) -> float:
        # float() so mypy sees a float return even where rclpy is untyped (nanoseconds -> Any on the
        # Layer-A runner, which has no rclpy); mirrors _now_us's int() cast. Without it: no-any-return.
        return float(self.get_clock().now().nanoseconds / 1e9)

    def _issue(self, cmd: Command) -> None:
        """Translate a decision-layer Command into /fmu/in/* messages.

        The setpoint is published first so the offboard setpoint stream is established before any
        mode/arm command (A-2). Which VehicleCommands to send — gated on the warmup window and
        ordered offboard-before-arm — is decided by the pure ``build_vehicle_commands`` builder
        (Layer-A tested); here we only map each kind to its px4_msgs ID and publish it.
        """
        if cmd.setpoint_ned is not None:
            self._publish_setpoint(cmd.setpoint_ned, cmd.yaw)
        warmup_elapsed = self._warmup >= _OFFBOARD_STREAM_WARMUP_TICKS
        for pc in build_vehicle_commands(cmd, warmup_elapsed):
            self._send_command(_VEHICLE_CMD_ID[pc.kind], param1=pc.param1, param2=pc.param2)

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
