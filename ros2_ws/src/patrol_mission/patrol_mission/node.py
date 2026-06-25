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

This module imports rclpy + px4_msgs, so it is not part of the ≥85% Layer-A
coverage gate. Its glue logic is still guarded fast and per-PR by
``tests/unit/test_node_glue.py`` (which stubs rclpy/px4_msgs so the branch logic
runs ROS-free); ``colcon build`` and the SITL integration test (T1.7 / AC-5)
remain the live-environment end-to-end checks.
"""

from __future__ import annotations

import rclpy
from px4_msgs.msg import (
    BatteryStatus,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String

from patrol_mission import topics
from patrol_mission.commands import Px4CommandKind, build_vehicle_commands
from patrol_mission.config import Waypoint, load_mission_config
from patrol_mission.frames import Point, enu_yaw_to_ned, to_ned_from_origin
from patrol_mission.qos import (
    patrol_abort_qos,
    patrol_event_qos,
    patrol_state_qos,
    px4_qos,
)
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
# Max age (s) of the cached BatteryStatus before the node treats the low-battery reading as unknown
# rather than live safety evidence (Hermes Medium, PR #8 R11). This is a STOPPED-stream detector, not
# a latency policer. Deliberately decoupled from (and much larger than) _TELEMETRY_TIMEOUT_S: the
# battery stream may publish more slowly than position/status, so reusing the tighter position/status
# budget here could forward "unknown" on every tick and silently disable the low-battery abort — the
# opposite of the intent (and unit-tested only, never exercised in SITL, so it would go unnoticed).
# 10 s leaves wide margin for a slow-but-alive battery stream; revisit if a measured SITL cadence
# (not yet pinned in the repo) ever approaches it.
_BATTERY_TIMEOUT_S = 10.0
# Settle delay (s) after entering DWELL before firing the atomic /patrol/dwell capture trigger. The
# drone needs a moment to stop and yaw onto the stand-off pose so the tag is framed and apriltag has
# produced a buffered detection by the time 04's capture gate evaluates. Must be < the configured
# dwell_s so the trigger still fires within the hold. Live runs showed detection lagging dwell entry
# by ~2 s; revisit jointly with completion.hold_time_s / dwell_s if SITL timing changes.
_DWELL_SETTLE_S = 2.0


def _waypoint_yaw_ned(wp: Waypoint) -> float:
    """NED yaw to hold at a waypoint: face the tag for a checkpoint waypoint, else hold North (0.0).

    An inline waypoint carries no facing constraint (``yaw_enu is None``) and keeps the prior NED-0
    heading; a checkpoint waypoint's ENU facing yaw converts at the single MC-7 boundary (SIM-4).
    """
    if wp.yaw_enu is None:
        return 0.0
    return enu_yaw_to_ned(wp.yaw_enu)


# The ONE site that binds the pure Px4CommandKind symbols (patrol_mission.commands) to their
# px4_msgs MAVLink IDs. Referencing the VehicleCommand.* constants directly means the IDs can't
# drift; the pure builder stays rclpy-free and Layer-A testable.
_VEHICLE_CMD_ID: dict[Px4CommandKind, int] = {
    Px4CommandKind.SET_OFFBOARD: VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    Px4CommandKind.ARM: VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    Px4CommandKind.LAND: VehicleCommand.VEHICLE_CMD_NAV_LAND,
}


class PatrolMissionNode(Node):
    def __init__(self) -> None:
        super().__init__("patrol_mission")
        mission_yaml = str(self.declare_parameter("mission_yaml", "").value)
        if not mission_yaml:
            raise ValueError("parameter 'mission_yaml' is required (path to the mission YAML)")
        # OQ-2: the checkpoints path is a parameter (the file is 03-owned). No CWD-relative default —
        # a checkpoint-referencing mission must pass an explicit path, so the resolution never
        # depends on where the launch ran from (Hermes Medium). Only read when a waypoint references
        # a checkpoint_id, so the basic mission never needs the file.
        checkpoints_yaml = str(self.declare_parameter("checkpoints_yaml", "").value)
        self._cfg = load_mission_config(mission_yaml, checkpoints_yaml)

        # Topic names are the PX4 v1.17 `_v1`-suffixed contract, defined once in
        # patrol_mission.topics (01-platform design §4.2.4) and pinned by a Layer-A test.
        qos = px4_qos()
        self._pub_ctrl = self.create_publisher(
            OffboardControlMode, topics.OFFBOARD_CONTROL_MODE, qos
        )
        self._pub_sp = self.create_publisher(TrajectorySetpoint, topics.TRAJECTORY_SETPOINT, qos)
        self._pub_cmd = self.create_publisher(VehicleCommand, topics.VEHICLE_COMMAND, qos)
        self.create_subscription(
            VehicleLocalPosition, topics.VEHICLE_LOCAL_POSITION, self._on_pos, qos
        )
        self.create_subscription(VehicleStatus, topics.VEHICLE_STATUS, self._on_status, qos)
        self.create_subscription(BatteryStatus, topics.BATTERY_STATUS, self._on_battery, qos)

        # /patrol/* — the mission-orchestration surface (OQ-3). mission_state + current_waypoint are
        # the observable mission/capture surface, latched (transient-local) so a late 04/05
        # subscriber sees the latest; /patrol/abort is the inbound external-abort *command*, on a
        # volatile profile so a plain `ros2 topic pub` is QoS-compatible (the abort sticks via the
        # state machine's latch, not topic durability — Hermes Medium).
        state_qos = patrol_state_qos()
        self._pub_state = self.create_publisher(String, topics.PATROL_MISSION_STATE, state_qos)
        self._pub_wp = self.create_publisher(Int32, topics.PATROL_CURRENT_WAYPOINT, state_qos)
        # /patrol/dwell — the atomic OQ-7 capture trigger: one Int32 (the dwelled waypoint index) on
        # the rising edge into DWELL, so 04 never correlates the two non-atomic state topics above.
        self._pub_dwell = self.create_publisher(Int32, topics.PATROL_DWELL, patrol_event_qos())
        self.create_subscription(Bool, topics.PATROL_ABORT, self._on_abort, patrol_abort_qos())

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
        # Receipt time of the latest BatteryStatus — None until first arrival. _build_telemetry uses
        # it to forward "unknown" once a battery reading goes stale, so an old high sample is never
        # treated as live low-battery safety evidence indefinitely (Hermes Medium, PR #8 R11).
        self._battery_rx_s: float | None = None
        # Battery defaults to "full" and abort to False so their ABSENCE never fabricates an abort: a
        # missing BatteryStatus must not fire low-battery, and no /patrol/abort means no abort. The
        # latest received value replaces these; the abort latch itself lives in the state machine's
        # _NON_ABORTABLE exclusion, so a momentary external True still drives the full return home.
        self._battery_remaining = 1.0
        self._abort_requested = False
        self._state = MissionState.IDLE
        self._warmup = 0
        # Dwell-trigger settling (capture timing): /patrol/dwell is fired NOT on the rising edge into
        # DWELL (the drone has not settled and the camera/apriltag have not yet produced a detection at
        # the stand-off pose), but once the drone has held DWELL for _DWELL_SETTLE_S — by then the tag
        # is framed and a fresh detection is buffered for 04's capture gate. One trigger per dwell.
        self._dwell_entered_s: float | None = None
        self._dwell_fired: bool = False
        # True once a DO_SET_MODE (offboard) has been issued on a prior tick. The pure command builder
        # uses it to hold the first ARM one tick after the first offboard request, so arm never races
        # the mode switch on the BEST_EFFORT publisher (M3 review #2).
        self._offboard_requested = False
        # True while the node is skipping state-machine progression (no telemetry, EKF not yet valid,
        # or a stale /fmu/out stream). On the edge back to a fresh, usable tick the node restarts the
        # machine's time-based windows (HOVER/tolerance-hold) so they never complete on wall-time that
        # elapsed while the machine was NOT ticking (Codex/Claude review #1).
        self._progression_paused = True

        home_ned = to_ned_from_origin(
            self._cfg.home_position, self._cfg.home_frame, _EKF_ORIGIN_NED
        )
        waypoints_ned = [
            to_ned_from_origin(w.position, w.frame, _EKF_ORIGIN_NED) for w in self._cfg.waypoints
        ]
        waypoint_yaws_ned = [_waypoint_yaw_ned(w) for w in self._cfg.waypoints]
        self._sm = MissionStateMachine(self._cfg, waypoints_ned, home_ned, waypoint_yaws_ned)

        self.create_timer(_TIMER_PERIOD_S, self._on_tick)
        self.get_logger().info(f"patrol_mission up; mission={mission_yaml}")

    # --- subscriptions ------------------------------------------------------

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        self._pos = msg
        self._pos_rx_s = self._clock_s()

    def _on_status(self, msg: VehicleStatus) -> None:
        self._status = msg
        self._status_rx_s = self._clock_s()

    def _on_battery(self, msg: BatteryStatus) -> None:
        # BatteryStatus.remaining is the 0..1 fraction the low-battery abort threshold compares, but
        # PX4 reports -1 (and connected=False) when capacity is unknown — not yet estimated after
        # boot, or the battery disconnected. Pass an explicit "unknown" (-1.0) through when
        # disconnected so the state machine's battery_low() guard ignores it instead of reading the
        # sentinel as a near-empty battery and false-aborting a valid flight (Hermes High).
        self._battery_remaining = float(msg.remaining) if msg.connected else -1.0
        self._battery_rx_s = self._clock_s()

    def _on_abort(self, msg: Bool) -> None:
        # Reflect the latest /patrol/abort value; the state machine makes a True "stick" through RTH.
        self._abort_requested = bool(msg.data)

    # --- 10 Hz control loop -------------------------------------------------

    def _on_tick(self) -> None:
        self._publish_keepalive()  # A-2: heartbeat every tick, even before telemetry/offboard
        pos, status = self._pos, self._status
        if pos is None or status is None:
            self._progression_paused = True
            return  # no telemetry yet — keep the heartbeat alive but do not progress/arm on defaults
        if not local_position_usable(pos.xy_valid, pos.z_valid):
            self._progression_paused = True
            return  # EKF position estimate not yet valid — heartbeat stays alive, don't arm on it
        now_s = self._clock_s()
        if self._telemetry_stale(now_s):
            # A /fmu/out stream stopped after a valid sample: keep the heartbeat alive (so PX4's own
            # failsafe governs) but do NOT advance the mission on the frozen fix (Hermes Medium).
            self._progression_paused = True
            self.get_logger().warning(
                "stale /fmu/out telemetry — pausing mission progression", throttle_duration_sec=1.0
            )
            return
        if self._progression_paused:
            # Resuming after a pause: restart the active state's time-based windows so a HOVER/hold
            # cannot complete on wall-time that elapsed while the machine was NOT ticking (review #1).
            self._sm.reset_timing()
            self._progression_paused = False
        telem = self._build_telemetry(pos, status, now_s)
        prev_state = self._state
        self._state, cmd = self._sm.tick(self._state, telem)
        self._issue(cmd)
        self._publish_patrol(cmd)  # observable mission surface (OQ-3)
        self._publish_dwell_event(prev_state, cmd, now_s)  # atomic 04 capture trigger (OQ-7)
        if self._warmup < _OFFBOARD_STREAM_WARMUP_TICKS:
            self._warmup += 1

    def _publish_dwell_event(self, prev_state: MissionState, cmd: Command, now_s: float) -> None:
        """Emit the atomic /patrol/dwell capture trigger once per checkpoint (OQ-7).

        Fired once per dwell, _DWELL_SETTLE_S after entering DWELL (not on the rising edge): the drone
        needs that long to stop and yaw onto the stand-off pose so the tag is framed and apriltag has
        buffered a fresh detection by the time 04's capture gate runs — firing on entry (the prior
        behavior) raced detection and the capture was always skipped (live runs: detection lagged dwell
        entry ~2 s). Still exactly one Int32 per checkpoint carrying the dwelled index, so 04 gets an
        unambiguous per-checkpoint trigger with no two-topic correlation (Hermes High).
        """
        if self._state is not MissionState.DWELL:
            self._dwell_entered_s = None  # left DWELL; re-arm for the next checkpoint
            self._dwell_fired = False
            return
        if prev_state is not MissionState.DWELL:
            self._dwell_entered_s = now_s  # rising edge: start the settle clock
        if self._dwell_settle_elapsed(now_s):
            self._pub_dwell.publish(Int32(data=cmd.current_waypoint))
            self._dwell_fired = True

    def _dwell_settle_elapsed(self, now_s: float) -> bool:
        """True once the drone has held DWELL for _DWELL_SETTLE_S and has not yet fired this episode."""
        if self._dwell_fired or self._dwell_entered_s is None:
            return False
        return now_s - self._dwell_entered_s >= _DWELL_SETTLE_S

    def _publish_patrol(self, cmd: Command) -> None:
        """Publish the /patrol/* mission surface (OQ-3).

        mission_state is derived from the just-updated ``self._state`` enum (the single source of
        truth — the Command does not duplicate it); current_waypoint is the active index. This is the
        observable/Foxglove surface; 04's once-per-checkpoint capture trigger is the atomic
        ``/patrol/dwell`` event (OQ-7), not a correlation of these two non-atomic topics.
        """
        state_msg = String()
        state_msg.data = self._state.name
        self._pub_state.publish(state_msg)
        wp_msg = Int32()
        wp_msg.data = cmd.current_waypoint
        self._pub_wp.publish(wp_msg)

    def _telemetry_stale(self, now_s: float) -> bool:
        """True if either required /fmu/out stream's latest sample is older than the freshness timeout."""
        # Each rx time is set in the same callback as its message, so once _on_tick has confirmed
        # both _pos and _status are non-None neither rx is None here; the `is None` arm is the
        # type-narrowing guard (rx_s is float | None) and a belt-and-braces "unknown age == stale".
        for rx_s in (self._pos_rx_s, self._status_rx_s):
            if rx_s is None or not telemetry_fresh(now_s - rx_s, _TELEMETRY_TIMEOUT_S):
                return True
        return False

    def _fresh_battery(self, now_s: float) -> float:
        """The battery fraction to act on this tick, or -1.0 ("unknown") if the reading is stale.

        Forwards the cached value only while a battery sample has been seen recently. A stale sample
        (the stream stopped after a real reading) is reported as unknown — the same -1.0 sentinel the
        disconnected path uses, which ``battery_low()`` ignores — so an old high reading is never
        treated as live low-battery safety evidence (Hermes Medium, PR #8 R11). The no-sample-yet
        default stays non-aborting (a missing BatteryStatus must not fabricate an abort).
        """
        if self._battery_rx_s is None:
            return self._battery_remaining  # no sample yet: keep the non-aborting default (1.0)
        if not telemetry_fresh(now_s - self._battery_rx_s, _BATTERY_TIMEOUT_S):
            return -1.0  # a real reading went stale: report unknown, not the cached value
        return self._battery_remaining

    def _build_telemetry(
        self, pos: VehicleLocalPosition, status: VehicleStatus, now_s: float
    ) -> Telemetry:
        return Telemetry(
            now_s=now_s,
            position_ned=(float(pos.x), float(pos.y), float(pos.z)),
            armed=status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            offboard_active=status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD,
            battery_remaining=self._fresh_battery(now_s),
            abort_requested=self._abort_requested,
            # manual_takeover / timed_out are scaffold guards — never wired in SITL (Phase 2 RC),
            # so they keep their False defaults here. The state machine still unit-tests both paths.
        )

    def _clock_s(self) -> float:
        # float() so mypy sees a float return even where rclpy is untyped (nanoseconds -> Any on the
        # Layer-A runner, which has no rclpy); mirrors _now_us's int() cast. Without it: no-any-return.
        return float(self.get_clock().now().nanoseconds / 1e9)

    def _issue(self, cmd: Command) -> None:
        """Translate a decision-layer Command into /fmu/in/* messages.

        The setpoint is published first so the offboard setpoint stream is established before any
        mode/arm command (A-2). Which VehicleCommands to send — gated on the warmup window, ordered
        offboard-before-arm, and holding the first arm one tick past the first offboard request — is
        decided by the pure ``build_vehicle_commands`` builder (Layer-A tested); here we only map each
        kind to its px4_msgs ID, publish it, and latch that offboard has now been requested.
        """
        if cmd.setpoint_ned is not None:
            self._publish_setpoint(cmd.setpoint_ned, cmd.yaw)
        warmup_elapsed = self._warmup >= _OFFBOARD_STREAM_WARMUP_TICKS
        px4_cmds = build_vehicle_commands(cmd, warmup_elapsed, self._offboard_requested)
        for pc in px4_cmds:
            self._send_command(_VEHICLE_CMD_ID[pc.kind], param1=pc.param1, param2=pc.param2)
        if any(pc.kind is Px4CommandKind.SET_OFFBOARD for pc in px4_cmds):
            self._offboard_requested = True  # next tick may arm (one tick after the mode switch)

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
