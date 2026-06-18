"""Fast fake-based seam test for the PatrolMissionNode rclpy glue (Hermes Medium, design §4.2.6).

Layer-A by construction: it stubs ``rclpy`` + ``px4_msgs`` in :data:`sys.modules` so ``node.py`` —
the thin plumbing that owns telemetry-presence/EKF gating, the stale-telemetry pause, the 10 Hz
``tick`` dispatch, command issuing, and warmup mutation — runs on the ROS-free runner. Full SITL
(AC-5 / T1.7) stays the end-to-end check; this guards the glue's *branch* logic per-PR rather than
only in the nightly tier, closing the "required checks can pass while glue regresses" gap.

The node is still omitted from the ≥85% coverage gate (pyproject ``[tool.coverage.run]``): that gate
scopes the percentage floor to the three rclpy-free modules. This test adds behavioural coverage of
the plumbing without measuring it as a percentage (``main()`` and the publishers only run under SITL).
"""

from __future__ import annotations

import importlib
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, cast

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
MISSION_BASIC = REPO_ROOT / "ros2_ws/src/patrol_bringup/config/mission_basic.yaml"


# --- px4_msgs / rclpy stand-ins -------------------------------------------------


class _Msg:
    """Permissive stand-in for a px4_msgs message: any field can be set at construction or after."""

    def __init__(self, **fields: Any) -> None:
        self.__dict__.update(fields)


class _VehicleCommand(_Msg):
    # MAVLink IDs the node binds Px4CommandKind to in its ONE _VEHICLE_CMD_ID site.
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
    VEHICLE_CMD_NAV_LAND = 21


class _VehicleStatus(_Msg):
    ARMING_STATE_ARMED = 2
    NAVIGATION_STATE_OFFBOARD = 14


class _QoSProfile:
    def __init__(self, **_kw: Any) -> None: ...


class _FakePublisher:
    def __init__(self) -> None:
        self.published: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class _FakeClock:
    def __init__(self) -> None:
        self.ns = 0

    def now(self) -> SimpleNamespace:
        return SimpleNamespace(nanoseconds=self.ns)


class _RecordingLogger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def info(self, _msg: object) -> None: ...

    def warning(self, msg: str, **_kw: Any) -> None:
        self.warnings.append(msg)  # node throttles via throttle_duration_sec= (swallowed by **_kw)


class _FakeNode:
    """rclpy.node.Node stand-in: records publishers/timers and serves the mission_yaml param."""

    mission_yaml = ""  # the fixture sets this before each node is constructed

    def __init__(self, _name: str) -> None:
        self.clock = _FakeClock()
        self.logger = _RecordingLogger()
        self.pubs: dict[str, _FakePublisher] = {}
        self.timers: list[tuple[float, Any]] = []

    def declare_parameter(self, name: str, default: Any) -> SimpleNamespace:
        value = self.mission_yaml if name == "mission_yaml" else default
        return SimpleNamespace(value=value)

    def create_publisher(self, _msg_type: Any, topic: str, _qos: Any) -> _FakePublisher:
        pub = _FakePublisher()
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, *_a: Any) -> None: ...  # node calls this positionally only

    def create_timer(self, period: float, callback: Any) -> None:
        self.timers.append((period, callback))

    def get_clock(self) -> _FakeClock:
        return self.clock

    def get_logger(self) -> _RecordingLogger:
        return self.logger


def _stub_module(name: str, **attrs: Any) -> ModuleType:
    mod = ModuleType(name)
    for attr, value in attrs.items():
        setattr(mod, attr, value)
    return mod


def _px4_msg_module() -> ModuleType:
    return _stub_module(
        "px4_msgs.msg",
        BatteryStatus=_Msg,
        OffboardControlMode=_Msg,
        TrajectorySetpoint=_Msg,
        VehicleCommand=_VehicleCommand,
        VehicleLocalPosition=_Msg,
        VehicleStatus=_VehicleStatus,
    )


def _std_msg_module() -> ModuleType:
    # std_msgs/{Bool,Int32,String} — the node's /patrol/* surface. Permissive _Msg: `.data` is set
    # at publish time and read back off the recording publisher in the assertions.
    return _stub_module("std_msgs.msg", Bool=_Msg, Int32=_Msg, String=_Msg)


def _qos_module() -> ModuleType:
    enum = SimpleNamespace(BEST_EFFORT=1, RELIABLE=2, TRANSIENT_LOCAL=1, VOLATILE=2, KEEP_LAST=1)
    return _stub_module(
        "rclpy.qos",
        QoSProfile=_QoSProfile,
        ReliabilityPolicy=enum,
        DurabilityPolicy=enum,
        HistoryPolicy=enum,
    )


@pytest.fixture
def node_mod(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """Import patrol_mission.node against stubbed rclpy/px4_msgs; restored on teardown."""
    stubs = {
        "rclpy": _stub_module("rclpy"),
        "rclpy.node": _stub_module("rclpy.node", Node=_FakeNode),
        "rclpy.qos": _qos_module(),
        "px4_msgs": _stub_module("px4_msgs"),
        "px4_msgs.msg": _px4_msg_module(),
        "std_msgs": _stub_module("std_msgs"),
        "std_msgs.msg": _std_msg_module(),
    }
    for name, mod in stubs.items():
        monkeypatch.setitem(sys.modules, name, mod)
    # Force a fresh import so the module binds against the stubs, and drop it afterwards so a later
    # real-ROS import (integration tier) re-binds against the genuine rclpy.
    monkeypatch.delitem(sys.modules, "patrol_mission.node", raising=False)
    module = importlib.import_module("patrol_mission.node")
    yield module
    sys.modules.pop("patrol_mission.node", None)


@pytest.fixture
def node(node_mod: ModuleType) -> Any:
    _FakeNode.mission_yaml = str(MISSION_BASIC)
    return node_mod.PatrolMissionNode()


# --- helpers --------------------------------------------------------------------


def _valid_pos(node_mod: ModuleType, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Any:
    pos = node_mod.VehicleLocalPosition(x=x, y=y, z=z, xy_valid=True, z_valid=True)
    return pos


def _status(node_mod: ModuleType, *, armed: bool = False, offboard: bool = False) -> Any:
    status = node_mod.VehicleStatus()
    status.arming_state = node_mod.VehicleStatus.ARMING_STATE_ARMED if armed else 0
    status.nav_state = node_mod.VehicleStatus.NAVIGATION_STATE_OFFBOARD if offboard else 0
    return status


def _feed_valid_fresh(node: Any, node_mod: ModuleType, **status_kw: Any) -> None:
    node._on_pos(_valid_pos(node_mod))
    node._on_status(_status(node_mod, **status_kw))


def _feed_battery(
    node: Any, node_mod: ModuleType, *, remaining: float, connected: bool = True
) -> None:
    node._on_battery(node_mod.BatteryStatus(remaining=remaining, connected=connected))


def _feed_abort(node: Any, *, value: bool) -> None:
    node._on_abort(_Msg(data=value))


def _pub(node: Any, topic: str) -> _FakePublisher:
    """The recording publisher the node registered for ``topic`` (its /fmu/in/* sink)."""
    return cast(_FakePublisher, node.pubs[topic])


# --- tests ----------------------------------------------------------------------


def test_keepalive_published_every_tick_even_without_telemetry(node: Any, node_mod: ModuleType):
    # A-2: the OffboardControlMode heartbeat streams from the very first tick, before any /fmu/out.
    node._on_tick()

    ctrl = _pub(node, node_mod.topics.OFFBOARD_CONTROL_MODE)
    assert len(ctrl.published) == 1
    assert ctrl.published[0].position is True


def _arrange_no_telemetry(node: Any, node_mod: ModuleType) -> None:
    pass  # leave _pos/_status as None


def _arrange_invalid_ekf(node: Any, node_mod: ModuleType) -> None:
    node._on_pos(node_mod.VehicleLocalPosition(x=0.0, y=0.0, z=0.0, xy_valid=False, z_valid=False))
    node._on_status(_status(node_mod))


def _arrange_stale_telemetry(node: Any, node_mod: ModuleType) -> None:
    _feed_valid_fresh(node, node_mod)
    node.clock.ns += int(5 * 1e9)  # advance 5 s, well past _TELEMETRY_TIMEOUT_S, freezing the fix


@pytest.mark.parametrize(
    "arrange",
    [_arrange_no_telemetry, _arrange_invalid_ekf, _arrange_stale_telemetry],
    ids=["no_telemetry", "invalid_ekf", "stale_telemetry"],
)
def test_gates_hold_heartbeat_but_block_progression(node: Any, node_mod: ModuleType, arrange):
    arrange(node, node_mod)

    node._on_tick()

    assert len(_pub(node, node_mod.topics.OFFBOARD_CONTROL_MODE).published) == 1  # heartbeat alive
    assert _pub(node, node_mod.topics.TRAJECTORY_SETPOINT).published == []  # no setpoint advanced
    assert _pub(node, node_mod.topics.VEHICLE_COMMAND).published == []  # no arm/offboard issued
    assert node._warmup == 0  # warmup does not advance on a gated tick
    assert node._state is node_mod.MissionState.IDLE


def test_stale_gate_logs_warning(node: Any, node_mod: ModuleType):
    _arrange_stale_telemetry(node, node_mod)

    node._on_tick()

    assert any("stale" in w for w in node.logger.warnings)


def test_resume_after_stale_resets_machine_timing(
    node: Any, node_mod: ModuleType, monkeypatch: pytest.MonkeyPatch
):
    # Review #1: on the stale->fresh resume edge the node restarts the state-machine's time-based
    # windows (reset_timing) so a HOVER/hold can't complete on wall-time elapsed during a blackout.
    # A normal run resets once (the startup pause edge) and never again while telemetry stays fresh.
    calls = 0
    original = node._sm.reset_timing

    def _spy() -> None:
        nonlocal calls
        calls += 1
        original()

    monkeypatch.setattr(node._sm, "reset_timing", _spy)

    _feed_valid_fresh(node, node_mod)
    node._on_tick()  # first fresh tick after the startup pause -> exactly one reset
    node._on_tick()  # steady fresh ticking -> no further reset
    assert calls == 1

    node.clock.ns += int(5 * 1e9)  # freeze the fix past _TELEMETRY_TIMEOUT_S
    node._on_tick()  # stale -> paused, machine not ticked, no reset
    assert calls == 1

    _feed_valid_fresh(node, node_mod)  # fresh samples arrive at the advanced clock
    node._on_tick()  # resume edge -> the window is restarted
    assert calls == 2


def test_valid_fresh_tick_dispatches_and_advances_warmup(node: Any, node_mod: ModuleType):
    _feed_valid_fresh(node, node_mod)

    node._on_tick()

    # tick() ran: IDLE -> ARMING, and the takeoff setpoint stream was published this tick.
    assert node._state is node_mod.MissionState.ARMING
    assert len(_pub(node, node_mod.topics.TRAJECTORY_SETPOINT).published) == 1
    # Warmup mutation advanced exactly one tick...
    assert node._warmup == 1
    # ...and arm/offboard VehicleCommands are still withheld inside the warmup window (A-2).
    assert _pub(node, node_mod.topics.VEHICLE_COMMAND).published == []


def test_issue_defers_arm_one_tick_after_offboard(node: Any, node_mod: ModuleType):
    # Past warmup, in ARMING (not yet armed): the FIRST tick issues SET_OFFBOARD ONLY — the arm is
    # held one tick so it can't race the mode switch (M3 review #2). The SECOND tick (offboard now
    # requested) issues SET_OFFBOARD then ARM, with the PX4 params.
    node._warmup = node_mod._OFFBOARD_STREAM_WARMUP_TICKS
    node._state = node_mod.MissionState.ARMING
    _feed_valid_fresh(node, node_mod, armed=False, offboard=False)

    node._on_tick()  # tick 1: offboard only, arm deferred
    cmds = _pub(node, node_mod.topics.VEHICLE_COMMAND).published
    assert [c.command for c in cmds] == [node_mod.VehicleCommand.VEHICLE_CMD_DO_SET_MODE]
    assert (cmds[0].param1, cmds[0].param2) == (1.0, 6.0)  # custom-offboard mode params

    node._on_tick()  # tick 2: offboard (re-requested) then arm
    cmds = _pub(node, node_mod.topics.VEHICLE_COMMAND).published
    assert [c.command for c in cmds] == [
        node_mod.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        node_mod.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        node_mod.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    ]
    assert cmds[-1].param1 == 1.0  # arm


def test_issue_maps_land_ungated_by_warmup(node: Any, node_mod: ModuleType):
    # LAND is the one VehicleCommand never gated on the offboard warmup (A-2): a LANDING tick must
    # map straight through to VEHICLE_CMD_NAV_LAND even while still inside the warmup window.
    node._state = node_mod.MissionState.LANDING
    _feed_valid_fresh(node, node_mod, armed=True)  # armed -> _landing keeps issuing land

    node._on_tick()

    assert node._warmup < node_mod._OFFBOARD_STREAM_WARMUP_TICKS  # still warming up, yet...
    cmds = _pub(node, node_mod.topics.VEHICLE_COMMAND).published
    assert [c.command for c in cmds] == [node_mod.VehicleCommand.VEHICLE_CMD_NAV_LAND]


# T2.4 (OQ-3): the node publishes the observable /patrol/* surface every progressing tick —
# mission_state derived from the returned enum name (one source) + the active waypoint index.
def test_patrol_surface_published_from_returned_state(node: Any, node_mod: ModuleType):
    _feed_valid_fresh(node, node_mod)

    node._on_tick()  # IDLE -> ARMING

    assert _pub(node, node_mod.topics.PATROL_MISSION_STATE).published[-1].data == "ARMING"
    assert _pub(node, node_mod.topics.PATROL_CURRENT_WAYPOINT).published[-1].data == -1


# T2.4 (MC-6): an external /patrol/abort True wired into telemetry drives the ABORT transition,
# observable on /patrol/mission_state.
def test_external_abort_wired_into_telemetry_drives_abort(node: Any, node_mod: ModuleType):
    _feed_abort(node, value=True)
    _feed_valid_fresh(node, node_mod)

    node._on_tick()

    assert node._state is node_mod.MissionState.ABORT
    assert _pub(node, node_mod.topics.PATROL_MISSION_STATE).published[-1].data == "ABORT"


# T2.4 (MC-6/AC-7): a BatteryStatus below the configured threshold drives the low-battery abort.
def test_low_battery_telemetry_drives_abort(node: Any, node_mod: ModuleType):
    _feed_battery(node, node_mod, remaining=0.1)  # mission_basic.yaml threshold is 0.20
    _feed_valid_fresh(node, node_mod)

    node._on_tick()

    assert node._state is node_mod.MissionState.ABORT


# T2.4: an ABSENT BatteryStatus must not fabricate a low-battery abort (defaults to full) — the
# mission progresses normally (IDLE -> ARMING) when only pos+status are present.
def test_absent_battery_does_not_abort(node: Any, node_mod: ModuleType):
    _feed_valid_fresh(node, node_mod)

    node._on_tick()

    assert node._state is node_mod.MissionState.ARMING


# Hermes High: PX4 reports remaining=-1 (and connected=False) when capacity is unknown — not yet
# estimated after boot, or the battery disconnected. The node must not feed that as a near-empty
# battery and false-abort; the mission progresses normally (IDLE -> ARMING).
@pytest.mark.parametrize(
    ("remaining", "connected"),
    [(-1.0, True), (0.5, False)],
    ids=["invalid_sentinel", "disconnected"],
)
def test_unknown_battery_reading_does_not_abort(
    node: Any, node_mod: ModuleType, remaining: float, connected: bool
):
    _feed_battery(node, node_mod, remaining=remaining, connected=connected)
    _feed_valid_fresh(node, node_mod)

    node._on_tick()

    assert node._state is node_mod.MissionState.ARMING
