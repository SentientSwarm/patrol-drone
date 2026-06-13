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

    def info(self, *_a: Any, **_k: Any) -> None: ...

    def warning(self, msg: str, *_a: Any, **_k: Any) -> None:
        self.warnings.append(msg)


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

    def create_subscription(self, *_a: Any, **_k: Any) -> None: ...

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
        OffboardControlMode=_Msg,
        TrajectorySetpoint=_Msg,
        VehicleCommand=_VehicleCommand,
        VehicleLocalPosition=_Msg,
        VehicleStatus=_VehicleStatus,
    )


def _qos_module() -> ModuleType:
    enum = SimpleNamespace(BEST_EFFORT=1, TRANSIENT_LOCAL=1, KEEP_LAST=1)
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


def test_issue_maps_kinds_to_px4_ids_offboard_before_arm(node: Any, node_mod: ModuleType):
    # Past the warmup window and in ARMING (not yet armed): the node must issue SET_OFFBOARD then ARM.
    node._warmup = node_mod._OFFBOARD_STREAM_WARMUP_TICKS
    node._state = node_mod.MissionState.ARMING
    _feed_valid_fresh(node, node_mod, armed=False, offboard=False)

    node._on_tick()

    cmds = _pub(node, node_mod.topics.VEHICLE_COMMAND).published
    assert [c.command for c in cmds] == [
        node_mod.VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
        node_mod.VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
    ]
    assert (cmds[0].param1, cmds[0].param2) == (1.0, 6.0)  # custom-offboard mode params
    assert cmds[1].param1 == 1.0  # arm


def test_issue_maps_land_ungated_by_warmup(node: Any, node_mod: ModuleType):
    # LAND is the one VehicleCommand never gated on the offboard warmup (A-2): a LANDING tick must
    # map straight through to VEHICLE_CMD_NAV_LAND even while still inside the warmup window.
    node._state = node_mod.MissionState.LANDING
    _feed_valid_fresh(node, node_mod, armed=True)  # armed -> _landing keeps issuing land

    node._on_tick()

    assert node._warmup < node_mod._OFFBOARD_STREAM_WARMUP_TICKS  # still warming up, yet...
    cmds = _pub(node, node_mod.topics.VEHICLE_COMMAND).published
    assert [c.command for c in cmds] == [node_mod.VehicleCommand.VEHICLE_CMD_NAV_LAND]
