"""Fast fake-based seam test for the PerceptionNode rclpy glue (M6.B, T B.4 — design §4.2.9).

Layer-A by construction: it stubs ``rclpy`` + ``cv2`` + the message packages in :data:`sys.modules`
so ``perception_node.py`` — the thin plumbing that owns param declaration, the rosidl message
factory's field mapping, the cv_bridge encode seam, the EKF-origin constant, and the subscription
topic wiring — runs on the ROS-free runner. Full SITL (AC-2/AC-4/AC-6) stays the end-to-end check;
this guards the glue's *mapping/branch* logic per-PR rather than only in the nightly tier.

It exists because the node is omitted from the coverage gate (rclpy entrypoint), yet it holds real
logic a stub-free suite cannot see: a transposed quaternion field, a wrong PX4 topic string (e.g. a
``_v{N}`` version drift), or a regressed required-param guard would otherwise surface only in SITL —
mirroring the ``.q``-vs-``heading`` mismatch a sampler stand-in once masked.
"""

from __future__ import annotations

import importlib
import sys
from collections.abc import Iterator
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import Any, ClassVar

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
CHECKPOINTS_YAML = REPO_ROOT / "sim/config/checkpoints.yaml"

# Toggles whether the stubbed cv2.imencode reports success; reset to True by the node_mod fixture.
_cv2_state = {"ok": True}


# --- permissive message / rclpy stand-ins ---------------------------------------


class _Msg:
    """Permissive message stand-in: any field can be set at construction or after."""

    def __init__(self, **fields: Any) -> None:
        self.__dict__.update(fields)


class _Header(_Msg):
    """std_msgs/Header stand-in: pre-seeds a nested ``stamp`` so ``header.stamp.sec = ...`` works
    the way the real Header (a stamp sub-message + frame_id) does."""

    def __init__(self, **fields: Any) -> None:
        super().__init__(**fields)
        if not hasattr(self, "stamp"):
            self.stamp = SimpleNamespace(sec=0, nanosec=0)


class _QoSProfile:
    def __init__(self, **kw: Any) -> None:
        self.__dict__.update(kw)


class _FakePublisher:
    def __init__(self) -> None:
        self.published: list[Any] = []

    def publish(self, msg: Any) -> None:
        self.published.append(msg)


class _FakeClock:
    def now(self) -> SimpleNamespace:
        return SimpleNamespace(to_msg=lambda: SimpleNamespace(sec=7, nanosec=8))


class _RecordingLogger:
    def info(self, _msg: object) -> None: ...

    def warning(self, _msg: object, **_kw: Any) -> None: ...


class _FakeNode:
    """rclpy.node.Node stand-in: records params, publishers, and subscriptions."""

    params: ClassVar[dict[str, Any]] = {}

    def __init__(self, _name: str) -> None:
        self.clock = _FakeClock()
        self.logger = _RecordingLogger()
        self.subscriptions_made: list[tuple[Any, str, Any, Any]] = []
        self.pubs: dict[str, _FakePublisher] = {}

    def declare_parameter(self, name: str, default: Any) -> SimpleNamespace:
        return SimpleNamespace(value=self.params.get(name, default))

    def create_publisher(self, _msg_type: Any, topic: str, _qos: Any) -> _FakePublisher:
        pub = _FakePublisher()
        self.pubs[topic] = pub
        return pub

    def create_subscription(self, msg_type: Any, topic: str, cb: Any, qos: Any) -> None:
        self.subscriptions_made.append((msg_type, topic, cb, qos))

    def get_clock(self) -> _FakeClock:
        return self.clock

    def get_logger(self) -> _RecordingLogger:
        return self.logger


class _CvBridge:
    def __init__(self) -> None: ...

    def imgmsg_to_cv2(self, image_msg: Any, desired_encoding: str = "") -> Any:
        return SimpleNamespace(image=image_msg, encoding=desired_encoding)


def _stub_module(name: str, **attrs: Any) -> ModuleType:
    mod = ModuleType(name)
    for attr, value in attrs.items():
        setattr(mod, attr, value)
    return mod


@pytest.fixture
def node_mod(monkeypatch: pytest.MonkeyPatch) -> Iterator[ModuleType]:
    """Import patrol_perception.perception_node against stubbed ROS/cv2; restored on teardown."""
    _cv2_state["ok"] = True  # reset the encode-failure toggle between tests
    _FakeNode.params = {}  # reset node params between tests
    qos_enum = SimpleNamespace(
        RELIABLE=2, BEST_EFFORT=1, VOLATILE=2, TRANSIENT_LOCAL=1, KEEP_LAST=1
    )
    # imencode returns (ok, buf); buf.tobytes() -> the PNG bytes. Toggle ok via _cv2_state.
    cv2_stub = _stub_module(
        "cv2",
        imencode=lambda _ext, _frame: (
            _cv2_state["ok"],
            SimpleNamespace(tobytes=lambda: b"PNGBYTES"),
        ),
    )
    stubs = {
        "cv2": cv2_stub,
        "rclpy": _stub_module("rclpy", init=lambda **_k: None, spin=lambda _n: None),
        "rclpy.node": _stub_module("rclpy.node", Node=_FakeNode),
        "rclpy.qos": _stub_module(
            "rclpy.qos",
            QoSProfile=_QoSProfile,
            DurabilityPolicy=qos_enum,
            HistoryPolicy=qos_enum,
            ReliabilityPolicy=qos_enum,
            qos_profile_sensor_data=SimpleNamespace(name="sensor_data"),
        ),
        "apriltag_msgs": _stub_module("apriltag_msgs"),
        "apriltag_msgs.msg": _stub_module("apriltag_msgs.msg", AprilTagDetectionArray=_Msg),
        "cv_bridge": _stub_module("cv_bridge", CvBridge=_CvBridge),
        "diagnostic_msgs": _stub_module("diagnostic_msgs"),
        "diagnostic_msgs.msg": _stub_module("diagnostic_msgs.msg", KeyValue=_Msg),
        "geometry_msgs": _stub_module("geometry_msgs"),
        "geometry_msgs.msg": _stub_module(
            "geometry_msgs.msg", Point=_Msg, Pose=_Msg, PoseStamped=_Msg, Quaternion=_Msg
        ),
        "px4_msgs": _stub_module("px4_msgs"),
        "px4_msgs.msg": _stub_module("px4_msgs.msg", VehicleLocalPosition=_Msg),
        "sensor_msgs": _stub_module("sensor_msgs"),
        "sensor_msgs.msg": _stub_module("sensor_msgs.msg", Image=_Msg),
        "std_msgs": _stub_module("std_msgs"),
        "std_msgs.msg": _stub_module("std_msgs.msg", Header=_Header, Int32=_Msg),
        "patrol_interfaces": _stub_module("patrol_interfaces"),
        "patrol_interfaces.msg": _stub_module("patrol_interfaces.msg", CheckpointCapture=_Msg),
    }
    for name, mod in stubs.items():
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.delitem(sys.modules, "patrol_perception.perception_node", raising=False)
    module = importlib.import_module("patrol_perception.perception_node")
    yield module
    sys.modules.pop("patrol_perception.perception_node", None)


def _make_node(node_mod: ModuleType) -> Any:
    _FakeNode.params = {
        "camera_topic": "/drone/camera/image_raw",
        "checkpoint_config_path": str(CHECKPOINTS_YAML),
    }
    return node_mod.PerceptionNode()


# --- _RosCaptureMessageFactory: field mapping (the transposition risk) ----------


def test_factory_make_header_maps_stamp_and_frame(node_mod: ModuleType) -> None:
    factory = node_mod._RosCaptureMessageFactory()
    header = factory.make_header(11, 22, "patrol_world")
    assert (header.stamp.sec, header.stamp.nanosec) == (11, 22)
    assert header.frame_id == "patrol_world"


def test_factory_pose_stamped_maps_position_and_orientation_in_order(node_mod: ModuleType) -> None:
    # Guards against an x/y/z or quaternion transposition that every ROS-free builder test misses.
    factory = node_mod._RosCaptureMessageFactory()
    ps = factory.make_pose_stamped(1, 2, "w", (1.5, 2.5, 3.5), (0.1, 0.2, 0.3, 0.4))
    assert (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z) == (1.5, 2.5, 3.5)
    o = ps.pose.orientation
    assert (o.x, o.y, o.z, o.w) == (0.1, 0.2, 0.3, 0.4)
    assert ps.header.frame_id == "w"


def test_factory_key_value_maps_key_and_value(node_mod: ModuleType) -> None:
    kv = node_mod._RosCaptureMessageFactory().make_key_value("tag_id", "0")
    assert (kv.key, kv.value) == ("tag_id", "0")


# --- _required_param: fail-loud guard -------------------------------------------


def test_missing_required_camera_topic_raises(node_mod: ModuleType) -> None:
    _FakeNode.params = {"checkpoint_config_path": str(CHECKPOINTS_YAML)}  # camera_topic unset -> ""
    with pytest.raises(ValueError, match="camera_topic"):
        node_mod.PerceptionNode()


# --- _encode_image: cv2 encode seam + failure branch ----------------------------


def test_encode_image_returns_png_bytes(node_mod: ModuleType) -> None:
    node = _make_node(node_mod)
    assert node._encode_image(_Msg()) == b"PNGBYTES"


def test_encode_image_raises_when_imencode_fails(node_mod: ModuleType) -> None:
    node = _make_node(node_mod)
    _cv2_state["ok"] = False
    with pytest.raises(RuntimeError, match="imencode"):
        node._encode_image(_Msg())


# --- subscription wiring: topics + EKF-origin constant --------------------------


def test_subscribes_to_versioned_px4_local_position_topic(node_mod: ModuleType) -> None:
    # The PX4 topic string is a drift hazard (the v1.17 _v{N} versioning); pin it here so a regression
    # is caught per-PR instead of silently subscribing to a dead topic in SITL.
    node = _make_node(node_mod)
    topics = {topic for (_t, topic, _cb, _q) in node.subscriptions_made}
    assert "/fmu/out/vehicle_local_position_v1" in topics
    assert "/drone/camera/image_raw" in topics


def test_capture_publisher_created_on_checkpoint_capture_topic(node_mod: ModuleType) -> None:
    node = _make_node(node_mod)
    assert "/patrol/checkpoint_capture" in node.pubs


def test_ekf_origin_is_zero_in_sitl(node_mod: ModuleType) -> None:
    # VehicleLocalPosition is already EKF-origin-relative in this SITL setup (mission node agrees).
    assert node_mod._EKF_ORIGIN_NED == (0.0, 0.0, 0.0)
