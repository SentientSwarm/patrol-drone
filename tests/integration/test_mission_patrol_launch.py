"""Layer-A ordering probe for mission_patrol.launch.py (M7 F-02; no SITL).

The blocking Hermes review (Medium) flagged that mission_patrol.launch.py listed the mission Node
BEFORE the recorder/run-id OpaqueFunctions, so with record:=true the default the patrol process could
start publishing (mission_state/offboard/takeoff) before `ros2 bag record` had subscribed — the
earliest samples racing ahead of recording. The fix reorders the LaunchDescription so _resolve_run_id
+ _maybe_perception + _maybe_record come BEFORE mission_node; this probe pins that order so a
regression that re-lists mission_node first is caught.

CAVEAT (mirrors the source comment): launch process startup is asynchronous, so list order only
NARROWS the race window, it does not fully close it — the complete fix is a hard OnProcessStart
readiness gate (a tracked follow-up). This probe asserts the order contract that is the agreed
minimum, not zero-race.

Like test_patrol_world_launch.py this introspects the SHIPPED launch description WITHOUT spinning
SITL. It needs `launch`/`launch_ros`/`ament_index_python` (ROS-sourced only), so it is marked `ros`
and lives in the integration tier — but it is fast (pure description introspection), not a SITL test.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import ModuleType

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

pytestmark = pytest.mark.ros


def _mission_patrol_module() -> ModuleType:
    """Load the SHIPPED mission_patrol.launch.py module (so the function identities match the LD)."""
    share = get_package_share_directory("patrol_bringup")
    path = Path(share) / "launch" / "mission_patrol.launch.py"
    spec = importlib.util.spec_from_file_location("mission_patrol_launch", path)
    assert spec is not None, f"cannot load {path}"
    assert spec.loader is not None, f"no loader for {path}"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _index_of_opaque(ld: LaunchDescription, func: object) -> int:
    """List index of the OpaqueFunction wrapping `func` (its module-level callable)."""
    for i, entity in enumerate(ld.entities):
        if isinstance(entity, OpaqueFunction) and entity._OpaqueFunction__function is func:
            return i
    raise AssertionError(f"no OpaqueFunction wrapping {getattr(func, '__name__', func)} in the LD")


def _index_of_mission_node(ld: LaunchDescription) -> int:
    """List index of the single mission Node action."""
    nodes = [i for i, e in enumerate(ld.entities) if isinstance(e, Node)]
    assert len(nodes) == 1, f"expected exactly one Node in the LD, found {len(nodes)}"
    return nodes[0]


def test_recorder_and_run_id_are_ordered_before_the_mission_node() -> None:
    # F-02: the recorder/run-id setup must precede the mission node so `ros2 bag record` is
    # subscribing before patrol_mission starts publishing (narrows the start-up race).
    module = _mission_patrol_module()
    ld = module.generate_launch_description()

    mission_idx = _index_of_mission_node(ld)
    run_id_idx = _index_of_opaque(ld, module._resolve_run_id)
    record_idx = _index_of_opaque(ld, module._maybe_record)

    assert run_id_idx < mission_idx, "_resolve_run_id must be listed before the mission node"
    assert record_idx < mission_idx, "_maybe_record must be listed before the mission node"
    # _resolve_run_id stays first so _maybe_record reads the id it stashes on the context.
    assert run_id_idx < record_idx, "_resolve_run_id must precede _maybe_record"
