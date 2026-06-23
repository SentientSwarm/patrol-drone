"""Layer-A composition smoke test for patrol_world.launch.py (M5 F-02; no SITL).

patrol_world.launch.py composes the ROS side of the M5 stage by including camera_bridge.launch.py +
mission_patrol.launch.py and forwarding checkpoints_yaml/record. The nightly SITL runner
(scripts/run_patrol_world_sitl.sh) launches those two children DIRECTLY, so the composition file's
include wiring and argument forwarding are never executed by anything that bring-up tests — a wrong
include path or a dropped/renamed forwarded arg would ship undetected.

This constructs the LaunchDescription and asserts the composition WITHOUT spinning SITL: the two
includes are present and resolve, and the mission_patrol include forwards both checkpoints_yaml and
record. It needs the `launch`/`launch_ros`/`ament_index_python` packages (ROS-sourced only), so it is
marked `ros` and lives in the integration tier — but it is fast (pure description introspection), not
a SITL test.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

pytestmark = pytest.mark.ros

_CHILD_LAUNCH_FILES = {"camera_bridge.launch.py", "mission_patrol.launch.py"}
_FORWARDED_ARGS = {"checkpoints_yaml", "record"}


def _world_launch_description() -> LaunchDescription:
    """Import the SHIPPED patrol_world.launch.py and build its LaunchDescription (no side effects)."""
    share = get_package_share_directory("patrol_bringup")
    path = Path(share) / "launch" / "patrol_world.launch.py"
    spec = importlib.util.spec_from_file_location("patrol_world_launch", path)
    assert spec is not None, f"cannot load {path}"
    assert spec.loader is not None, f"no loader for {path}"
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _includes(ld: LaunchDescription) -> list[IncludeLaunchDescription]:
    return [e for e in ld.entities if isinstance(e, IncludeLaunchDescription)]


def _arg_keys(inc: IncludeLaunchDescription) -> set[str]:
    # IncludeLaunchDescription.launch_arguments is the (key, value) tuple passed in; our keys are
    # plain strings (see patrol_world.launch.py `_include`).
    return {key for key, _ in inc.launch_arguments}


def test_patrol_world_declares_record_and_checkpoints_args():
    ld = _world_launch_description()
    declared = {e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)}
    assert declared >= _FORWARDED_ARGS


def test_patrol_world_forwards_args_to_exactly_one_child():
    ld = _world_launch_description()
    forwarding = [inc for inc in _includes(ld) if _arg_keys(inc)]
    assert len(forwarding) == 1, "exactly one include (mission_patrol) forwards arguments"
    assert _arg_keys(forwarding[0]) == _FORWARDED_ARGS


def test_patrol_world_includes_resolve_to_both_children():
    ld = _world_launch_description()
    ctx = LaunchContext()
    names = set()
    for inc in _includes(ld):
        inc.launch_description_source.get_launch_description(
            ctx
        )  # raises if the include can't resolve
        names.add(Path(inc.launch_description_source.location).name)
    assert names == _CHILD_LAUNCH_FILES
