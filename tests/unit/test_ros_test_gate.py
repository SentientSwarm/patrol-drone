"""Unit tests for scripts/check_ros_test_gate.py (Hermes Medium #2 footgun guard).

The guard fails iff ros-ci.yml still skips tests AND a first-party package has a test surface.
These tests pin that truth table and the vendored-external/ exemption with temp trees — ROS-free,
fast (CLAUDE.md unit convention).
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

_MODULE_PATH = Path(__file__).resolve().parents[2] / "scripts" / "check_ros_test_gate.py"
_spec = importlib.util.spec_from_file_location("check_ros_test_gate", _MODULE_PATH)
assert _spec is not None
assert _spec.loader is not None
gate = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gate)


def _make_pkg(root: Path, name: str, *, test_files: list[str] | None = None) -> Path:
    pkg = root / "ros2_ws" / "src" / name
    pkg.mkdir(parents=True)
    (pkg / "package.xml").write_text("<package/>\n")
    for rel in test_files or []:
        f = pkg / rel
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text("def test_x():\n    assert True\n")
    return pkg


def _write_workflow(root: Path, *, skip: bool) -> None:
    wf = root / ".github" / "workflows" / "ros-ci.yml"
    wf.parent.mkdir(parents=True)
    wf.write_text(f"          skip-tests: '{'true' if skip else 'false'}'\n")


def test_skeleton_package_without_tests_is_clean(tmp_path: Path):
    _make_pkg(tmp_path, "patrol_bringup")
    assert gate.first_party_packages_with_tests(tmp_path) == []


def test_first_party_test_dir_is_detected(tmp_path: Path):
    _make_pkg(tmp_path, "patrol_mission", test_files=["test/test_state_machine.py"])
    assert gate.first_party_packages_with_tests(tmp_path) == ["ros2_ws/src/patrol_mission"]


def test_unconventionally_named_test_under_tests_dir_is_detected(tmp_path: Path):
    """Hermes High #1 probe: a test/ or tests/ file that matches no test_*/*_test convention
    (e.g. an ament/CMake source `tests/state_machine.cpp`) must still count as a test surface."""
    _make_pkg(tmp_path, "patrol_mission", test_files=["tests/state_machine.cpp"])
    assert gate.first_party_packages_with_tests(tmp_path) == ["ros2_ws/src/patrol_mission"]


def test_vendored_external_tests_are_ignored(tmp_path: Path):
    _make_pkg(tmp_path, "external/px4_ros_com", test_files=["test/test_demo.py"])
    assert gate.first_party_packages_with_tests(tmp_path) == []


def test_non_test_python_is_not_a_test_surface(tmp_path: Path):
    _make_pkg(tmp_path, "patrol_perception", test_files=["patrol_perception/node.py"])
    assert gate.first_party_packages_with_tests(tmp_path) == []


def test_skip_tests_true_is_detected(tmp_path: Path):
    _write_workflow(tmp_path, skip=True)
    assert gate.ros_ci_skips_tests(tmp_path) is True


def test_skip_tests_false_disarms_guard(tmp_path: Path):
    _write_workflow(tmp_path, skip=False)
    assert gate.ros_ci_skips_tests(tmp_path) is False


def test_missing_workflow_disarms_guard(tmp_path: Path):
    assert gate.ros_ci_skips_tests(tmp_path) is False


def test_real_repo_is_currently_green():
    """Today the repo has skeleton-only first-party packages, so the guard must pass."""
    assert gate.main() == 0
