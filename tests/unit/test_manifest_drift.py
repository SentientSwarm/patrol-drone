"""Unit tests for scripts/check_manifest_drift.py and a live-repo no-drift regression guard.

This is the repo's first unit suite; it runs ROS-free (London-style, no rclpy) per CLAUDE.md.
The module is loaded by path because scripts/ is intentionally not a Python package.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "check_manifest_drift.py"


def _load_drift() -> Any:
    spec = importlib.util.spec_from_file_location("check_manifest_drift", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


drift = _load_drift()


def test_px4_release_line_extracts_major_minor():
    assert drift.px4_release_line("v1.17.0") == "1.17"
    assert drift.px4_release_line("1.16.2") == "1.16"


def test_alignment_passes_when_branch_matches_version():
    manifest = {"flight_stack": {"px4_version": "v1.17.0", "px4_msgs_ref": "release/1.17"}}
    assert drift.check_px4_msgs_alignment(manifest) == []


def test_alignment_flags_off_release_line_branch():
    manifest = {"flight_stack": {"px4_version": "v1.17.0", "px4_msgs_ref": "release/1.16"}}
    problems = drift.check_px4_msgs_alignment(manifest)
    assert len(problems) == 1
    assert "off px4_version's release line" in problems[0]


def test_live_repo_has_no_manifest_drift():
    assert drift.run_checks(REPO_ROOT) == []
