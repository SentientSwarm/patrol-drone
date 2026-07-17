"""Pytest wrapper that runs the bash unit test for run_patrol_world_sitl.sh::wait_for_pgroup_exit.

The behavior under test is in shell (the record-path finalize wait: poll the whole process GROUP, not
just the `ros2 launch` leader — PR #16 / F-01, Hermes High), so the assertions live in
tests/unit/test_stop_launch_group.sh; this wrapper just runs it under `uv run pytest` / CI and
surfaces its output on failure. Keeps the ROS-free, fast-unit convention (CLAUDE.md).
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_stop_launch_group.sh"


@pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash required for the wait_for_pgroup_exit test"
)
@pytest.mark.skipif(
    shutil.which("pgrep") is None,
    reason="pgrep (procps-ng) required to probe the process group",
)
def test_wait_for_pgroup_exit_waits_on_the_group():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
