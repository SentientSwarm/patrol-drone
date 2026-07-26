"""Pytest wrapper that runs the bash unit test for run_patrol_world_sitl.sh::report_keep_up.

The behavior under test is in shell (the --keep-up teardown hint must never interpolate a `:-0`
default into a signal target, because `kill -INT -- -0` targets the caller's own process group —
F-11), so the assertions live in tests/unit/test_keep_up_teardown_hint.sh; this wrapper just runs it
under `uv run pytest` / CI and surfaces its output on failure. Keeps the ROS-free, fast-unit
convention (CLAUDE.md), mirroring test_stop_launch_group.py / test_finalize_wait_bound.py.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_keep_up_teardown_hint.sh"


@pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash required for the report_keep_up hint test"
)
@pytest.mark.skipif(
    shutil.which("grep") is None, reason="grep required to assert on the emitted hint text"
)
def test_keep_up_hint_never_targets_the_callers_process_group():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
