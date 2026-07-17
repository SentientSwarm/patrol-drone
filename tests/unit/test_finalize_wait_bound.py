"""Pytest wrapper that runs the bash unit test for run_patrol_world_sitl.sh::require_finalize_wait.

The behavior under test is in shell (the FINALIZE_WAIT lower bound: reject any override <= the
recorder's SIGTERM timeout, scraped from record.launch.py — PR #16 / F-03, Mira Medium), so the
assertions live in tests/unit/test_finalize_wait_bound.sh; this wrapper just runs it under
`uv run pytest` / CI and surfaces its output on failure. Keeps the ROS-free, fast-unit convention
(CLAUDE.md), mirroring test_stop_launch_group.py.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_finalize_wait_bound.sh"


@pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash required for the require_finalize_wait test"
)
@pytest.mark.skipif(
    shutil.which("grep") is None,
    reason="grep required to scrape the recorder sigterm timeout",
)
def test_require_finalize_wait_enforces_lower_bound():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
