"""Pytest wrapper that runs the bash unit test for setup_phase1.sh::install_xrce_agent.

The behavior under test is in shell (the bootstrap script), so the assertions live in
tests/unit/test_setup_xrce.sh; this wrapper just runs it under `uv run pytest` / CI and surfaces
its output on failure. Keeps the ROS-free, fast-unit convention (CLAUDE.md).
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_setup_xrce.sh"


@pytest.mark.skipif(shutil.which("bash") is None, reason="bash required for the setup-script test")
def test_install_xrce_agent_fatal_by_default():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
