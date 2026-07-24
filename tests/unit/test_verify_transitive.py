"""Pytest wrapper that runs the bash unit test for build_xrce_agent.sh::verify_transitive (F-05).

The behavior under test is in shell (the agent build recipe's post-build transitive-pin guard), so the
assertions live in tests/unit/test_verify_transitive.sh; this wrapper just runs it under `uv run
pytest` / CI and surfaces its output on failure. Keeps the ROS-free, fast-unit convention (CLAUDE.md).
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_verify_transitive.sh"


@pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash required for the verify_transitive test"
)
@pytest.mark.skipif(
    shutil.which("git") is None, reason="git required to build the fixture checkouts"
)
def test_verify_transitive_checks_every_checkout():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
