"""Pytest wrapper that runs the bash unit test for run_patrol_world_sitl.sh::finalize_bag_sidecars.

The behavior under test is in shell (the sidecar-finalize OUTCOME gate: a finalized bag left
without its ``<bag>.meta.json`` must fail the finalize pass, not warn-and-succeed — PR #16, Mira
Medium, review 4728294643), so the assertions live in tests/unit/test_finalize_sidecars_outcome.sh;
this wrapper just runs it under `uv run pytest` / CI and surfaces its output on failure. Keeps the
ROS-free, fast-unit convention (CLAUDE.md).
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parent / "test_finalize_sidecars_outcome.sh"


@pytest.mark.skipif(
    shutil.which("bash") is None, reason="bash required for the finalize_bag_sidecars test"
)
def test_finalize_bag_sidecars_outcome_gate():
    result = subprocess.run(
        ["bash", str(_SCRIPT)],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, f"{result.stdout}\n{result.stderr}"
    assert "PASS:" in result.stdout
