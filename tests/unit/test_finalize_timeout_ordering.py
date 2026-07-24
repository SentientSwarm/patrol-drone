"""Contract test locking the record-path finalize timeout ordering (PR #16 / F-03, Hermes Low).

The runner's bounded wait for a clean recorder finalize (`FINALIZE_WAIT`, default in
run_patrol_world_sitl.sh) MUST be strictly larger than the launch system's recorder sigterm timeout
(`_RECORDER_SIGTERM_TIMEOUT_S` in record.launch.py). If the runner escalates to a group SIGTERM
before the launch's clean SIGINT-mediated finalize completes, the OnProcessExit sidecar handler is
torn down and `<bag>.meta.json` is lost — the very bug ADR-0013's follow-up closed. The two literals
live in different files (one shell, one Python) and drifted once already (ADR quoted 45 s/30 s while
code had moved to 90 s/60 s); this test asserts the ordering directly so they can't silently invert.

ROS-free: parses both literals out of the source files by their stable constant names — no ROS
import, no shell exec — matching the fast Layer-A unit convention (CLAUDE.md).
"""

from __future__ import annotations

import re
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_LAUNCH = _REPO_ROOT / "ros2_ws" / "src" / "patrol_logging" / "launch" / "record.launch.py"
_RUNNER = _REPO_ROOT / "scripts" / "run_patrol_world_sitl.sh"


def _first_int(pattern: str, text: str, source: str) -> int:
    match = re.search(pattern, text)
    assert match is not None, f"could not find /{pattern}/ in {source} (constant renamed?)"
    return int(match.group(1))


def test_finalize_wait_strictly_exceeds_recorder_sigterm_timeout():
    sigterm_timeout = _first_int(
        r'_RECORDER_SIGTERM_TIMEOUT_S\s*=\s*"(\d+)"',
        _LAUNCH.read_text(),
        "record.launch.py",
    )
    finalize_wait = _first_int(
        r"FINALIZE_WAIT:-(\d+)",
        _RUNNER.read_text(),
        "run_patrol_world_sitl.sh",
    )
    assert finalize_wait > sigterm_timeout, (
        f"runner FINALIZE_WAIT default ({finalize_wait}s) must be strictly larger than the launch "
        f"recorder sigterm timeout ({sigterm_timeout}s), or a group SIGTERM escalates before the "
        "clean SIGINT finalize completes and the bag sidecar is lost (ADR-0013)"
    )
