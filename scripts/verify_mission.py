#!/usr/bin/env python3
"""Host-side M3 acceptance verifier — prints PASS/FAIL against a LIVE SITL mission.

Run AFTER ``scripts/run_sitl_mission.sh`` has the stack up (agent + PX4 SITL + the mission node).
This launches NOTHING: it subscribes to the live ``/fmu/out/*`` telemetry, watches the
``arm -> offboard -> settle@5 m -> land/disarm`` progression, prints each criterion PASS/FAIL, and
exits ``0`` (all passed) or ``1`` (any failed) — removing the human judgment call.

The acceptance criteria are NOT defined here: they live in ``tests/integration/mission_acceptance.py``,
shared verbatim with the nightly SITL test (``tests/integration/test_mission_basic.py``) so the host
path and CI can't drift.

Must run in a shell with BOTH sourced (``run_sitl_mission.sh`` does this for you), or the px4_msgs
import fails::

    source /opt/ros/jazzy/setup.bash
    source <repo>/ros2_ws/install/setup.bash
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy

# The shared acceptance criteria live under tests/integration (so they ship to the nightly container
# via `docker cp tests`). That dir isn't importable from scripts/ by default, so bootstrap the path
# before importing the module.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests" / "integration"))

from mission_acceptance import (
    Check,
    MissionAcceptanceWatcher,
    evaluate,
    load_thresholds,
    spin_until_complete,
)


def _report(checks: list[Check]) -> int:
    passed = all(c.passed for c in checks)
    print("\n=== M3 basic-mission acceptance ===")
    for c in checks:
        print(f"  [{'PASS' if c.passed else 'FAIL'}] {c.name}: {c.detail}")
    print(f"=== overall: {'PASS' if passed else 'FAIL'} ===")
    return 0 if passed else 1


def verify(timeout_s: float | None) -> int:
    rclpy.init()
    thresholds = load_thresholds()
    watcher = MissionAcceptanceWatcher(thresholds)
    try:
        spin_until_complete(watcher, thresholds, timeout_s=timeout_s)
        return _report(evaluate(watcher, thresholds))
    finally:
        watcher.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Verify the live M3 basic mission (arm -> offboard -> settle@5 m -> land) and print "
            "PASS/FAIL. Run after scripts/run_sitl_mission.sh has the stack up, in a shell with "
            "/opt/ros/jazzy and ros2_ws/install both sourced."
        )
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="seconds to watch before giving up (default: mission_acceptance.MISSION_TIMEOUT_S)",
    )
    args = parser.parse_args()
    return verify(args.timeout)


if __name__ == "__main__":
    sys.exit(main())
