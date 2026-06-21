#!/usr/bin/env python3
"""Host-side M4 patrol acceptance verifier — prints PASS/FAIL against a LIVE SITL patrol.

Run AFTER ``scripts/run_sitl_mission.sh --patrol`` has the stack up (agent + PX4 SITL + the mission
node launched from ``mission_patrol.launch.py``). This launches NOTHING: it subscribes to the live
``/patrol/*`` surface + ``/fmu/out/vehicle_status``, watches the nominal patrol progression
(``arm -> visit every waypoint (dwell) -> RTH -> land/disarm``), prints each criterion PASS/FAIL,
and exits ``0`` (all passed) / ``1`` (any failed) — removing the human judgment call (AC-2).

The criteria are NOT defined here: they live in ``tests/integration/patrol_acceptance.py``, shared
verbatim with the nightly SITL test (``tests/integration/test_mission_patrol.py``) so the host path
and CI can't drift.

The external-abort half (AC-6) is exercised automatically by the nightly test; on the host it is a
manual step in the runbook (``docs/uat/m4.md``): publish ``/patrol/abort`` mid-patrol and watch
``/patrol/mission_state`` go ABORT -> RTH.

Must run in a shell with BOTH sourced (``run_sitl_mission.sh`` does this for you)::

    source /opt/ros/jazzy/setup.bash
    source <repo>/ros2_ws/install/setup.bash
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import rclpy

# The shared acceptance criteria live under tests/integration (so they ship to the nightly container
# via `docker cp tests`). That dir isn't importable from scripts/ by default, so bootstrap the path.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tests" / "integration"))

from mission_acceptance import Check
from patrol_acceptance import (
    PatrolWatcher,
    evaluate_nominal,
    expected_waypoint_count,
    spin_until,
)


def _report(checks: list[Check]) -> int:
    passed = all(c.passed for c in checks)
    print("\n=== M4 patrol-mission acceptance (nominal, AC-2) ===")
    for c in checks:
        print(f"  [{'PASS' if c.passed else 'FAIL'}] {c.name}: {c.detail}")
    print(f"=== overall: {'PASS' if passed else 'FAIL'} ===")
    return 0 if passed else 1


def verify(timeout_s: float) -> int:
    rclpy.init()
    expected = expected_waypoint_count()
    watcher = PatrolWatcher(expected)
    try:
        spin_until(watcher, lambda w: w.nominal_complete, timeout_s=timeout_s)
        return _report(evaluate_nominal(watcher, expected))
    finally:
        watcher.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Verify the live M4 nominal patrol (arm -> visit all waypoints -> RTH -> land) and "
            "print PASS/FAIL. Run after scripts/run_sitl_mission.sh --patrol has the stack up, in a "
            "shell with /opt/ros/jazzy and ros2_ws/install both sourced."
        )
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=300.0,
        help="seconds to watch before giving up (default: 300, patrol_acceptance.PATROL_TIMEOUT_S)",
    )
    args = parser.parse_args()
    return verify(args.timeout)


if __name__ == "__main__":
    sys.exit(main())
