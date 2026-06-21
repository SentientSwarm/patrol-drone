#!/usr/bin/env python3
"""CI gate: the committed sim assets are in sync with their single source of truth (M5, INF-S2).

Two generated-asset classes, two checks — both fail the PR if a committed artifact drifted from the
config/generator it is produced from, the same contract the `manifest-drift` gate enforces for the
stack manifest:

  1. sim/worlds/patrol_world.sdf markers == sim/config/checkpoints.yaml positions/ids
     (someone edited checkpoints.yaml but did not re-run sim/tools/compose_world.py)
  2. sim/models/apriltag_36h11_<id>/ == sim/tools/gen_apriltag_models.py output
     (someone changed the generator/tag set but did not regenerate the model dirs)

Pure stdlib (the composer + generator are dependency-free), so it runs on the runner's system
`python3` with zero setup — no `uv sync`, no ROS. Exit 0 = in sync; exit 1 = drift, one line per
problem with the fix command.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "sim" / "tools"))

import compose_world  # noqa: E402  (path bootstrap above)
import gen_apriltag_models  # noqa: E402


def _world_problems() -> list[str]:
    try:
        return compose_world.check_drift() + compose_world.validate_world_design()
    except compose_world.ComposeError as exc:
        return [str(exc)]


def _model_problems() -> list[str]:
    stale = [
        f"stale generated model file: {rel}" for rel in gen_apriltag_models.stale_model_files()
    ]
    orphans = [
        f"orphan apriltag model dir (remove it; no canonical tag): {rel}"
        for rel in gen_apriltag_models.orphan_model_dirs()
    ]
    return stale + orphans


def run_checks() -> list[str]:
    problems = _world_problems()
    if problems:
        problems.append("fix: python3 sim/tools/compose_world.py")
    model_drift = _model_problems()
    if model_drift:
        model_drift.append("fix: python3 sim/tools/gen_apriltag_models.py")
    return problems + model_drift


def main() -> int:
    problems = run_checks()
    if problems:
        print("sim asset drift detected:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    print("sim assets: patrol_world.sdf + apriltag models in sync with their sources.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
