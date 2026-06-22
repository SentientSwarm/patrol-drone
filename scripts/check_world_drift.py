#!/usr/bin/env python3
"""CI gate: the committed sim assets are in sync with their single source of truth (M5, INF-S2).

Two generated-asset classes plus a cross-surface contract — three checks, each failing the PR when a
committed artifact drifted from the config/generator it is produced from (1, 2) or a shared literal
diverged across the surfaces that must agree (3), the same contract the `manifest-drift` gate enforces
for the stack manifest:

  1. sim/worlds/patrol_world.sdf markers == sim/config/checkpoints.yaml positions/ids
     (someone edited checkpoints.yaml but did not re-run sim/tools/compose_world.py)
  2. sim/models/apriltag_36h11_<id>/ == sim/tools/gen_apriltag_models.py output
     (someone changed the generator/tag set but did not regenerate the model dirs)
  3. the /drone/camera/image_raw 04/05 contract is identical across the SDF <topic>, the camera
     bridge launch default, and the SITL runner constant (no generator sits between them, so a
     one-sided edit would otherwise ship a silent three-way divergence M6/M7 hard-depend on)

Pure stdlib (the composer + generator are dependency-free), so it runs on the runner's system
`python3` with zero setup — no `uv sync`, no ROS. Exit 0 = in sync; exit 1 = drift, one line per
problem with the fix command.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "sim" / "tools"))

import compose_world  # noqa: E402  (path bootstrap above)
import gen_apriltag_models  # noqa: E402

# Camera image topic — the 04/05 contract (design §4.4.4) literal'd in three executable surfaces with
# no generator between them, so the world-drift gate parses none of them (F-01). A one-sided edit would
# ship a silent three-way divergence M6/M7 hard-depend on. The anchored patterns match only the
# executable literal in each file (the SDF has exactly one <topic>; CAMERA_IMAGE_TOPIC first occurs at
# its assignment; ^CAMERA_TOPIC= is unique), so the comment-line mentions of the string are ignored.
_CAMERA_SDF = REPO_ROOT / "sim" / "px4_sitl_overrides" / "gz_x500_patrol" / "model.sdf"
_CAMERA_LAUNCH = (
    REPO_ROOT / "ros2_ws" / "src" / "patrol_bringup" / "launch" / "camera_bridge.launch.py"
)
_CAMERA_RUNNER = REPO_ROOT / "scripts" / "run_patrol_world_sitl.sh"
_CAMERA_TOPIC_SURFACES: tuple[tuple[Path, re.Pattern[str]], ...] = (
    (_CAMERA_SDF, re.compile(r"<topic>\s*(\S+?)\s*</topic>")),
    (_CAMERA_LAUNCH, re.compile(r"""CAMERA_IMAGE_TOPIC\s*=\s*\(?\s*["'](\S+?)["']""")),
    (_CAMERA_RUNNER, re.compile(r'^CAMERA_TOPIC="(\S+?)"', re.MULTILINE)),
)


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


def _camera_topic_problems(
    surfaces: tuple[tuple[Path, re.Pattern[str]], ...] = _CAMERA_TOPIC_SURFACES,
) -> list[str]:
    topics: dict[str, str] = {}
    for path, pattern in surfaces:
        rel = str(path.relative_to(REPO_ROOT))
        match = pattern.search(path.read_text())
        if match is None:
            return [f"camera topic literal not found in {rel} (pattern {pattern.pattern!r})"]
        topics[rel] = match.group(1)
    if len(set(topics.values())) > 1:
        listing = ", ".join(f"{rel}={topic}" for rel, topic in sorted(topics.items()))
        return [
            f"camera topic drift — the 04/05 contract must match across all surfaces: {listing}"
        ]
    return []


def _annotate(problems: list[str], fix: str) -> list[str]:
    """Append a one-line fix hint iff there were problems (keeps run_checks flat — no repeated ifs)."""
    return [*problems, fix] if problems else problems


def run_checks() -> list[str]:
    return (
        _annotate(_world_problems(), "fix: python3 sim/tools/compose_world.py")
        + _annotate(_model_problems(), "fix: python3 sim/tools/gen_apriltag_models.py")
        + _annotate(
            _camera_topic_problems(),
            "fix: align the camera <topic> / CAMERA_IMAGE_TOPIC / CAMERA_TOPIC literals by hand",
        )
    )


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
