"""The expected checkpoint-capture count for a full patrol (F-01 lock).

Layer-A: ROS-free, composes the REAL loaders so there is no second source of truth.

**Why this test exists.** A Phase 1 exit-checklist pass read a full-patrol bag as a defect: it
recorded ``/patrol/dwell`` 4 and ``/patrol/checkpoint_capture`` 3 and concluded perception had failed
to resolve a tag at a fourth checkpoint — a phantom ``cp_west`` that does not exist anywhere in this
repo. It was logged as a Medium defect ("reproducible across two runs a month apart"), and three
candidate root causes were proposed for a bug that was never there.

The route simply has **four waypoints and three checkpoints**. The fourth waypoint is an inline ENU
overlook carrying no ``checkpoint_id`` and standing near no AprilTag — deliberately kept to exercise
the inline-waypoint path alongside the checkpoint-reference path (see the header comment in
``patrol_mission.yaml``). Perception behaves correctly there: :class:`CaptureCoordinator` finds no
tag in view, logs the ADR-A gate skip, and does not latch. So the honest capture expectation is
**one per checkpoint-bearing waypoint (3), not one per dwell (4)** — a 100% capture rate, not 75%.

This pins that relationship across the four files that have to agree, so the same misreading costs
a failing test instead of an investigation.
"""

from __future__ import annotations

import re
from pathlib import Path

from patrol_mission.config import load_mission_config

_REPO_ROOT = Path(__file__).resolve().parents[2]
_PATROL_MISSION_YAML = _REPO_ROOT / "ros2_ws/src/patrol_bringup/config/patrol_mission.yaml"
_CHECKPOINTS_YAML = _REPO_ROOT / "sim/config/checkpoints.yaml"
_WORLD_SDF = _REPO_ROOT / "sim/worlds/patrol_world.sdf"
_MODELS_DIR = _REPO_ROOT / "sim/models"


def _patrol_config():
    """The real patrol route, resolved against the real canonical checkpoints file."""
    return load_mission_config(str(_PATROL_MISSION_YAML), str(_CHECKPOINTS_YAML))


def _checkpoint_waypoints():
    return [wp for wp in _patrol_config().waypoints if wp.checkpoint_id is not None]


def _inline_waypoints():
    return [wp for wp in _patrol_config().waypoints if wp.checkpoint_id is None]


def test_expected_captures_is_one_per_checkpoint_waypoint_not_one_per_dwell():
    # The exact arithmetic that was misread off the bag: dwells > captures is CORRECT here, because
    # /patrol/dwell fires once per waypoint while a capture needs a tag to resolve.
    waypoints = _patrol_config().waypoints
    checkpoint_wps = _checkpoint_waypoints()
    inline_wps = _inline_waypoints()

    assert len(checkpoint_wps) + len(inline_wps) == len(waypoints)
    assert inline_wps, (
        "the route no longer exercises the inline-waypoint path; if that was deliberate, the "
        "expected capture count now equals the dwell count and this test should say so"
    )
    # The witness assertion: expected captures == checkpoint waypoints, strictly fewer than dwells.
    assert len(checkpoint_wps) < len(waypoints)


def test_every_checkpoint_waypoint_resolves_to_a_configured_checkpoint():
    # A capture can only happen where a tag exists, so each checkpoint waypoint must name a real
    # entry in 03's canonical file. This is what makes the count above an achievable target rather
    # than an aspiration.
    configured = set(_load_checkpoint_ids())
    referenced = {wp.checkpoint_id for wp in _checkpoint_waypoints()}

    assert referenced <= configured, (
        f"route references unconfigured checkpoints: {referenced - configured}"
    )


def test_checkpoint_count_agrees_across_config_models_and_world():
    # The four files that must agree for the expected count to be real: the canonical checkpoints
    # file, the generated AprilTag model library, and the composed world's tag includes. A phantom
    # fourth checkpoint would have to show up in all of them.
    checkpoint_ids = _load_checkpoint_ids()
    model_dirs = sorted(p.name for p in _MODELS_DIR.glob("apriltag_36h11_*") if p.is_dir())
    world_includes = re.findall(r"apriltag_36h11_\d+", _WORLD_SDF.read_text(encoding="utf-8"))

    assert len(model_dirs) == len(checkpoint_ids)
    assert len(set(world_includes)) == len(checkpoint_ids)


def _load_checkpoint_ids() -> list[str]:
    """The configured checkpoint ids, read straight from 03's canonical file."""
    text = _CHECKPOINTS_YAML.read_text(encoding="utf-8")
    return re.findall(r"^\s*-\s*checkpoint_id:\s*\"?([A-Za-z0-9_]+)\"?", text, flags=re.MULTILINE)
