"""Unit tests for the camera-topic drift check in scripts/check_world_drift.py (F-01).

ROS-free and dependency-free (regex-against-text, like test_ci_compose_override.py /
test_manifest_drift.py). The module is loaded by path because scripts/ is intentionally not a Python
package.

Pins the one un-gated single-source-of-truth surface the rest of the PR closed: ``/drone/camera/image_raw``
(the 04/05 contract) is literal'd independently in the SDF ``<topic>``, the camera bridge launch
default, and the SITL runner constant, with no generator between them. ``_camera_topic_problems``
fails the world-drift gate if the three diverge; ``test_real_surfaces_agree`` is the early-warning
for any future regex rot in those three patterns.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "check_world_drift.py"


def _load_gate() -> Any:
    spec = importlib.util.spec_from_file_location("check_world_drift", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


cwd = _load_gate()


@pytest.fixture
def repo_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    # _camera_topic_problems reports paths relative to REPO_ROOT, so the temp surfaces must live under
    # a REPO_ROOT the function agrees with; point it at tmp_path.
    monkeypatch.setattr(cwd, "REPO_ROOT", tmp_path)
    return tmp_path


def _write_surfaces(
    root: Path, sdf_topic: str, launch_topic: str, runner_topic: str
) -> tuple[tuple[Path, Any], ...]:
    """Write the three surfaces in their real syntaxes, paired with the production patterns."""
    patterns = [pattern for _, pattern in cwd._CAMERA_TOPIC_SURFACES]
    sdf = root / "model.sdf"
    sdf.write_text(f"<sensor>\n  <topic>{sdf_topic}</topic>\n</sensor>\n")
    launch = root / "camera_bridge.launch.py"
    launch.write_text(f'CAMERA_IMAGE_TOPIC = (\n    "{launch_topic}"\n)\n')
    runner = root / "run_patrol_world_sitl.sh"
    runner.write_text(f'CAMERA_TOPIC="{runner_topic}"\n')
    return tuple(zip([sdf, launch, runner], patterns, strict=True))


def test_real_surfaces_agree() -> None:
    # The committed SDF / launch / runner literals must already agree (pins the live contract and
    # catches a pattern that stops matching after a stylistic reformat of any of the three lines).
    assert cwd._camera_topic_problems() == []


# The canonical topic vs a single diverged surface — naming the odd-one-out keeps each row readable
# and avoids repeating the long literal across the table. Each case is one (sdf, launch, runner,
# expected-problem-count) tuple so the test takes a single parameter (see test_detects_divergence).
_CANON = "/drone/camera/image_raw"
_DRIFT = "/drone/camera/drifted"


@pytest.mark.parametrize(
    "case",
    [
        (_CANON, _CANON, _CANON, 0),  # all three agree -> no problem
        (_DRIFT, _CANON, _CANON, 1),  # SDF diverged
        (_CANON, _DRIFT, _CANON, 1),  # launch diverged
        (_CANON, _CANON, _DRIFT, 1),  # runner diverged
    ],
)
def test_detects_divergence(repo_root: Path, case: tuple[str, str, str, int]) -> None:
    sdf, launch, runner, expected = case
    problems = cwd._camera_topic_problems(_write_surfaces(repo_root, sdf, launch, runner))
    assert len(problems) == expected
    if expected:
        assert "camera topic drift" in problems[0]


def test_missing_literal_is_loud(repo_root: Path) -> None:
    bad = repo_root / "model.sdf"
    bad.write_text("<sdf>no topic here</sdf>\n")
    sdf_pattern = cwd._CAMERA_TOPIC_SURFACES[0][1]
    problems = cwd._camera_topic_problems(((bad, sdf_pattern),))
    assert len(problems) == 1
    assert "camera topic literal not found" in problems[0]
