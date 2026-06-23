"""Layer-A unit tests for patrol_perception.checkpoint_config (M6.A, T A.2).

Covers TS-6 (schema validation + duplicate-tag_id rejection) and TS-7 (fail-fast on
missing/malformed source). ROS-free: imports the core directly, no rclpy/Gazebo/PX4 (AC-5).
The loader reads 03's canonical `sim/config/checkpoints.yaml` (settled-default schema,
OQ-5) into a {tag_id: CheckpointEntry} map; 04 consumes the relation read-only (design §4.2.5).
"""

import textwrap
from pathlib import Path

import pytest
from patrol_perception.checkpoint_config import (
    CheckpointConfigError,
    CheckpointConfigLoader,
    CheckpointEntry,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
CANONICAL = REPO_ROOT / "sim/config/checkpoints.yaml"

_VALID = """\
checkpoints:
  - checkpoint_id: "cp_a"
    position: { x: 1.0, y: 2.0, z: 3.0 }
    tag_family: "tag36h11"
    tag_id: 0
  - checkpoint_id: "cp_b"
    position: { x: -4.0, y: 5.5, z: 6.0 }
    tag_family: "tag36h11"
    tag_id: 1
"""

# (row body, id) pairs — each is one `checkpoints:` row that violates the schema (TS-6).
_INVALID_ROWS = {
    "missing-checkpoint_id": 'position: { x: 1.0, y: 2.0, z: 3.0 }\n    tag_family: "tag36h11"\n    tag_id: 0',
    "empty-checkpoint_id": 'checkpoint_id: ""\n    position: { x: 1.0, y: 2.0, z: 3.0 }\n    tag_family: "tag36h11"\n    tag_id: 0',
    "missing-tag_family": 'checkpoint_id: "cp_a"\n    position: { x: 1.0, y: 2.0, z: 3.0 }\n    tag_id: 0',
    "missing-tag_id": 'checkpoint_id: "cp_a"\n    position: { x: 1.0, y: 2.0, z: 3.0 }\n    tag_family: "tag36h11"',
    "non-int-tag_id": 'checkpoint_id: "cp_a"\n    position: { x: 1.0, y: 2.0, z: 3.0 }\n    tag_family: "tag36h11"\n    tag_id: "oops"',
    "missing-position": 'checkpoint_id: "cp_a"\n    tag_family: "tag36h11"\n    tag_id: 0',
    "incomplete-position": 'checkpoint_id: "cp_a"\n    position: { x: 1.0, y: 2.0 }\n    tag_family: "tag36h11"\n    tag_id: 0',
    "non-numeric-position": 'checkpoint_id: "cp_a"\n    position: { x: "a", y: 2.0, z: 3.0 }\n    tag_family: "tag36h11"\n    tag_id: 0',
}

# (source text, id) pairs — each is a whole-file source that should fail fast (TS-7).
_BAD_SOURCES = {
    "malformed-yaml": "checkpoints: [unclosed",
    "top-level-not-mapping": "- just\n- a\n- list\n",
    "missing-checkpoints-key": "other_key: 1\n",
    "checkpoints-not-list": "checkpoints: 42\n",
    "empty-checkpoints-list": "checkpoints: []\n",
}


def _write(tmp_path: Path, text: str) -> str:
    p = tmp_path / "checkpoints.yaml"
    p.write_text(textwrap.dedent(text))
    return str(p)


def test_loads_canonical_checkpoints_file():
    """TS-6: the shipped canonical checkpoints.yaml loads, keyed by tag_id."""
    result = CheckpointConfigLoader().load(str(CANONICAL))

    assert set(result) == {0, 1, 2}
    north = result[0]
    assert isinstance(north, CheckpointEntry)
    assert north.checkpoint_id == "cp_north"
    assert north.tag_id == 0
    assert north.tag_family == "tag36h11"
    assert north.position == (12.0, 8.0, 1.5)


def test_loads_valid_config(tmp_path):
    """TS-6: a well-formed two-row config parses into two entries keyed by tag_id."""
    result = CheckpointConfigLoader().load(_write(tmp_path, _VALID))

    assert set(result) == {0, 1}
    assert result[1].checkpoint_id == "cp_b"
    assert result[1].position == (-4.0, 5.5, 6.0)


def test_rejects_duplicate_tag_id(tmp_path):
    """TS-6: two rows sharing a tag_id break the map — reject loudly."""
    dup = """\
    checkpoints:
      - checkpoint_id: "cp_a"
        position: { x: 1.0, y: 2.0, z: 3.0 }
        tag_family: "tag36h11"
        tag_id: 0
      - checkpoint_id: "cp_b"
        position: { x: 4.0, y: 5.0, z: 6.0 }
        tag_family: "tag36h11"
        tag_id: 0
    """
    with pytest.raises(CheckpointConfigError, match="duplicate tag_id"):
        CheckpointConfigLoader().load(_write(tmp_path, dup))


@pytest.mark.parametrize("row", _INVALID_ROWS.values(), ids=_INVALID_ROWS.keys())
def test_rejects_invalid_rows(tmp_path, row):
    """TS-6: every required field is schema-validated; a malformed row fails loud."""
    text = "checkpoints:\n  - " + row + "\n"
    with pytest.raises(CheckpointConfigError):
        CheckpointConfigLoader().load(_write(tmp_path, text))


def test_missing_file_fails_fast(tmp_path):
    """TS-7: a non-existent config path fails fast (don't run blind on a broken map)."""
    with pytest.raises(CheckpointConfigError, match="not found"):
        CheckpointConfigLoader().load(str(tmp_path / "nope.yaml"))


@pytest.mark.parametrize("text", _BAD_SOURCES.values(), ids=_BAD_SOURCES.keys())
def test_fail_fast_on_bad_source(tmp_path, text):
    """TS-7: malformed/empty/wrong-shape sources fail fast at load."""
    with pytest.raises(CheckpointConfigError):
        CheckpointConfigLoader().load(_write(tmp_path, text))
