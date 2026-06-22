"""Unit tests for the M5 AprilTag model generator (sim/tools/gen_apriltag_models.py).

Layer-A: pure stdlib, deterministic. Guards the hardware-parity contract (SIM-7) — model dir naming,
the contiguous tag-id set, a valid 8-bit grayscale PNG, and that the committed model library matches
the generator (no drift).
"""

from __future__ import annotations

import struct

import pytest

import gen_apriltag_models as gm

_PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


def test_committed_models_match_generator():
    assert gm.stale_model_files() == [], "run: python3 sim/tools/gen_apriltag_models.py"


@pytest.fixture
def patched_models(tmp_path, monkeypatch):
    """An empty MODELS_DIR under a tmp REPO_ROOT — shared scaffolding for the orphan-dir tests."""
    models = tmp_path / "sim" / "models"
    models.mkdir(parents=True)
    monkeypatch.setattr(gm, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(gm, "MODELS_DIR", models)
    return models


def test_orphan_model_dirs_reports_injected_dir(patched_models):
    (patched_models / "apriltag_36h11_0").mkdir()  # canonical id -> not an orphan
    (patched_models / "apriltag_36h11_99").mkdir()  # orphan -> reported
    assert gm.orphan_model_dirs() == ["sim/models/apriltag_36h11_99"]


def test_no_orphan_model_dirs_in_committed_tree():
    # Real MODELS_DIR / REPO_ROOT — locks the current tree clean (mirrors
    # test_committed_models_match_generator). No monkeypatch, so relative_to(REPO_ROOT) resolves.
    assert gm.orphan_model_dirs() == []


def test_check_flags_orphan_dir(patched_models, capsys):
    # --check must run the SAME union the CI wrapper does (stale + orphan). An orphan alone must fail
    # it; assert the orphan LINE is printed so the test fails if the new orphan branch ever regresses.
    (patched_models / "apriltag_36h11_99").mkdir()
    assert gm.main(["--check"]) == 1
    assert "orphan dir" in capsys.readouterr().out


def test_check_clean_when_in_sync():
    # Real REPO_ROOT / MODELS_DIR (no monkeypatch): the committed tree has neither stale nor orphan.
    assert gm.main(["--check"]) == 0


@pytest.mark.parametrize("tag_id", sorted(gm.CANONICAL_TAG36H11))
def test_model_dir_naming_is_the_contract(tag_id):
    # The dir / model:// name is apriltag_36h11_<id> (the composer emits model://apriltag_36h11_<id>).
    assert gm._model_name(tag_id) == f"apriltag_36h11_{tag_id}"
    assert gm.model_dir(tag_id).name == f"apriltag_36h11_{tag_id}"


def test_tag_ids_are_contiguous_from_zero():
    ids = sorted(gm.CANONICAL_TAG36H11)
    assert ids == list(range(len(ids)))  # 0..N-1 (SIM-7)


@pytest.mark.parametrize("tag_id", sorted(gm.CANONICAL_TAG36H11))
def test_render_png_is_valid_square_grayscale(tag_id):
    png = gm.render_png(gm.CANONICAL_TAG36H11[tag_id])
    assert png.startswith(_PNG_SIGNATURE)
    width, height, bit_depth, colour_type = struct.unpack(">IIBB", png[16:26])
    assert width == height == len(gm.CANONICAL_TAG36H11[tag_id]) * gm.CELL_PX
    assert (bit_depth, colour_type) == (8, 0)  # 8-bit grayscale


def test_render_png_is_deterministic():
    grid = gm.CANONICAL_TAG36H11[0]
    assert gm.render_png(grid) == gm.render_png(grid)


@pytest.mark.parametrize(
    "bad_grid",
    [
        ("11", "1O"),  # non-binary cell ('O' typo)
        ("111", "11"),  # ragged row
        ("11", "11", "11"),  # non-square (3x2)
    ],
)
def test_render_png_rejects_malformed_grid(bad_grid):
    with pytest.raises(ValueError, match="tag grid row"):
        gm.render_png(bad_grid)


def test_canonical_grids_are_10x10_with_quiet_zone():
    for grid in gm.CANONICAL_TAG36H11.values():
        assert len(grid) == 10
        assert all(len(row) == 10 for row in grid)
        assert grid[0] == "1111111111"  # white quiet-zone ring (top)
        assert grid[-1] == "1111111111"  # white quiet-zone ring (bottom)
