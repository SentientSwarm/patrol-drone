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


def test_canonical_grids_are_10x10_with_quiet_zone():
    for grid in gm.CANONICAL_TAG36H11.values():
        assert len(grid) == 10
        assert all(len(row) == 10 for row in grid)
        assert grid[0] == "1111111111"  # white quiet-zone ring (top)
        assert grid[-1] == "1111111111"  # white quiet-zone ring (bottom)
