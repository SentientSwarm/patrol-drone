"""Unit tests for CaptureWriter on-disk persistence (M6.C, T C.1/C.4 — design §4.2.6).

Layer-A: writes to a tmp dir, no ROS. Asserts the layout (<root>/<run>/NNN_<checkpoint_id>.{png,json}),
image-then-sidecar ordering, the returned image_path, and sidecar<->message KV consistency (PCAP-5/6).
"""

import json
from pathlib import Path

import pytest
from patrol_perception.capture_builder import CaptureRecord
from patrol_perception.capture_writer import CaptureWriter


def _record(checkpoint_id="cp_north", image_path=""):
    return CaptureRecord(
        stamp_sec=12,
        stamp_nanosec=34,
        frame_id="patrol_world",
        checkpoint_id=checkpoint_id,
        position=(1.0, 2.0, 3.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
        image_path=image_path,
        metadata={"tag_id": "0", "detection_confidence": "42.0"},
    )


def test_write_creates_image_and_sidecar_pair(tmp_path):
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    writer.write(_record(), b"PNGDATA")
    run_dir = tmp_path / "run1"
    assert (run_dir / "000_cp_north.png").read_bytes() == b"PNGDATA"
    assert (run_dir / "000_cp_north.json").exists()


def test_write_returns_image_path(tmp_path):
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    image_path = writer.write(_record(), b"PNGDATA")
    assert image_path == str(tmp_path / "run1" / "000_cp_north.png")
    assert Path(image_path).exists()


def test_index_increments_monotonically_even_for_repeated_checkpoint(tmp_path):
    # A multi-loop patrol re-visits the same checkpoint; the NNN prefix keeps each write distinct (AC-1).
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    writer.write(_record(checkpoint_id="cp_north"), b"a")
    writer.write(_record(checkpoint_id="cp_north"), b"b")
    run_dir = tmp_path / "run1"
    assert (run_dir / "000_cp_north.png").read_bytes() == b"a"
    assert (run_dir / "001_cp_north.png").read_bytes() == b"b"


def test_sidecar_carries_same_kv_as_record(tmp_path):
    # PCAP-5/6: the sidecar must carry the SAME checkpoint_id / pose / stamp / metadata as the message.
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    writer.write(_record(), b"PNGDATA")
    sidecar = json.loads((tmp_path / "run1" / "000_cp_north.json").read_text())
    assert sidecar["checkpoint_id"] == "cp_north"
    assert sidecar["metadata"] == {"tag_id": "0", "detection_confidence": "42.0"}
    assert sidecar["stamp"] == {"sec": 12, "nanosec": 34}
    assert sidecar["image"] == "000_cp_north.png"


def test_sidecar_image_matches_written_png(tmp_path):
    # The sidecar's image basename must match the actual PNG the writer produced (consistency).
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    image_path = writer.write(_record(), b"PNGDATA")
    sidecar = json.loads((tmp_path / "run1" / "000_cp_north.json").read_text())
    assert sidecar["image"] == Path(image_path).name


def test_creates_run_dir_if_absent(tmp_path):
    # Guard: output dir is created if it doesn't exist.
    nested = tmp_path / "deep" / "output"
    writer = CaptureWriter(output_root=str(nested), run_id="run1")
    writer.write(_record(), b"PNGDATA")
    assert (nested / "run1" / "000_cp_north.png").exists()


def test_image_written_before_sidecar(tmp_path, monkeypatch):
    # Guard (§4.2.6): a sidecar must never reference a missing image -> image is written first.
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    order = []
    real_write_bytes = Path.write_bytes
    real_write_text = Path.write_text

    def _track_bytes(self, data):
        order.append("png" if self.suffix == ".png" else "other")
        return real_write_bytes(self, data)

    def _track_text(self, data, *args, **kwargs):
        order.append("json" if self.suffix == ".json" else "other")
        return real_write_text(self, data, *args, **kwargs)

    monkeypatch.setattr(Path, "write_bytes", _track_bytes)
    monkeypatch.setattr(Path, "write_text", _track_text)
    writer.write(_record(), b"PNGDATA")
    assert order == ["png", "json"]


@pytest.mark.parametrize("checkpoint_id", ["cp_north", "cp_east", "cp_south_west"])
def test_filename_uses_checkpoint_id(tmp_path, checkpoint_id):
    writer = CaptureWriter(output_root=str(tmp_path), run_id="run1")
    writer.write(_record(checkpoint_id=checkpoint_id), b"x")
    assert (tmp_path / "run1" / f"000_{checkpoint_id}.png").exists()
