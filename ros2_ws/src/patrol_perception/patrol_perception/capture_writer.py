"""On-disk capture persistence — image + JSON sidecar (M6.C, T C.1 — design §4.2.6, PCAP-5).

Writes one PNG + one JSON sidecar per successful capture under a run-scoped directory:

    <output_root>/<run_id>/NNN_<checkpoint_id>.png
    <output_root>/<run_id>/NNN_<checkpoint_id>.json

``NNN`` is a monotonically increasing per-writer counter so a checkpoint re-visited on a second
patrol loop never overwrites its earlier capture (AC-1). The sidecar is the SAME KV set as the
published message — it is produced by ``CheckpointCaptureBuilder.build_sidecar`` from the same
``CaptureRecord`` (PCAP-6 single shape), so message and file can never drift. The image is written
BEFORE the sidecar so a sidecar never references a missing image (§4.2.6 guard).

Pure filesystem + stdlib json — ROS-free and unit-tested in a tmp dir (no rclpy/Gazebo).
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

from patrol_perception.capture_builder import CaptureRecord, CheckpointCaptureBuilder


class CaptureWriter:
    """Persists a CaptureRecord's image + sidecar to ``<output_root>/<run_id>/`` (PCAP-5)."""

    def __init__(self, output_root: str, run_id: str) -> None:
        self._run_dir = Path(output_root) / run_id
        self._index = 0

    def write(self, rec: CaptureRecord, image_bytes: bytes) -> str:
        """Write ``NNN_<checkpoint_id>.{png,json}``; return the PNG path for ``rec.image_path``.

        The PNG is written first, then the sidecar, so a sidecar never points at a missing image.
        The sidecar is built from the record AFTER its ``image_path`` is set to the final PNG path,
        so the sidecar's ``image`` basename matches the file on disk (UAC-PCAP-5 consistency).
        """
        self._run_dir.mkdir(parents=True, exist_ok=True)
        stem = f"{self._index:03d}_{rec.checkpoint_id}"
        image_path = self._run_dir / f"{stem}.png"
        sidecar_path = self._run_dir / f"{stem}.json"

        image_path.write_bytes(image_bytes)
        sidecar = CheckpointCaptureBuilder.build_sidecar(replace(rec, image_path=str(image_path)))
        sidecar_path.write_text(json.dumps(sidecar, indent=2))

        # Index advances only after BOTH writes succeed: an OSError here propagates (the coordinator
        # catches it and degrades per §4.4.5), so a failed write does NOT consume an NNN — the next
        # write reuses this index, overwriting any orphaned PNG from a partial write (no NNN gap, no
        # dangling sidecar; §4.4.5 row 2).
        self._index += 1
        return str(image_path)
