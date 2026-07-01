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


def _run_id_is_unsafe(run_id: str) -> bool:
    """True if ``run_id`` is not a safe single path segment (SWM-83 defense-in-depth, mirrors
    ``patrol_logging.recorder.run_id_rejection`` — a separate colcon package can't import it)."""
    return (
        not run_id
        or run_id != run_id.strip()
        or "/" in run_id
        or "\\" in run_id
        or run_id in (".", "..")
    )


class CaptureWriter:
    """Persists a CaptureRecord's image + sidecar to ``<output_root>/<run_id>/`` (PCAP-5)."""

    def __init__(self, output_root: str, run_id: str) -> None:
        # Defense-in-depth (SWM-83): run_id is validated once upstream at recorder.resolve_run_id,
        # but CaptureWriter is the actual path-join site, so reject a path-hostile token here too —
        # a separator / `..` / absolute run_id would let <output_root>/<run_id> escape the root.
        if _run_id_is_unsafe(run_id):
            raise ValueError(f"run_id must be a safe single path segment: {run_id!r}")
        self._run_dir = Path(output_root) / run_id
        self._index = 0

    @property
    def run_dir(self) -> Path:
        """The run-scoped output directory (``<output_root>/<run_id>``)."""
        return self._run_dir

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
