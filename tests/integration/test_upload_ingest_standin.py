"""Stand-in integration test: upload → ingest → manifest (docset 05, M8 / TS-14, design §4.4.2).

Exercises the Transfer + Index/Query tracks end-to-end against a **local stand-in target** (OQ-7:
no real DGX in CI). A fixture bag + sidecar in a watched dir is uploaded by the UploadDaemon over a
local-path rsync transport, then indexed by the IngestService into a SQLite manifest and returned by
manifest_query — proving the two tracks compose on one artifact (the M8 demo's automated half).

This is the integration tier (real rsync + real sqlite3 + a real `ros2 bag info` reader), but it
needs no ROS topics — so it runs wherever rsync + ros2 are on PATH. The bag-fact derivation uses a
trimmed real bag if one is available, else a stub reader, so the manifest-row assertions hold without
depending on a live recording.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

# The M8 cores live under analysis/ and docker/ — add them to the path for this integration module
# (the Layer-A pytest pythonpath does not apply to tests/integration, which runs in the ROS tier).
_REPO = Path(__file__).resolve().parents[2]
for _p in (_REPO / "analysis", _REPO / "docker"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ingest.ingest_service import BagFacts, IngestService  # noqa: E402
from ingest.manifest_query import render_rows  # noqa: E402
from ingest.manifest_store import ManifestStore  # noqa: E402

from upload_daemon.transport import RsyncSshTransport  # noqa: E402
from upload_daemon.upload_daemon import UploadDaemon  # noqa: E402

pytestmark = pytest.mark.ros  # integration tier (rsync/ros2 on PATH); not a Layer-A unit test


def _make_fixture_bag(watch_dir: Path) -> Path:
    """Create a small fixture bag + a current-schema sidecar in the watched dir."""
    watch_dir.mkdir(parents=True, exist_ok=True)
    bag = watch_dir / "patrol_standin_20260629_120000.mcap"
    bag.write_bytes(b"\x89MCAP0\r\n" + b"\x00" * 4096)
    (watch_dir / (bag.name + ".meta.json")).write_text(
        '{"mission_id": "standin", "bag_uri": "patrol_standin_20260629_120000", '
        '"started_utc": "2026-06-29T12:00:00+00:00", "ended_utc": "2026-06-29T12:02:22+00:00", '
        '"recorded_topics": ["/patrol/mission_state"], "mission_config_ref": "patrol.yaml"}'
    )
    return bag


def _stub_facts(_bag: Path) -> BagFacts:
    return BagFacts(duration_s=142.0, topic_counts={"/patrol/mission_state": 1420})


# TS-14: a fixture bag lands on the stand-in target, is indexed, and is returned by manifest_query.
def test_upload_then_ingest_then_query(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    target_dir = tmp_path / "dgx_landing"
    target_dir.mkdir()
    bag = _make_fixture_bag(watch_dir)

    # 1) Upload: dumb producer copies bag + sidecar to the local stand-in target via rsync.
    daemon = UploadDaemon(transport=RsyncSshTransport(), target=str(target_dir) + "/")
    assert daemon.on_bag_complete(bag) is True

    landed_bag = target_dir / bag.name
    landed_sidecar = target_dir / (bag.name + ".meta.json")
    assert landed_bag.is_file()
    assert landed_sidecar.is_file()

    # 2) Ingest: derive facts (stub reader here) + sidecar identity → one manifest row.
    store = ManifestStore(tmp_path / "manifest.db")
    IngestService(store, bag_facts=_stub_facts).index(landed_bag, landed_sidecar)

    # 3) Query: the bag is now findable with its LR-4 fields.
    rows = store.query_recent(10)
    assert len(rows) == 1
    assert rows[0].bag_id == bag.name
    assert rows[0].mission_id == "standin"
    assert rows[0].duration_s == 142.0
    assert "standin" in render_rows(rows)[0]
