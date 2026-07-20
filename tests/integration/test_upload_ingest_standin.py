"""Stand-in integration test: upload → ingest → manifest (docset 05, M8 / TS-14, design §4.4.2).

Exercises the Transfer + Index/Query tracks end-to-end against a **local stand-in target** (OQ-7:
no real DGX in CI). A fixture bag + sidecar in a watched dir is uploaded by the UploadDaemon over a
local-path rsync transport, then indexed by the IngestService into a SQLite manifest and returned by
manifest_query — proving the two tracks compose on one artifact (the M8 demo's automated half).

This is the integration tier (real rsync + real sqlite3 + a real `ros2 bag info` reader), but it
needs no ROS topics — so it runs wherever rsync + ros2 are on PATH. Bag-fact derivation is covered
at two tiers (F-05, Mira review 4731322384): the stub-reader tests are fast COMPOSITION checks
(upload → index → query wiring), and the real-reader tests drive the production ``read_bag_facts``
over the checked-in LFS reference bag — both its structured metadata.yaml branch and its
``ros2 bag info`` subprocess fallback — so a regression in the production bag-fact boundary is
caught here rather than masked by the stub.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

# The M8 cores live under analysis/ and docker/ — add them to the path for this integration module
# (the Layer-A pytest pythonpath does not apply to tests/integration, which runs in the ROS tier).
_REPO = Path(__file__).resolve().parents[2]
for _p in (_REPO / "analysis", _REPO / "docker"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ingest.__main__ import _drain_once  # noqa: E402
from ingest.bag_reader import read_bag_facts  # noqa: E402
from ingest.bounded_seen import _BoundedSeen  # noqa: E402
from ingest.ingest_service import BagFacts, IngestService  # noqa: E402
from ingest.manifest_query import render_rows  # noqa: E402
from ingest.manifest_store import ManifestStore  # noqa: E402

from upload_daemon.transport import RsyncSshTransport  # noqa: E402
from upload_daemon.upload_daemon import UploadDaemon  # noqa: E402

pytestmark = pytest.mark.ros  # integration tier (rsync/ros2 on PATH); not a Layer-A unit test


def _make_fixture_bag(
    watch_dir: Path,
    name: str = "patrol_standin_20260629_120000",
    *,
    sidecar_text: str | None = None,
) -> Path:
    """Create a fixture in the REAL rosbag2 layout + a sibling sidecar in the watched dir.

    rosbag2 writes each run as a directory ``<name>/`` (the ``-o`` URI) holding a nested
    ``<name>_0.mcap`` and a ``metadata.yaml`` finalization marker; the sidecar is a sibling
    ``<name>.meta.json``. Building the real shape here is what makes the upload/ingest directory
    discovery (F-05) genuinely exercised instead of hidden behind a flat fixture. ``sidecar_text``
    overrides the sidecar body (e.g. malformed JSON for the fault-tolerance test); else a
    current-schema sidecar is written. Returns the bag-directory path.
    """
    watch_dir.mkdir(parents=True, exist_ok=True)
    bag = watch_dir / name
    bag.mkdir()
    (bag / f"{name}_0.mcap").write_bytes(b"\x89MCAP0\r\n" + b"\x00" * 4096)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    (watch_dir / (name + ".meta.json")).write_text(
        sidecar_text
        if sidecar_text is not None
        else (
            f'{{"mission_id": "standin", "bag_uri": "{name}", '
            '"started_utc": "2026-06-29T12:00:00+00:00", "ended_utc": "2026-06-29T12:02:22+00:00", '
            '"recorded_topics": ["/patrol/mission_state"], "mission_config_ref": "patrol.yaml"}'
        )
    )
    return bag


def _stub_facts(_bag: Path) -> BagFacts:
    return BagFacts(duration_s=142.0, topic_counts={"/patrol/mission_state": 1420})


def _assert_bag_dir_landed(landed_bag: Path, name: str) -> None:
    """The WHOLE bag directory (nested MCAP + metadata.yaml finalization marker) crossed via rsync -a."""
    assert landed_bag.is_dir()
    assert (landed_bag / "metadata.yaml").is_file()
    assert (landed_bag / f"{name}_0.mcap").is_file()


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
    _assert_bag_dir_landed(landed_bag, bag.name)
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


# TS-15 (F-04): one bag with a malformed sidecar must not starve the healthy ones — the watch loop
# skips it (logs + leaves it un-indexed so it can retry) and indexes the rest, no crash-loop.
def test_one_bad_sidecar_does_not_block_other_bags(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    _make_fixture_bag(watch_dir, "patrol_good_a_20260629_120000")
    _make_fixture_bag(watch_dir, "patrol_bad_20260629_121000", sidecar_text="{not valid json")
    _make_fixture_bag(watch_dir, "patrol_good_b_20260629_122000")

    store = ManifestStore(tmp_path / "manifest.db")
    service = IngestService(store, bag_facts=_stub_facts)
    indexed = _BoundedSeen()  # the loop's real bounded seen-set (mypy: _drain_once's declared type)

    _drain_once(service, watch_dir, indexed)  # one deterministic pass, never raises

    by_id = {r.bag_id for r in store.query_recent(10)}
    assert by_id == {"patrol_good_a_20260629_120000", "patrol_good_b_20260629_122000"}
    # The bad bag is NOT marked indexed, so a later (corrected) poll would retry it.
    assert (watch_dir / "patrol_bad_20260629_121000") not in indexed
    assert len(indexed) == 2


# --- Real-reader tier (F-05, Mira review 4731322384): the stub tests above prove the COMPOSITION;
# these prove the PRODUCTION bag-fact boundary — the real read_bag_facts over a real finalized bag
# (the LFS reference bag the replay lane already materializes), covering BOTH of its branches:
# the structured metadata.yaml parse and the `ros2 bag info` subprocess fallback Mira named.

_REFERENCE_BAG = _REPO / "tests" / "replay" / "reference" / "patrol_reference"


def _require_reference_bag() -> Path:
    """The LFS reference bag as a real production fixture — hard-fail if it's an unresolved pointer."""
    mcap = next(_REFERENCE_BAG.glob("*.mcap"), None)
    if mcap is None or mcap.stat().st_size < 1024:
        pytest.fail(
            f"reference bag missing/unresolved at {_REFERENCE_BAG} — checkout needs lfs:true "
            "(this lane materializes it; the real-reader ingest tests require it)."
        )
    return _REFERENCE_BAG


# TS-16 (F-05): a REAL finalized bag ingested through the PRODUCTION read_bag_facts (no stub) — the
# metadata.yaml branch. A regression in the real reader (a changed metadata shape, a parse bug) fails
# here, where _stub_facts would have masked it. Loose bag-derived assertions (duration > 0, non-empty
# topics) so a reference-bag regeneration doesn't false-fail the test.
def test_ingest_real_bag_through_production_reader(tmp_path: Path) -> None:
    bag = _require_reference_bag()
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text(
        f'{{"mission_id": "refbag", "bag_uri": "{bag.name}", '
        '"started_utc": "2026-06-27T17:01:01+00:00", "ended_utc": "2026-06-27T17:01:21+00:00", '
        '"recorded_topics": ["/patrol/mission_state"], "mission_config_ref": "patrol.yaml"}'
    )
    store = ManifestStore(tmp_path / "manifest.db")
    IngestService(store, bag_facts=read_bag_facts).index(bag, sidecar)  # REAL reader, no stub

    rows = store.query_recent(10)
    assert len(rows) == 1
    assert rows[0].mission_id == "refbag"
    assert rows[0].duration_s > 0  # DERIVED from the bag, not the sidecar (dumb-producer, §3.4)
    assert json.loads(rows[0].topics_json)  # non-empty real topic set from the production reader


# TS-17 (F-05): the `ros2 bag info` SUBPROCESS branch Mira explicitly named. The reference bag
# carries a real metadata.yaml, so read_bag_facts prefers the structured parse and never shells out —
# to force the fallback we strip metadata.yaml from a copy and hand the reader the bare .mcap. (The
# metadata-less COPY can't go through IngestService.index: _require_finalized_bag refuses a bag dir
# without metadata.yaml by design, and `ros2 bag info` itself errors on a metadata-less DIR — but it
# reads a bare .mcap fine, which is exactly the artifact shape the fallback exists for.) Both
# production branches must derive the SAME truth from the same bag.
def test_read_bag_facts_ros2_bag_info_fallback_matches_metadata(tmp_path: Path) -> None:
    src = _require_reference_bag()
    bag_copy = tmp_path / src.name
    shutil.copytree(src, bag_copy)
    (bag_copy / "metadata.yaml").unlink()  # force the subprocess fallback
    mcap = next(bag_copy.glob("*.mcap"))

    info_facts = read_bag_facts(mcap)  # metadata.yaml unfindable from a file path → ros2 bag info
    meta_facts = read_bag_facts(src)  # the structured metadata.yaml branch, same underlying bag

    assert info_facts.duration_s > 0
    assert info_facts.topic_counts  # non-empty, parsed from real `ros2 bag info` stdout
    assert info_facts.topic_counts == meta_facts.topic_counts
    assert info_facts.duration_s == pytest.approx(meta_facts.duration_s, rel=1e-6)
