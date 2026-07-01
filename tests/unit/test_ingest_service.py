"""Layer-A unit tests for IngestService (docset 05-logging-replay, M8 / T8.4, SWM-76).

Covers `ingest.ingest_service.IngestService.index` — the fact-deriver that turns a landed bag +
sidecar into one manifest row (design §4.2.4, LR-4). The load-bearing invariant is the
dumb-producer / smart-ingestion split (design §3.4):

  * duration_s and the per-topic counts are DERIVED FROM THE BAG, never read from the sidecar — a
    buggy sidecar can never corrupt the indexed topic truth.
  * mission_id / recorded_utc / the metadata blob come FROM THE SIDECAR (identity + correlation).
  * the sidecar is parsed with the CURRENT recorder schema (``bag_uri`` + ``started_utc``), not the
    older on-disk ``bag_filename`` shape (Research F4).
  * guards: an unreadable bag or an unparseable sidecar fail loudly, not silently (§4.4.5).

The bag-fact reader is injected so the core stays ROS-free; the real reader (``ros2 bag info`` /
an MCAP reader) is exercised by the stand-in integration test.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest
from ingest.ingest_service import BagFacts, BagFactsReader, IngestService
from ingest.manifest_store import ManifestStore


def _make_bag_dir(tmp_path: Path, name: str = "patrol_patrol_20260626_080740") -> Path:
    """Create a finalized rosbag2 bag directory (``<name>/metadata.yaml`` + nested ``_0.mcap``).

    rosbag2 writes each run as a directory finalized by ``metadata.yaml``; the ingest guard keys on
    that marker. Returns the bag-directory path (the unit IngestService.index now takes).
    """
    bag = tmp_path / name
    bag.mkdir()
    (bag / f"{name}_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    return bag


def _write_sidecar(path: Path, *, mission_id: str = "patrol") -> Path:
    """Write a sidecar in the CURRENT recorder schema (bag_uri/started_utc — Research F4)."""
    path.write_text(
        json.dumps(
            {
                "mission_id": mission_id,
                "bag_uri": "patrol_patrol_20260626_080740",
                "started_utc": "2026-06-26T08:07:40.635796+00:00",
                "ended_utc": "2026-06-26T08:10:03.410167+00:00",
                "recorded_topics": ["/patrol/mission_state", "/fmu/out/.*"],
                "mission_config_ref": "patrol_mission.yaml",
            }
        )
    )
    return path


def _fixed_facts(duration: float = 142.0) -> BagFactsReader:
    """A bag-fact reader returning fixed DERIVED facts (duration + per-topic counts)."""

    def reader(_bag_path: Path) -> BagFacts:
        return BagFacts(
            duration_s=duration,
            topic_counts={"/patrol/mission_state": 1420, "/fmu/out/vehicle_status": 7100},
        )

    return reader


# TS-9: duration_s + topic counts come FROM THE BAG (the injected reader), not the sidecar.
def test_index_derives_duration_and_topics_from_bag(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    IngestService(store, bag_facts=_fixed_facts(duration=142.0)).index(bag, sidecar)

    row = store.query_recent(1)[0]
    assert row.duration_s == 142.0  # from the reader, NOT any sidecar field
    topics = json.loads(row.topics_json)
    assert topics["/patrol/mission_state"] == 1420
    assert topics["/fmu/out/vehicle_status"] == 7100


# TS-10: identity (mission_id, recorded_utc) + metadata come from the CURRENT sidecar schema.
def test_index_reads_identity_from_current_sidecar_schema(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"), mission_id="patrol")
    store = ManifestStore(tmp_path / "m.db")

    IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    row = store.query_recent(1)[0]
    assert row.bag_id == bag.name
    assert row.mission_id == "patrol"
    assert row.recorded_utc == "2026-06-26T08:07:40.635796+00:00"  # started_utc, current schema
    assert json.loads(row.metadata_json)["bag_uri"] == "patrol_patrol_20260626_080740"


# TS-9: re-indexing the same bag is idempotent end-to-end (one row, latest facts).
def test_reindex_same_bag_is_idempotent(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    IngestService(store, bag_facts=_fixed_facts(duration=100.0)).index(bag, sidecar)
    IngestService(store, bag_facts=_fixed_facts(duration=142.0)).index(bag, sidecar)

    rows = store.query_recent(10)
    assert len(rows) == 1
    assert rows[0].duration_s == 142.0


# TS-11: an unparseable sidecar fails loudly (not a silent skip) — §4.4.5 guard.
def test_index_raises_on_unparseable_sidecar(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path, name="patrol_x")
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text("{not json")
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(json.JSONDecodeError):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)


# TS-11: a bag dir without metadata.yaml (not finalized / wrong path) fails loudly, indexes nothing.
def test_index_raises_on_unfinalized_bag_dir(tmp_path: Path) -> None:
    bag = tmp_path / "patrol_unfinalized"
    bag.mkdir()  # exists, but no metadata.yaml — rosbag2 never finalized it
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(FileNotFoundError):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed


# TS-11: a wholly absent bag path also fails loudly before any manifest write.
def test_index_raises_on_missing_bag(tmp_path: Path) -> None:
    bag = tmp_path / "absent"  # never created
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(FileNotFoundError):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed
