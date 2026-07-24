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


def _write_sidecar(
    path: Path, *, mission_id: str = "patrol", bag_uri: str = "patrol_patrol_20260626_080740"
) -> Path:
    """Write a sidecar in the CURRENT recorder schema (bag_uri/started_utc — Research F4).

    ``bag_uri`` defaults to the ``_make_bag_dir`` default name so the identity guard (F-04) is
    satisfied by matched pairs; pass a different value to build the mismatched-sidecar negative case.
    """
    path.write_text(
        json.dumps(
            {
                "mission_id": mission_id,
                "bag_uri": bag_uri,
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


# F-01 (Mira High, review 4752923085): a finalized-LOOKING bag dir with no REAL .mcap payload is
# refused before any manifest write. Without this, the structured bag_reader branch parses the
# metadata cleanly and lands a fully-populated manifest row for an UNREPLAYABLE artifact — ingest
# deriving truth from a description of a bag that isn't there. Both shapes the review names: a
# metadata-only dir, and a dir whose only payload is a symlink out of the landing tree.
@pytest.mark.parametrize("payload", ["none", "symlink"], ids=["no-mcap", "symlinked-mcap"])
def test_index_raises_on_bag_dir_without_real_mcap_payload(tmp_path: Path, payload: str) -> None:
    bag = _make_bag_dir(tmp_path)
    (bag / f"{bag.name}_0.mcap").unlink()  # strip the real payload _make_bag_dir wrote
    if payload == "symlink":
        redirect = tmp_path / "payload-elsewhere.mcap"
        redirect.write_bytes(b"\x89MCAP0\r\n")
        (bag / f"{bag.name}_0.mcap").symlink_to(redirect)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(FileNotFoundError, match="payload"):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed for a payload-less bag


# TS-11: a wholly absent bag path also fails loudly before any manifest write.
def test_index_raises_on_missing_bag(tmp_path: Path) -> None:
    bag = tmp_path / "absent"  # never created
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"))
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(FileNotFoundError):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed


# F-02 (Mira High, review 4728294643): a required identity field that is valid JSON but not a
# non-empty string must be rejected by schema validation BEFORE any store call — previously such a
# value reached sqlite3 parameter binding and raised ProgrammingError, which is NOT an
# _INGEST_FAULTS member, so one malformed sidecar killed the ingest daemon. TypeError IS a member,
# so the watch loop now skips + retries the bag instead.
@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("mission_id", ["a"]),
        ("mission_id", 7),
        ("mission_id", ""),
        ("started_utc", {"t": 1}),
        ("started_utc", None),
        ("bag_uri", 42),
    ],
)
def test_index_rejects_non_string_required_sidecar_field(
    tmp_path: Path, field: str, value: object
) -> None:
    bag = _make_bag_dir(tmp_path)
    payload: dict[str, object] = {
        "mission_id": "patrol",
        "bag_uri": bag.name,
        "started_utc": "2026-06-26T08:07:40.635796+00:00",
    }
    payload[field] = value
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text(json.dumps(payload))
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(TypeError, match=field):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing half-indexed


# F-04: a sidecar whose bag_uri names a DIFFERENT bag must fail loudly and index nothing — a
# swapped/stale sidecar can no longer silently mis-label the manifest (Hermes Medium #2). ValueError is
# a member of _INGEST_FAULTS, so the watch loop skips (not crashes) the mismatched pair.
def test_index_raises_on_sidecar_bag_uri_mismatch(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"), bag_uri="patrol_some_other_bag")
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(ValueError, match="does not match bag dir"):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed


# Mira Medium (review 4728294643): index() refuses a SYMLINKED metadata.yaml even when invoked
# directly (not via the loop) — a planted symlink would redirect the finalized-bag read outside the
# landing tree. FileNotFoundError is an _INGEST_FAULTS member, so the watch loop logs + skips.
def test_index_raises_on_symlinked_metadata_yaml(tmp_path: Path) -> None:
    bag = tmp_path / "patrol_symmeta_20260626_080740"
    bag.mkdir()
    # A real payload: the SYMLINKED metadata.yaml must be the ONLY reason this is refused, or the
    # shared validator's payload requirement (F-01) would silently take over this symlink regression.
    (bag / f"{bag.name}_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    real_meta = tmp_path / "redirect-target-metadata.yaml"
    real_meta.write_text("rosbag2_bagfile_information:\n")
    (bag / "metadata.yaml").symlink_to(real_meta)
    sidecar = _write_sidecar(tmp_path / (bag.name + ".meta.json"), bag_uri=bag.name)
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(FileNotFoundError, match="symlinked"):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed through a symlink


# Mira Medium (review 4728294643): index() likewise refuses a SYMLINKED sidecar before reading it.
# ValueError is an _INGEST_FAULTS member, so the watch loop logs + skips.
def test_index_raises_on_symlinked_sidecar(tmp_path: Path) -> None:
    bag = _make_bag_dir(tmp_path)
    real_sidecar = _write_sidecar(tmp_path / "redirect-target.meta.json")
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.symlink_to(real_sidecar)
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(ValueError, match="symlinked sidecar"):
        IngestService(store, bag_facts=_fixed_facts()).index(bag, sidecar)

    assert store.query_recent(10) == []  # nothing indexed through a symlink
