"""Layer-A unit tests for the ManifestStore (docset 05-logging-replay, M8 / T8.3, SWM-75).

Covers `ingest.manifest_store` — the queryable SQLite manifest that makes every uploaded bag
findable (design §4.2.4, LR-4). The store is deliberately store-agnostic in its interface
(upsert / query_recent / query_by_mission) so the SQLite choice (OQ-3) stays an implementation
detail:

  * upsert inserts one ``bag_manifest`` row keyed on ``bag_id`` (the bag filename); query_recent
    returns it with every LR-4 field (mission, time, duration, topics, metadata).
  * upsert is idempotent on the ``bag_id`` primary key — re-indexing the same bag updates the row
    in place rather than duplicating it (re-upload safe, §4.4.5 concurrent-re-upload).
  * query_by_mission filters to one mission's runs.

An in-memory / tmp SQLite file is used so the test carries no external dependency (sqlite3 is
stdlib). Fact *derivation* (duration/topics from the bag) is IngestService's job, tested separately.
"""

from __future__ import annotations

from pathlib import Path

from ingest.manifest_store import ManifestRow, ManifestStore


def _row(bag_id: str, *, mission: str = "patrol", duration: float = 142.0) -> ManifestRow:
    return ManifestRow(
        bag_id=bag_id,
        mission_id=mission,
        recorded_utc="2026-06-26T08:07:40+00:00",
        duration_s=duration,
        topics_json='{"/patrol/mission_state": 1420}',
        metadata_json='{"mission_id": "patrol"}',
        ingested_utc="2026-06-26T09:00:00+00:00",
    )


# TS-7: upsert inserts a row; query_recent returns it with all LR-4 fields intact.
def test_upsert_then_query_recent_returns_row(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "manifest.db")
    store.upsert(_row("patrol_a_20260626_080740.mcap", mission="patrol", duration=142.0))

    rows = store.query_recent(10)

    assert len(rows) == 1
    got = rows[0]
    assert got.bag_id == "patrol_a_20260626_080740.mcap"
    assert got.mission_id == "patrol"
    assert got.duration_s == 142.0
    assert got.topics_json == '{"/patrol/mission_state": 1420}'
    assert got.metadata_json == '{"mission_id": "patrol"}'


# TS-8: upsert is idempotent on the bag_id PK — re-indexing updates in place, no duplicate row.
def test_upsert_is_idempotent_on_bag_id(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "manifest.db")
    store.upsert(_row("patrol_a_20260626_080740.mcap", duration=100.0))
    store.upsert(_row("patrol_a_20260626_080740.mcap", duration=142.0))  # re-index, new duration

    rows = store.query_recent(10)

    assert len(rows) == 1  # not two
    assert rows[0].duration_s == 142.0  # updated in place


# TS-7: query_recent returns the N most recent rows, newest first, bounded by N.
def test_query_recent_orders_and_limits(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "manifest.db")
    for i in range(5):
        r = _row(f"patrol_a_{i}.mcap")
        # later ingest time = more recent
        r = ManifestRow(**{**r.__dict__, "ingested_utc": f"2026-06-26T09:0{i}:00+00:00"})
        store.upsert(r)

    rows = store.query_recent(3)

    assert len(rows) == 3
    assert rows[0].bag_id == "patrol_a_4.mcap"  # newest first


# TS-13 (store half): query_by_mission filters to one mission's bags.
def test_query_by_mission_filters(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "manifest.db")
    store.upsert(_row("patrol_a.mcap", mission="patrol"))
    store.upsert(_row("survey_b.mcap", mission="survey"))

    rows = store.query_by_mission("survey")

    assert len(rows) == 1
    assert rows[0].mission_id == "survey"
