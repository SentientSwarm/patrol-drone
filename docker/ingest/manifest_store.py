"""The queryable bag manifest, backed by SQLite (design §4.2.4, OQ-3 / SWM-75 / T8.3).

``ManifestStore`` persists and serves one row per indexed bag. Its interface is store-agnostic —
``upsert`` / ``query_recent`` / ``query_by_mission`` say nothing about SQL — so the SQLite choice
(OQ-3, "SQLite or DuckDB, not Postgres") stays an implementation detail a later phase can swap.

The manifest is *just an index*: it can always be rebuilt by re-ingesting the bags, so a corrupt
SQLite file is recoverable (design §4.4.5). ``upsert`` is keyed on ``bag_id`` (the bag filename),
making re-ingestion of the same bag idempotent.
"""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path

_SCHEMA = """
CREATE TABLE IF NOT EXISTS bag_manifest (
    bag_id        TEXT PRIMARY KEY,   -- patrol_<missionId>_<timestamp>.mcap (LR-4 identity)
    mission_id    TEXT NOT NULL,      -- from sidecar (LR-4 "mission")
    recorded_utc  TEXT NOT NULL,      -- start time, from sidecar (LR-4 "time")
    duration_s    REAL NOT NULL,      -- DERIVED from the bag, not the sidecar (§3.4)
    topics_json   TEXT NOT NULL,      -- DERIVED topic list + per-topic msg counts (LR-4 "topics")
    metadata_json TEXT NOT NULL,      -- the sidecar contents (LR-4 "metadata")
    ingested_utc  TEXT NOT NULL       -- internal bookkeeping (when ingestion ran)
);
"""

_COLUMNS = (
    "bag_id",
    "mission_id",
    "recorded_utc",
    "duration_s",
    "topics_json",
    "metadata_json",
    "ingested_utc",
)


@dataclass(frozen=True)
class ManifestRow:
    """One indexed bag — the LR-4 record (mission, time, duration, topics, metadata) + bookkeeping."""

    bag_id: str
    mission_id: str
    recorded_utc: str
    duration_s: float
    topics_json: str
    metadata_json: str
    ingested_utc: str


class ManifestStore:
    """Persist + serve the bag manifest. SQLite-backed; store-agnostic interface."""

    def __init__(self, db_path: Path) -> None:
        self._db_path = db_path
        with self._connect() as conn:
            conn.executescript(_SCHEMA)

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._db_path)
        conn.row_factory = sqlite3.Row
        return conn

    def upsert(self, row: ManifestRow) -> None:
        """Insert ``row``, or replace it in place when its ``bag_id`` already exists (idempotent)."""
        placeholders = ", ".join("?" for _ in _COLUMNS)
        values = tuple(getattr(row, col) for col in _COLUMNS)
        with self._connect() as conn:
            conn.execute(
                f"INSERT OR REPLACE INTO bag_manifest ({', '.join(_COLUMNS)}) "
                f"VALUES ({placeholders})",
                values,
            )

    def query_recent(self, limit: int) -> list[ManifestRow]:
        """Return the ``limit`` most-recently-ingested bags, newest first."""
        with self._connect() as conn:
            cursor = conn.execute(
                "SELECT * FROM bag_manifest ORDER BY ingested_utc DESC LIMIT ?",
                (limit,),
            )
            return [self._to_row(r) for r in cursor.fetchall()]

    def query_by_mission(self, mission_id: str) -> list[ManifestRow]:
        """Return every indexed bag for ``mission_id``, newest first."""
        with self._connect() as conn:
            cursor = conn.execute(
                "SELECT * FROM bag_manifest WHERE mission_id = ? ORDER BY ingested_utc DESC",
                (mission_id,),
            )
            return [self._to_row(r) for r in cursor.fetchall()]

    @staticmethod
    def _to_row(record: sqlite3.Row) -> ManifestRow:
        return ManifestRow(**{col: record[col] for col in _COLUMNS})
