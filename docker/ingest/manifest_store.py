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
    bag_id        TEXT PRIMARY KEY,   -- the bag DIR name patrol_<missionId>_<timestamp> (no .mcap; LR-4 identity)
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


def _ensure_parent_dir(db_path: Path) -> None:
    """Create the DB file's parent directory if it doesn't exist yet.

    ``sqlite3.connect`` creates the DB *file* but never intermediate directories, so a documented
    nested path like ``/tmp/dgx_manifest/bag_manifest.db`` raises OperationalError on a clean host
    (Mira Medium, review 4748505221). Creating the parent first makes the documented fresh path
    "just work"; idempotent (``exist_ok=True``) and a no-op for a bare relative filename whose parent
    resolves to ``.``.
    """
    parent = db_path.parent
    if parent != Path():
        parent.mkdir(parents=True, exist_ok=True)


class ManifestStore:
    """Persist + serve the bag manifest. SQLite-backed; store-agnostic interface."""

    # SQLite waits up to this long for a competing writer's lock before raising OperationalError
    # ("database is locked"). A manifest_query CLI reading while the daemon writes is exactly this
    # transient contention; a bounded wait lets it clear instead of surfacing as a fault (Mira F-02).
    _BUSY_TIMEOUT_S = 5.0

    def __init__(self, db_path: Path) -> None:
        self._db_path = Path(db_path)
        _ensure_parent_dir(self._db_path)
        with self._connect() as conn:
            conn.executescript(_SCHEMA)

    def _connect(self) -> sqlite3.Connection:
        conn = sqlite3.connect(self._db_path, timeout=self._BUSY_TIMEOUT_S)
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
        """Return the ``limit`` most-recently-*ingested* bags, newest first (rebuild-order-stable).

        Ordered by ``ingested_utc`` DELIBERATELY — this is the "what did the daemon index last" view,
        stable across a manifest rebuild. For the "recently *flown*" (record-time) view, use
        :meth:`query_recently_recorded`, which orders by ``recorded_utc``. The two orderings are
        intentionally distinct methods; ``query_recent`` is not meant to track record time.
        """
        return self._query_ordered("ingested_utc DESC", limit)

    def query_recently_recorded(self, limit: int) -> list[ManifestRow]:
        """Return the ``limit`` most-recently-*recorded* (flown) bags, newest first.

        Orders by ``recorded_utc`` (the mission's start time from the sidecar) with ``ingested_utc``
        as a tiebreaker, so the operator ``--recent`` view tracks "recently flown" and is stable
        across a manifest rebuild (re-ingesting old bags does not float them to the top).
        """
        return self._query_ordered("recorded_utc DESC, ingested_utc DESC", limit)

    def _query_ordered(self, order_by: str, limit: int) -> list[ManifestRow]:
        """Run the ``SELECT * ... ORDER BY <order_by> LIMIT ?`` shared by the recent queries.

        ``order_by`` is a fixed internal literal (never operator input), so interpolating it here is
        safe; ``limit`` stays a bound ``?`` parameter. ``limit`` is rejected below 1 because SQLite
        reads a NEGATIVE LIMIT as *no limit* — the CLI rejects that at parse time (``positive_int``),
        and this is the store-level backstop so a future non-CLI caller cannot reintroduce an
        unbounded scan by passing a computed value through (Mira Low, review 4752923085). Raise
        rather than clamp: this repo's boundary convention is fail-loud, and ``ValueError`` is
        already an ``_INGEST_FAULTS`` member so it could never crash the watch loop.
        """
        if limit < 1:
            raise ValueError(
                f"limit must be >= 1, got {limit} (a negative SQLite LIMIT is unbounded)"
            )
        with self._connect() as conn:
            cursor = conn.execute(
                f"SELECT * FROM bag_manifest ORDER BY {order_by} LIMIT ?",
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

    def contains(self, bag_id: str) -> bool:
        """True iff a row for ``bag_id`` is already in the manifest (the ingest loop's durable skip)."""
        with self._connect() as conn:
            cursor = conn.execute("SELECT 1 FROM bag_manifest WHERE bag_id = ? LIMIT 1", (bag_id,))
            return cursor.fetchone() is not None

    @staticmethod
    def _to_row(record: sqlite3.Row) -> ManifestRow:
        return ManifestRow(**{col: record[col] for col in _COLUMNS})
