"""Index a landed bag into the manifest (design §4.2.4, LR-4 / SWM-76 / T8.4).

``IngestService.index`` is the smart half of the dumb-producer / smart-ingestion split (design
§3.4). It derives the *authoritative* facts — duration and per-topic message counts — FROM THE BAG
ITSELF (never from the sidecar, which a buggy producer could get wrong), reads identity and the
metadata blob from the sidecar, and upserts one manifest row. Re-indexing the same bag is
idempotent (the store is keyed on ``bag_id``).

The bag-fact reader is injected (``bag_facts``) so the core is ROS-free and unit-testable; the
default reader prefers the bag's structured ``metadata.yaml`` and falls back to shelling out to
``ros2 bag info`` (both derive facts from the bag), and is exercised by the stand-in integration
test.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from ingest.manifest_store import ManifestRow, ManifestStore

# The sidecar fields that feed typed manifest columns (identity + record time). Validated as
# non-empty strings BEFORE any store call: a valid-JSON sidecar with a list/dict/int/null value
# here would otherwise reach sqlite3 parameter binding and raise ProgrammingError — which is NOT an
# ingest fault, so it killed the watch loop (Mira High, review 4728294643). Schema validation at
# the boundary converts every shape surprise into a caught, logged, skip-and-retry TypeError,
# instead of enumerating downstream exception types one at a time.
_REQUIRED_SIDECAR_FIELDS = ("bag_uri", "mission_id", "started_utc")


def _validate_sidecar_fields(sidecar: dict, sidecar_path: Path) -> None:
    """Raise TypeError (an _INGEST_FAULTS member) unless every required field is a non-empty str."""
    for name in _REQUIRED_SIDECAR_FIELDS:
        value = sidecar.get(name)
        if not isinstance(value, str) or not value:
            raise TypeError(
                f"sidecar field {name!r} must be a non-empty string, got {value!r}: {sidecar_path}"
            )


@dataclass(frozen=True)
class BagFacts:
    """The facts derived FROM the bag (not the sidecar): duration + per-topic message counts."""

    duration_s: float
    topic_counts: dict[str, int]


# A bag-fact reader maps a bag path to its derived facts. Injected so tests stay ROS-free.
BagFactsReader = Callable[[Path], BagFacts]


class IngestService:
    """Derive facts from a bag, read identity from its sidecar, upsert one manifest row (LR-4)."""

    def __init__(self, store: ManifestStore, bag_facts: BagFactsReader) -> None:
        self._store = store
        self._bag_facts = bag_facts

    def index(self, bag_path: Path, sidecar_path: Path) -> None:
        """Index ``bag_path`` (a finalized rosbag2 bag dir) using ``sidecar_path`` for identity.

        Guards: the bag must be a finalized bag directory (``metadata.yaml`` present) and the sidecar
        must parse as JSON — both fail loudly (FileNotFoundError / JSONDecodeError) before any
        manifest write, so a bad input is never silently half-indexed (§4.4.5). Symlinked
        ``metadata.yaml``/sidecar files are refused (Mira Medium, review 4728294643): the landing
        dir's writers are remote, and a planted symlink would redirect the reads outside the
        landing tree. Both raised types are ``_INGEST_FAULTS`` members, so the loop logs + skips.
        """
        meta = bag_path / "metadata.yaml"
        if meta.is_symlink() or not meta.is_file():
            raise FileNotFoundError(
                f"not a finalized bag dir (no metadata.yaml, or symlinked): {bag_path}"
            )
        if sidecar_path.is_symlink():
            raise ValueError(f"refusing symlinked sidecar: {sidecar_path}")
        sidecar = json.loads(sidecar_path.read_text())
        if not isinstance(sidecar, dict):
            raise TypeError(
                f"sidecar is not a JSON object: {sidecar_path}"
            )  # caught by _INGEST_FAULTS
        _validate_sidecar_fields(sidecar, sidecar_path)
        if sidecar.get("bag_uri") != bag_path.name:
            raise ValueError(
                f"sidecar bag_uri {sidecar.get('bag_uri')!r} does not match bag dir "
                f"{bag_path.name!r} — refusing to index a mismatched sidecar/bag pair"
            )

        facts = self._bag_facts(bag_path)  # DERIVED from the bag — the trusted topic/duration truth

        self._store.upsert(
            ManifestRow(
                bag_id=bag_path.name,
                mission_id=sidecar["mission_id"],
                recorded_utc=sidecar["started_utc"],
                duration_s=facts.duration_s,
                topics_json=json.dumps(facts.topic_counts, sort_keys=True),
                metadata_json=json.dumps(sidecar, sort_keys=True),
                ingested_utc=datetime.now(UTC).isoformat(),
            )
        )

    def already_indexed(self, bag_path: Path) -> bool:
        """True iff ``bag_path`` already has a manifest row (the watch loop's durable skip, F-05).

        Backed by the manifest itself, so an already-indexed bag is skipped even after the loop's
        in-memory seen-set has evicted it — a long retention window never re-derives old bags each
        poll. Kept here (not reaching into the store from ``__main__``) so the service owns its store.
        """
        return self._store.contains(bag_path.name)
