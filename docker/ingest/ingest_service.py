"""Index a landed bag into the manifest (design §4.2.4, LR-4 / SWM-76 / T8.4).

``IngestService.index`` is the smart half of the dumb-producer / smart-ingestion split (design
§3.4). It derives the *authoritative* facts — duration and per-topic message counts — FROM THE BAG
ITSELF (never from the sidecar, which a buggy producer could get wrong), reads identity and the
metadata blob from the sidecar, and upserts one manifest row. Re-indexing the same bag is
idempotent (the store is keyed on ``bag_id``).

The bag-fact reader is injected (``bag_facts``) so the core is ROS-free and unit-testable; the
default reader shells out to ``ros2 bag info`` and is exercised by the stand-in integration test.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path

from ingest.manifest_store import ManifestRow, ManifestStore


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
        manifest write, so a bad input is never silently half-indexed (§4.4.5).
        """
        if not (bag_path / "metadata.yaml").is_file():
            raise FileNotFoundError(f"not a finalized bag dir (no metadata.yaml): {bag_path}")
        sidecar = json.loads(sidecar_path.read_text())

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
