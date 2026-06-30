"""CLI entry point for the ingest watch loop: ``python -m ingest`` (design §4.2.4 ingest trigger).

Thin watch-and-index shell around the tested :class:`~ingest.ingest_service.IngestService` core,
wiring the real :func:`~ingest.bag_reader.read_bag_facts` reader (which shells to ``ros2 bag info``,
so this runs inside the ingest container with a sourced ROS env). Mirrors the upload daemon's poll
loop: watch the DGX landing dir, and for each bag whose sidecar has also arrived, index it once.
Carries no first-party logic worth a Layer-A unit test — exercised by the stand-in integration test.

Usage:
    python -m ingest --watch /data/bags --db /data/manifest/bag_manifest.db
"""

from __future__ import annotations

import argparse
import json
import logging
import subprocess
import time
from pathlib import Path

from ingest.bag_reader import read_bag_facts
from ingest.ingest_service import IngestService
from ingest.manifest_store import ManifestStore

logger = logging.getLogger("ingest")

_POLL_INTERVAL_S = 5.0

# The documented fail-loud set IngestService.index raises on a bad input. The watch loop catches
# exactly these (not bare Exception) so one bad bag is skipped + retried later instead of crashing
# the service, while a genuine programming bug still surfaces loudly.
_INGEST_FAULTS = (
    FileNotFoundError,  # bag missing / not a finalized dir
    json.JSONDecodeError,  # malformed sidecar
    KeyError,  # sidecar missing a required field (e.g. mission_id)
    subprocess.CalledProcessError,  # `ros2 bag info` failed (non-zero exit)
    ValueError,  # `ros2 bag info` ran (exit 0) but had no parseable Duration line
)


def _sidecar_for(bag: Path) -> Path:
    return bag.with_name(bag.name + ".meta.json")


def _iter_bag_dirs(watch_dir: Path) -> list[Path]:
    """Candidate bag directories under ``watch_dir`` (sorted; empty if the dir doesn't exist yet)."""
    if not watch_dir.is_dir():
        return []
    return sorted(p for p in watch_dir.iterdir() if p.is_dir())


def _try_index(service: IngestService, bag: Path, sidecar: Path) -> bool:
    """Index one bag; on a known ingest fault log at ERROR and return False (skip, retry later)."""
    try:
        service.index(bag, sidecar)
    except _INGEST_FAULTS:
        logger.exception("skipping un-indexable bag %s (will retry on a later poll)", bag.name)
        return False
    logger.info("indexed %s", bag.name)
    return True


def _drain_once(service: IngestService, watch_dir: Path, indexed: set[Path]) -> None:
    """One discovery+index pass over ``watch_dir`` (mutates ``indexed`` with the freshly indexed)."""
    for bag in _iter_bag_dirs(watch_dir):
        sidecar = _sidecar_for(bag)
        if bag in indexed or not sidecar.is_file():
            continue
        if _try_index(service, bag, sidecar):
            indexed.add(bag)


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ingest", description=__doc__)
    parser.add_argument("--watch", required=True, type=Path, help="DGX landing dir to index")
    parser.add_argument("--db", required=True, type=Path, help="manifest SQLite path")
    parser.add_argument("--poll-interval", type=float, default=_POLL_INTERVAL_S)
    return parser.parse_args(argv)


def _watch_loop(service: IngestService, watch_dir: Path, poll_interval: float) -> None:
    """Poll ``watch_dir``; index each finalized bag dir once its sidecar is present.

    Fault-tolerant: a bag that fails to index (missing/corrupt input, malformed sidecar) is logged
    and skipped, never added to ``indexed`` — so it retries on a later poll once corrected and one
    bad bag can't starve the rest. A clean re-index of the same bag is idempotent (store-keyed).
    """
    indexed: set[Path] = set()
    while True:
        _drain_once(service, watch_dir, indexed)
        time.sleep(poll_interval)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s ingest %(message)s")
    args = _parse_args(argv)
    service = IngestService(ManifestStore(args.db), bag_facts=read_bag_facts)
    logger.info("watching %s -> %s", args.watch, args.db)
    _watch_loop(service, args.watch, args.poll_interval)


if __name__ == "__main__":
    main()
