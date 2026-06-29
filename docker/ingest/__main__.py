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
import logging
import time
from pathlib import Path

from ingest.bag_reader import read_bag_facts
from ingest.ingest_service import IngestService
from ingest.manifest_store import ManifestStore

logger = logging.getLogger("ingest")

_POLL_INTERVAL_S = 5.0


def _sidecar_for(bag: Path) -> Path:
    return bag.with_name(bag.name + ".meta.json")


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ingest", description=__doc__)
    parser.add_argument("--watch", required=True, type=Path, help="DGX landing dir to index")
    parser.add_argument("--db", required=True, type=Path, help="manifest SQLite path")
    parser.add_argument("--poll-interval", type=float, default=_POLL_INTERVAL_S)
    return parser.parse_args(argv)


def _watch_loop(service: IngestService, watch_dir: Path, poll_interval: float) -> None:
    """Poll ``watch_dir``; index each bag once its sidecar is also present (idempotent re-index)."""
    indexed: set[Path] = set()
    while True:
        for bag in sorted(watch_dir.glob("*.mcap")):
            if bag in indexed or not _sidecar_for(bag).is_file():
                continue
            service.index(bag, _sidecar_for(bag))
            indexed.add(bag)
            logger.info("indexed %s", bag.name)
        time.sleep(poll_interval)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s ingest %(message)s")
    args = _parse_args(argv)
    service = IngestService(ManifestStore(args.db), bag_facts=read_bag_facts)
    logger.info("watching %s -> %s", args.watch, args.db)
    _watch_loop(service, args.watch, args.poll_interval)


if __name__ == "__main__":
    main()
