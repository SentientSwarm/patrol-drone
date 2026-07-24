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
import sqlite3
import subprocess
import time
from pathlib import Path

import yaml

from ingest.bag_layout import is_regular_file
from ingest.bag_reader import read_bag_facts
from ingest.bounded_seen import _BoundedSeen
from ingest.ingest_service import IngestService
from ingest.manifest_store import ManifestStore
from ingest.positive_interval import positive_interval

logger = logging.getLogger("ingest")

_POLL_INTERVAL_S = 5.0


# The documented fail-loud set IngestService.index raises on a bad input. The watch loop catches
# exactly these (not bare Exception) so one bad bag is skipped + retried later instead of crashing
# the service, while a genuine programming bug still surfaces loudly.
_INGEST_FAULTS = (
    FileNotFoundError,  # bag missing / not a finalized dir
    json.JSONDecodeError,  # malformed sidecar
    KeyError,  # sidecar missing a required field (e.g. mission_id)
    TypeError,  # sidecar is valid JSON but not an object → sidecar["mission_id"] can't subscript
    UnicodeDecodeError,  # sidecar is not valid UTF-8 → read_text() fails (a ValueError subclass;
    #                      listed explicitly to keep the set self-documenting)
    sqlite3.IntegrityError,  # sidecar field present but JSON null (e.g. mission_id: null) → the DB
    #                          NOT NULL column rejects it at the write (greptile P1)
    subprocess.CalledProcessError,  # `ros2 bag info` failed (non-zero exit)
    subprocess.TimeoutExpired,  # `ros2 bag info` hung past its timeout (F-03) → skip + retry, don't
    #                             wedge the serial loop on one bad/pathological bag
    ValueError,  # `ros2 bag info` ran (exit 0) but had no parseable Duration line, or zero topics (F-02)
    yaml.YAMLError,  # metadata.yaml exists but is malformed (truncated/partially overwritten mid-write)
    #                  → parse_bag_metadata's yaml.safe_load raises → skip + retry, don't crash-loop
    sqlite3.OperationalError,  # transient locked/busy manifest (a concurrent reader/writer) → skip +
    #                            retry, don't crash the daemon; NOT IntegrityError (that stays fatal,
    #                            it signals corruption/constraint violation, not transient contention)
    PermissionError,  # metadata.yaml / sidecar present but unreadable (chmod 000, owned by another user
    #                   mid-sync) → skip + retry, don't kill the daemon on one bag (Mira F-04)
    OSError,  # other intended file-read failure reading metadata/sidecar (transient I/O, stale NFS
    #           handle). Kept AFTER the more specific subclasses above so the log still attributes the
    #           specific class; a genuine bug is not an OSError, so it still surfaces (Mira F-04)
)


def _sidecar_for(bag: Path) -> Path:
    return bag.with_name(bag.name + ".meta.json")


def _iter_bag_dirs(watch_dir: Path) -> list[Path]:
    """Candidate bag directories under ``watch_dir`` (sorted; empty if the dir doesn't exist yet).

    Symlinked entries are refused, and every candidate must resolve inside the resolved watch root
    (Mira Medium, review 4728294643) — mirroring the upload sibling's discovery rule.
    """
    if not watch_dir.is_dir():
        return []
    root = watch_dir.resolve()
    return sorted(
        p
        for p in watch_dir.iterdir()
        if p.is_dir() and not p.is_symlink() and p.resolve().is_relative_to(root)
    )


def _try_index(service: IngestService, bag: Path, sidecar: Path) -> bool:
    """Index one bag; on a known ingest fault log at ERROR and return False (skip, retry later)."""
    try:
        service.index(bag, sidecar)
    except _INGEST_FAULTS:
        logger.exception("skipping un-indexable bag %s (will retry on a later poll)", bag.name)
        return False
    logger.info("indexed %s", bag.name)
    return True


def _drain_once(service: IngestService, watch_dir: Path, indexed: _BoundedSeen) -> None:
    """One discovery+index pass over ``watch_dir`` (mutates ``indexed`` with the freshly indexed).

    'Already handled' is durable (F-05): a bag already in the manifest is skipped even after the
    in-memory ``indexed`` set has evicted it, so a long retention window never re-derives (``ros2 bag
    info``) old bags each poll. ``indexed`` stays a within-run fast path that avoids a manifest query
    per already-seen bag. A bag that FAILED to index is in neither the manifest nor ``indexed``, so it
    still retries on a later poll. (Trade-off: a same-named bag is not auto-re-indexed by the loop —
    fine, since bag names are timestamped; a deliberate re-index can still call ``index`` directly.)
    """
    for bag in _iter_bag_dirs(watch_dir):
        sidecar = _sidecar_for(bag)
        # The sidecar must be a real file: a planted symlink in the landing dir (whose writers are
        # remote by design) would redirect the read outside the tree (Mira, review 4728294643). The
        # bag dir's own layout is validated by IngestService.index's shared guard, not here.
        if bag in indexed or not is_regular_file(sidecar):
            continue
        seen = _already_indexed_safe(service, bag)
        if seen is None:  # dedup check hit a caught fault → skip this poll, retry later
            continue
        if seen:
            indexed.add(bag)  # remember it this run so we don't re-query the manifest every poll
            continue
        if _try_index(service, bag, sidecar):
            indexed.add(bag)


def _already_indexed_safe(service: IngestService, bag: Path) -> bool | None:
    """``already_indexed`` under the same fault boundary as indexing; None if a caught fault fired.

    ``already_indexed`` hits the manifest (SQLite), so a transient ``OperationalError`` (a locked DB
    from a concurrent reader/writer) must be caught HERE too — not just around ``index`` — or it
    escapes ``_drain_once`` and terminates the watch loop (Mira F-02). A ``None`` return tells the
    caller to skip this bag for now; it is retried on a later poll.
    """
    try:
        return service.already_indexed(bag)
    except _INGEST_FAULTS:
        logger.exception(
            "manifest dedup check failed for %s (will retry on a later poll)", bag.name
        )
        return None


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ingest", description=__doc__)
    parser.add_argument("--watch", required=True, type=Path, help="DGX landing dir to index")
    parser.add_argument("--db", required=True, type=Path, help="manifest SQLite path")
    parser.add_argument("--poll-interval", type=positive_interval, default=_POLL_INTERVAL_S)
    return parser.parse_args(argv)


def _watch_loop(service: IngestService, watch_dir: Path, poll_interval: float) -> None:
    """Poll ``watch_dir``; index each finalized bag dir once its sidecar is present.

    Fault-tolerant: a bag that fails to index (missing/corrupt input, malformed sidecar) is logged
    and skipped, never added to ``indexed`` — so it retries on a later poll once corrected and one
    bad bag can't starve the rest. A clean re-index of the same bag is idempotent (store-keyed). The
    ``indexed`` set is LRU-bounded (F-09) so a long-lived daemon can't grow it without limit.
    """
    indexed = _BoundedSeen()
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
