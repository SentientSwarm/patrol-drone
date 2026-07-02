"""CLI entry point for the upload daemon: ``python -m upload_daemon`` (design §4.2.3 operator surface).

Thin watch-and-dispatch wrapper around the ROS-free :class:`~upload_daemon.upload_daemon.UploadDaemon`
core (which holds the tested logic). This module is the I/O shell — argparse + a poll loop over the
watched directory — analogous to a ROS launch file: it carries no first-party logic worth a Layer-A
unit test, and is exercised by the stand-in integration test instead.

Usage:
    python -m upload_daemon --watch ~/patrol_bags --target dgx:/data/bags/ [--transport rsync|s3]
"""

from __future__ import annotations

import argparse
import logging
import time
from collections import OrderedDict
from pathlib import Path

from upload_daemon.transport import RsyncSshTransport, S3Transport, Transport
from upload_daemon.upload_daemon import UploadDaemon, is_complete, iter_bag_dirs

logger = logging.getLogger("upload_daemon")

_POLL_INTERVAL_S = 5.0

# The recoverable transport faults a per-bag upload can raise. The watch loop catches exactly these
# (not bare Exception) so one bad bag is logged + retried on a later poll instead of crashing the
# long-running daemon, while a genuine programming bug still surfaces. Mirrors ingest's _INGEST_FAULTS.
_UPLOAD_FAULTS = (
    OSError,  # rsync binary absent (FileNotFoundError), SSH/socket failure, etc.
    NotImplementedError,  # the S3Transport parity stub raises this until Phase-1+ implements it
)


class _BoundedSeen:
    """A membership set with an LRU cap: tracks 'already handled' bags without growing unbounded.

    The watch loops only need 'have I already handled this bag this run?'. An unbounded set grows one
    entry per bag for the process lifetime (F-09); this caps it at ``maxlen``, evicting the
    oldest-added key when full. Eviction is safe because both handlers are idempotent (rsync -a /
    INSERT OR REPLACE) — a re-seen evicted bag is just a cheap redundant re-handle, never data loss.
    """

    def __init__(self, maxlen: int = 4096) -> None:
        self._seen: OrderedDict[Path, None] = OrderedDict()
        self._maxlen = maxlen

    def __contains__(self, bag: Path) -> bool:
        return bag in self._seen

    def add(self, bag: Path) -> None:
        self._seen[bag] = None
        self._seen.move_to_end(bag)
        while len(self._seen) > self._maxlen:
            self._seen.popitem(last=False)  # evict the oldest-added

    def __len__(self) -> int:
        return len(self._seen)


def _make_transport(kind: str) -> Transport:
    """Resolve the ``--transport`` flag to a concrete Transport (rsync ships; s3 is a stub)."""
    if kind == "rsync":
        return RsyncSshTransport()
    if kind == "s3":
        return S3Transport()
    raise SystemExit(f"unknown --transport {kind!r} (expected 'rsync' or 's3')")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="upload_daemon", description=__doc__)
    parser.add_argument("--watch", required=True, type=Path, help="bag output directory to watch")
    parser.add_argument("--target", required=True, help="rsync/SSH dest or local stand-in dir")
    parser.add_argument("--transport", default="rsync", choices=("rsync", "s3"))
    parser.add_argument("--poll-interval", type=float, default=_POLL_INTERVAL_S)
    return parser.parse_args(argv)


def _try_upload(daemon: UploadDaemon, bag: Path) -> bool:
    """Upload one bag; on a known transport fault log at ERROR and return False (retry next poll)."""
    try:
        return daemon.on_bag_complete(bag)
    except _UPLOAD_FAULTS:
        logger.exception("skipping un-uploadable bag %s (will retry on a later poll)", bag.name)
        return False


def _drain_once(daemon: UploadDaemon, watch_dir: Path, uploaded: _BoundedSeen) -> None:
    """One discovery+upload pass over ``watch_dir`` (mutates ``uploaded`` with the freshly uploaded)."""
    for bag in iter_bag_dirs(watch_dir):
        if bag in uploaded or not is_complete(bag):
            continue
        if _try_upload(daemon, bag):
            uploaded.add(bag)
            logger.info("uploaded %s", bag.name)


def _watch_loop(daemon: UploadDaemon, watch_dir: Path, poll_interval: float) -> None:
    """Poll ``watch_dir`` for newly-completed bag dirs and upload each one exactly once.

    Fault-tolerant (mirrors the ingest loop): a bag whose transfer *raises* a known transport fault
    is logged and skipped, never added to ``uploaded`` — so it retries on a later poll and one bad
    bag can't crash the daemon. (``on_bag_complete`` returning False already handles a clean
    retry-exhausted failure; this guards the raising path — a missing rsync binary, an SSH error,
    the S3 stub's NotImplementedError.) The ``uploaded`` set is LRU-bounded (F-09) so a long-lived
    daemon can't grow it without limit.
    """
    uploaded = _BoundedSeen()
    while True:
        _drain_once(daemon, watch_dir, uploaded)
        time.sleep(poll_interval)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s upload_daemon %(message)s")
    args = _parse_args(argv)
    daemon = UploadDaemon(transport=_make_transport(args.transport), target=args.target)
    logger.info("watching %s -> %s (%s)", args.watch, args.target, args.transport)
    _watch_loop(daemon, args.watch, args.poll_interval)


if __name__ == "__main__":
    main()
