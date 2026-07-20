"""CLI entry point for the upload daemon: ``python -m upload_daemon`` (design §4.2.3 operator surface).

Thin watch-and-dispatch wrapper around the ROS-free :class:`~upload_daemon.upload_daemon.UploadDaemon`
core (which holds the tested logic). This module is the I/O shell — argparse + a poll loop over the
watched directory — analogous to a ROS launch file: it carries no first-party logic worth a Layer-A
unit test, and is exercised by the stand-in integration test instead.

Usage (from the repo root — ``upload_daemon`` lives under ``analysis/``, so put it on the path):
    PYTHONPATH=analysis python -m upload_daemon \
        --watch ~/patrol_bags --target dgx:/data/bags/ [--transport rsync|s3]
"""

from __future__ import annotations

import argparse
import logging
import time
from pathlib import Path

from _shared.bounded_seen import _BoundedSeen
from _shared.positive_interval import positive_interval
from upload_daemon.transport import RsyncSshTransport, Transport
from upload_daemon.upload_daemon import (
    UploadDaemon,
    is_already_uploaded,
    is_complete,
    iter_bag_dirs,
)

logger = logging.getLogger("upload_daemon")

# _BoundedSeen now lives in the neutral top-level `_shared.bounded_seen` module (PR #16 / F-03), so
# the daemon imports it normally — no cross-tree `sys.path` hop into `docker/ingest/`. Re-exported so
# `from upload_daemon.__main__ import _BoundedSeen` still resolves for the existing unit test.
__all__ = ["_BoundedSeen", "main"]

_POLL_INTERVAL_S = 5.0

# The recoverable transport faults a per-bag upload can raise. The watch loop catches exactly these
# (not bare Exception) so one bad bag is logged + retried on a later poll instead of crashing the
# long-running daemon, while a genuine programming bug still surfaces. Mirrors ingest's _INGEST_FAULTS.
# NotImplementedError is deliberately NOT here: it's a PERMANENT condition (an unimplemented
# transport), not a transient transport fault, so it must abort at startup — never retry forever
# (F-03). Only genuinely-transient faults belong in this set.
_UPLOAD_FAULTS = (
    OSError,  # rsync binary absent (FileNotFoundError), SSH/socket failure, etc.
)


def _make_transport(kind: str) -> Transport:
    """Resolve the ``--transport`` flag to a concrete Transport (rsync ships; s3 is a stub).

    ``s3`` is a selectable name for interface parity but is unimplemented in Phase 1, so selecting it
    aborts here with an actionable message rather than entering the watch loop and infinite-retrying
    an ``S3Transport.send`` that only raises (F-03).
    """
    if kind == "rsync":
        return RsyncSshTransport()
    if kind == "s3":
        raise SystemExit(
            "--transport s3 is an OQ-8 parity stub, not implemented in Phase 1; "
            "use --transport rsync (the shipping transport)."
        )
    raise SystemExit(f"unknown --transport {kind!r} (expected 'rsync' or 's3')")


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="upload_daemon", description=__doc__)
    parser.add_argument("--watch", required=True, type=Path, help="bag output directory to watch")
    parser.add_argument("--target", required=True, help="rsync/SSH dest or local stand-in dir")
    parser.add_argument("--transport", default="rsync", choices=("rsync", "s3"))
    parser.add_argument("--poll-interval", type=positive_interval, default=_POLL_INTERVAL_S)
    return parser.parse_args(argv)


def _try_upload(daemon: UploadDaemon, bag: Path) -> bool:
    """Upload one bag; on a known transport fault log at ERROR and return False (retry next poll)."""
    try:
        return daemon.on_bag_complete(bag)
    except _UPLOAD_FAULTS:
        logger.exception("skipping un-uploadable bag %s (will retry on a later poll)", bag.name)
        return False


def _drain_once(daemon: UploadDaemon, watch_dir: Path, uploaded: _BoundedSeen) -> None:
    """One discovery+upload pass over ``watch_dir`` (mutates ``uploaded`` with the freshly uploaded).

    'Already handled' is durable via a per-bag ``<bag>.uploaded`` marker (F-05): a bag with one is
    skipped even after the in-memory LRU has evicted it, so a long retention window never re-rsyncs
    old bags each poll. ``uploaded`` stays a within-run fast path that avoids a stat() per already-seen
    bag. A bag whose transfer failed has no marker, so it correctly retries on a later poll.
    """
    for bag in iter_bag_dirs(watch_dir):
        if bag in uploaded or not is_complete(bag):
            continue
        if is_already_uploaded(bag):
            uploaded.add(
                bag
            )  # durable skip: remember it this run, don't re-stat the marker each poll
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
