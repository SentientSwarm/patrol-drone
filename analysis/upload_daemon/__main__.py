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
from pathlib import Path

from upload_daemon.transport import RsyncSshTransport, S3Transport, Transport
from upload_daemon.upload_daemon import UploadDaemon, is_complete

logger = logging.getLogger("upload_daemon")

_POLL_INTERVAL_S = 5.0


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


def _watch_loop(daemon: UploadDaemon, watch_dir: Path, poll_interval: float) -> None:
    """Poll ``watch_dir`` for newly-completed bags and upload each one exactly once."""
    uploaded: set[Path] = set()
    while True:
        for bag in sorted(watch_dir.glob("*.mcap")):
            if bag in uploaded or not is_complete(bag):
                continue
            if daemon.on_bag_complete(bag):
                uploaded.add(bag)
                logger.info("uploaded %s", bag.name)
        time.sleep(poll_interval)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s upload_daemon %(message)s")
    args = _parse_args(argv)
    daemon = UploadDaemon(transport=_make_transport(args.transport), target=args.target)
    logger.info("watching %s -> %s (%s)", args.watch, args.target, args.transport)
    _watch_loop(daemon, args.watch, args.poll_interval)


if __name__ == "__main__":
    main()
