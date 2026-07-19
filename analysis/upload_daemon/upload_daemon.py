"""The dumb dev-host upload daemon (design §4.2.3, SWM-74 / T8.2).

Watches the recorder's bag-output directory and, for each *completed* bag, transfers the bag and
its sidecar to the configured target via a :class:`~upload_daemon.transport.Transport`. It does
nothing else — no indexing, no parsing, no fact derivation (the dumb-producer invariant, design
§3.4); all of that happens DGX-side in the ingest service.

"Completed" is an atomic marker: a finalized ``.mcap`` AND its ``<bag>.meta.json`` sidecar both
present. A bag without its sidecar (recorder killed mid-run) is never shipped (§4.4.5). On transfer
failure the daemon retries with backoff and leaves the bag on disk — the producer is the source of
truth until a transfer is confirmed, so no data is ever lost to a flaky link.
"""

from __future__ import annotations

import time
from pathlib import Path

from upload_daemon.transport import Transport


def sidecar_path_for(bag_path: Path) -> Path:
    """Return the sidecar path the recorder writes beside the bag dir (``<bag>.meta.json``)."""
    return bag_path.with_name(bag_path.name + ".meta.json")


def _is_regular_file(path: Path) -> bool:
    """True for a plain file that is NOT a symlink — the watch boundary refuses redirects.

    A symlinked ``metadata.yaml``/sidecar planted in a writable watch dir would redirect reads (and
    the rsync source) outside the watched tree (Mira Medium, review 4728294643), so every
    completeness predicate insists on real files. Mirrored by the ingest counterpart
    (``docker/ingest/__main__``) so the two loops keep their documented symmetry.
    """
    return path.is_file() and not path.is_symlink()


def is_finalized_bag_dir(bag_path: Path) -> bool:
    """True iff ``bag_path`` is a rosbag2 bag directory rosbag2 has finalized.

    The M7 recorder writes each run as a directory ``<name>/`` (the ``ros2 bag record -o`` URI) with
    the MCAP nested inside; rosbag2 drops ``metadata.yaml`` into it only on a clean finalize. So the
    finalized-bag marker is the directory's ``metadata.yaml`` — not a flat ``<name>.mcap`` (which
    never exists at the watch-dir top level). The marker must be a real file (a symlinked
    ``metadata.yaml`` is refused). The same predicate is the ingest service's bag guard.
    """
    return _is_regular_file(bag_path / "metadata.yaml")


def iter_bag_dirs(watch_dir: Path) -> list[Path]:
    """Return the candidate bag directories directly under ``watch_dir`` (sorted, deterministic).

    rosbag2 writes each run as its own directory; the upload + ingest loops both poll for these.
    Completeness/finalization is decided per-dir by :func:`is_complete` / :func:`is_finalized_bag_dir`,
    not here — this only enumerates the candidates so both loops share one discovery rule. Returns
    empty if ``watch_dir`` doesn't exist yet (the daemon may start before the first recording run
    creates it), mirroring the ingest sibling ``docker/ingest/__main__._iter_bag_dirs``. Symlinked
    entries are refused (Mira Medium, review 4728294643): a symlink planted in the watch dir would
    redirect the transfer source outside the watched tree, so discovery admits only real
    directories that resolve inside the watch root.
    """
    if not watch_dir.is_dir():
        return []
    root = watch_dir.resolve()
    return sorted(
        p
        for p in watch_dir.iterdir()
        if p.is_dir() and not p.is_symlink() and p.resolve().is_relative_to(root)
    )


def is_complete(bag_path: Path) -> bool:
    """A bag is complete iff it is a finalized bag dir AND its sidecar both exist (upload marker).

    Refuses symlinks at every component (the bag dir itself, ``metadata.yaml``, the sidecar) even
    when called directly rather than via discovery — a symlinked bag is never shippable (Mira).
    """
    if bag_path.is_symlink():
        return False
    return is_finalized_bag_dir(bag_path) and _is_regular_file(sidecar_path_for(bag_path))


def upload_marker_for(bag_path: Path) -> Path:
    """The LOCAL marker written beside a bag once its transfer is confirmed (``<bag>.uploaded``)."""
    return bag_path.with_name(bag_path.name + ".uploaded")


def is_already_uploaded(bag_path: Path) -> bool:
    """True iff a confirmed-upload marker exists for ``bag_path`` (the watch loop's durable skip).

    Survives the in-memory LRU's eviction (F-05), so an old bag with its marker is skipped forever
    rather than re-rsynced every poll. The marker is written only on a CONFIRMED transfer, so a
    failed/partial upload leaves none and correctly retries.
    """
    return upload_marker_for(bag_path).is_file()


class UploadDaemon:
    """Transfers completed bags to the target; dumb watch-and-send only (LR-3)."""

    def __init__(
        self,
        transport: Transport,
        target: str,
        *,
        max_retries: int = 3,
        backoff_s: float = 2.0,
    ) -> None:
        self._transport = transport
        self._target = target
        self._max_retries = max_retries
        self._backoff_s = backoff_s

    def on_bag_complete(self, bag_path: Path) -> bool:
        """Ship ``bag_path`` (+ sidecar) if complete; return True only on a confirmed transfer.

        Guard: both the .mcap and its sidecar must be present, else the bag is skipped (returns
        False) and left untouched. On a complete bag, the bag then the sidecar are each sent with
        retry/backoff; the bag stays on disk regardless (deletion is not this daemon's job).
        """
        if not is_complete(bag_path):
            return False

        if not all(self._send_with_retry(path) for path in (bag_path, sidecar_path_for(bag_path))):
            return False
        upload_marker_for(
            bag_path
        ).touch()  # LOCAL-only durable 'uploaded' signal (F-05); not shipped
        return True

    def _send_with_retry(self, path: Path) -> bool:
        """Send ``path`` to the target, retrying up to ``max_retries`` times with backoff."""
        for attempt in range(self._max_retries + 1):
            if self._transport.send(path, self._target):
                return True
            if attempt < self._max_retries:
                time.sleep(self._backoff_s)
        return False
