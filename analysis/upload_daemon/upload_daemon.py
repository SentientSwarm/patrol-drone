"""The dumb dev-host upload daemon (design §4.2.3, SWM-74 / T8.2).

Watches the recorder's bag-output directory and, for each *completed* bag, transfers the bag and
its sidecar to the configured target via a :class:`~upload_daemon.transport.Transport`. It does
nothing else — no indexing, no parsing, no fact derivation (the dumb-producer invariant, design
§3.4); all of that happens DGX-side in the ingest service.

"Completed" is an atomic marker: a finalized bag dir carrying a real ``.mcap`` payload AND its
``<bag>.meta.json`` sidecar (the shared ``_shared.bag_layout`` validator, also applied by ingest).
A bag without its sidecar (recorder killed mid-run) is never shipped (§4.4.5). On transfer
failure the daemon retries with backoff and leaves the bag on disk — the producer is the source of
truth until a transfer is confirmed, so no data is ever lost to a flaky link.
"""

from __future__ import annotations

import json
import os
import time
from pathlib import Path

from _shared.bag_layout import is_regular_file, is_valid_bag_dir
from upload_daemon.transport import Transport


def sidecar_path_for(bag_path: Path) -> Path:
    """Return the sidecar path the recorder writes beside the bag dir (``<bag>.meta.json``)."""
    return bag_path.with_name(bag_path.name + ".meta.json")


def iter_bag_dirs(watch_dir: Path) -> list[Path]:
    """Return the candidate bag directories directly under ``watch_dir`` (sorted, deterministic).

    rosbag2 writes each run as its own directory; the upload + ingest loops both poll for these.
    Completeness is decided per-dir by :func:`is_complete` (the shared bag-layout validator),
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
    """A bag is complete iff it is a VALID bag dir AND its sidecar is present (the upload marker).

    "Valid" is the shared :func:`~_shared.bag_layout.is_valid_bag_dir` predicate — a real directory
    holding a real ``metadata.yaml`` AND at least one real ``.mcap`` payload, symlinks refused at
    every component. Requiring the PAYLOAD, not just the finalize marker, is what stops a
    metadata-only directory — a recorder killed before the MCAP flushed, or a half-landed rsync —
    from shipping and then indexing as a manifest row for an unreplayable bag (Mira High, review
    4752923085). The ingest boundary applies the same predicate, so the two cannot drift.
    """
    return is_valid_bag_dir(bag_path) and is_regular_file(sidecar_path_for(bag_path))


def upload_marker_for(bag_path: Path) -> Path:
    """The LOCAL marker written beside a bag once its transfer is confirmed (``<bag>.uploaded``)."""
    return bag_path.with_name(bag_path.name + ".uploaded")


# The receipt schema the marker carries so a durable skip is bound to a real transfer, not a bare
# sentinel. A stale/pre-planted EMPTY or malformed marker fails this parse and is treated as "not
# uploaded" (retry) — closing the false-success a `touch`-ed or backup-restored marker would cause
# (F-02, review 4748505221), the stale-plain-marker counterpart to the symlink hardening.
_RECEIPT_TARGET_KEY = "target"
_RECEIPT_FILES_KEY = "files"


def _canonical_target(target: str) -> str:
    """Normalize a transfer target so the write and read sides of the receipt cannot drift (F-02).

    ``--target`` is raw operator input: ``dgx:/data/bags`` and ``dgx:/data/bags/`` name the SAME
    destination but serialize differently, so a literal string compare would re-upload every bag on a
    trailing-slash edit. Normalizing once — here, used by BOTH :func:`_receipt_bytes` and
    :func:`_is_valid_receipt` — keeps the two in lockstep. Deliberately light (whitespace + trailing
    separators): this is a comparison key, not a URL parser.
    """
    stripped = target.strip()
    return stripped.rstrip("/") or stripped  # "/" normalizes to itself, not to ""


def _receipt_bytes(bag_path: Path, target: str) -> bytes:
    """The JSON transfer receipt written into the marker on a confirmed transfer.

    Records the CANONICAL transfer target and the covered file names (bag dir + sidecar), so
    :func:`is_already_uploaded` can validate the marker describes *this* bag's transfer *to this
    destination* rather than trusting a bare sentinel. Kept to structural fields (no digest) — enough
    to defeat an empty ``touch``-ed or backup-restored marker while staying dependency-light.
    """
    receipt = {
        _RECEIPT_TARGET_KEY: _canonical_target(target),
        _RECEIPT_FILES_KEY: sorted([bag_path.name, sidecar_path_for(bag_path).name]),
    }
    return json.dumps(receipt, sort_keys=True).encode()


def _write_upload_marker(bag_path: Path, target: str) -> None:
    """Write the LOCAL <bag>.uploaded RECEIPT the daemon exclusively owns (F-04 + F-02).

    O_CREAT|O_EXCL|O_NOFOLLOW so a pre-planted file OR symlink at the marker path fails the create
    loudly (the daemon never adopts a marker it didn't make); mode 0o600 keeps it owner-only. The
    body is a JSON receipt (target + covered filenames) so the marker is bound to a real transfer,
    not an empty sentinel — an empty/malformed marker no longer reads as "uploaded". This is the
    write-side pair to is_already_uploaded's symlink-strict + receipt-validated read.
    """
    marker = upload_marker_for(bag_path)
    fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    try:
        os.write(fd, _receipt_bytes(bag_path, target))
    finally:
        os.close(fd)


def _is_valid_receipt(marker: Path, bag_path: Path, target: str) -> bool:
    """True iff ``marker`` is a well-formed receipt covering ``bag_path`` sent to ``target``.

    Three gates, all required: parseable JSON object; the recorded target equals the canonical form
    of the CONFIGURED target (F-02, review 4752923085 — a receipt for a previous ``--target`` must
    not suppress the transfer to a new one); and ``files`` names this bag dir plus its sidecar. Any
    miss returns False → the bag reads as NOT uploaded and is retried, the same conservative
    direction the existing malformed-receipt path already takes. A non-string ``target`` value simply
    fails the equality, so no separate type check is needed.
    """
    try:
        receipt = json.loads(marker.read_text())
    except (OSError, ValueError):
        return False
    if not isinstance(receipt, dict):
        return False
    if receipt.get(_RECEIPT_TARGET_KEY) != _canonical_target(target):
        return False
    return receipt.get(_RECEIPT_FILES_KEY) == sorted(
        [bag_path.name, sidecar_path_for(bag_path).name]
    )


def is_already_uploaded(bag_path: Path, target: str) -> bool:
    """True iff a VALID receipt for ``bag_path`` **and** ``target`` exists (the durable skip).

    Survives the in-memory LRU's eviction (F-05), so an old bag with its receipt is skipped forever
    rather than re-rsynced every poll. Three gates, closing three false-success vectors together:
      * symlink-strict — a planted symlink named ``<bag>.uploaded`` is refused (``is_regular_file``),
        so a symlink-following read can't mark a never-shipped bag "uploaded" (F-04, review
        4731322384).
      * receipt-validated — the body must be a well-formed JSON receipt naming *this* bag + sidecar,
        so an empty ``touch``-ed or backup-restored marker no longer suppresses the transfer (F-02,
        review 4748505221).
      * target-matched — the receipt's canonical target must equal the CONFIGURED one, so repointing
        the daemon at a new DGX re-ships every bag still on disk instead of skipping it forever
        (F-02, review 4752923085).
    The daemon trusts ONLY a real, non-symlink marker it wrote via ``_write_upload_marker``.
    """
    marker = upload_marker_for(bag_path)
    return is_regular_file(marker) and _is_valid_receipt(marker, bag_path, target)


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

    def is_already_uploaded(self, bag_path: Path) -> bool:
        """True iff a valid receipt for ``bag_path`` names THIS daemon's configured target (F-02).

        The watch loop's durable skip, bound to ``self._target`` — the call site had no destination
        to compare with before, which is why a receipt written for a previous ``--target``
        suppressed the transfer to a new one forever (Mira Medium, review 4752923085). Delegates to
        the module-level :func:`is_already_uploaded` (a method body resolves names at module scope,
        so this is the free function, not recursion).
        """
        return is_already_uploaded(bag_path, self._target)

    def on_bag_complete(self, bag_path: Path) -> bool:
        """Ship ``bag_path`` (+ sidecar) if complete; return True only on a confirmed transfer.

        Guard: a real MCAP payload, its finalize marker and the sidecar must all be present, else the
        bag is skipped (returns False) and left untouched. On a complete bag, the bag then the
        sidecar are each sent with retry/backoff; the bag stays on disk regardless (deletion is not
        this daemon's job).
        """
        if not is_complete(bag_path):
            return False

        if not all(self._send_with_retry(path) for path in (bag_path, sidecar_path_for(bag_path))):
            return False
        # LOCAL-only durable 'uploaded' signal (F-05); not shipped. Exclusive create (F-04): a
        # pre-planted marker raises FileExistsError rather than being silently adopted — the driver
        # loop's _UPLOAD_FAULTS boundary (⊇ OSError) logs it and leaves the bag un-marked to retry.
        _write_upload_marker(bag_path, self._target)
        return True

    def _send_with_retry(self, path: Path) -> bool:
        """Send ``path`` to the target, retrying up to ``max_retries`` times with backoff."""
        for attempt in range(self._max_retries + 1):
            if self._transport.send(path, self._target):
                return True
            if attempt < self._max_retries:
                time.sleep(self._backoff_s)
        return False
