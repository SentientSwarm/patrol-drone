"""The single bag-layout validator shared by the upload and ingest boundaries (PR #16 / F-01).

Three places used to decide "is this a real bag" — ``upload_daemon.is_complete``,
``ingest_service._require_finalized_bag`` and the ingest watch loop — and each keyed on
``metadata.yaml`` alone. A directory holding a plausible ``metadata.yaml`` (plus a sidecar) but NO
nested ``.mcap`` therefore passed every gate: it shipped, then ``bag_reader`` derived a duration and
a topic map from the *description* of a bag that isn't there, landing a fully-populated manifest row
for an unreplayable artifact (Mira High, review 4752923085). Two realistic paths in, no adversary
required: a recorder killed after ``metadata.yaml`` is written but before the MCAP is flushed, and a
partially-completed rsync into the landing dir where the small files land and the large one does not.

So a valid bag dir is a real directory carrying BOTH the finalize marker AND at least one real
``.mcap`` payload, with symlinks refused at every component (the existing symlink-strict posture,
Mira review 4728294643). Living in the neutral top-level ``_shared`` package — like ``bounded_seen``
and ``positive_interval`` — lets both deploy trees share ONE predicate: the ingest container COPYs
``analysis/_shared/`` alongside ``docker/ingest/``, and ``ingest.bag_layout`` re-exports it.
"""

from __future__ import annotations

from pathlib import Path

_FINALIZE_MARKER = "metadata.yaml"
_PAYLOAD_GLOB = "*.mcap"


def is_regular_file(path: Path) -> bool:
    """True for a plain file that is NOT a symlink — every boundary refuses redirects.

    A symlinked ``metadata.yaml``/sidecar/payload planted in a writable watch or landing dir would
    redirect reads (and the rsync source) outside the tree (Mira Medium, review 4728294643).
    """
    return path.is_file() and not path.is_symlink()


def has_finalize_marker(bag_path: Path) -> bool:
    """True iff the dir carries rosbag2's real (non-symlink) ``metadata.yaml`` finalize marker.

    rosbag2 drops ``metadata.yaml`` into the ``-o`` directory only on a clean finalize, so it marks
    finalization — but on its own it says nothing about the payload (see :func:`has_mcap_payload`).
    """
    return is_regular_file(bag_path / _FINALIZE_MARKER)


def has_mcap_payload(bag_path: Path) -> bool:
    """True iff the dir holds at least one real (non-symlink) ``.mcap`` — the payload itself.

    rosbag2 writes the storage files as ``<name>_<n>.mcap`` directly inside the bag dir, so a
    top-level scan is the whole contract. A symlinked ``.mcap`` does not count: it would redirect the
    transfer source (and the reader) outside the tree exactly like a symlinked marker would. An
    in-flight ``rsync -a`` payload is written under a temp name and renamed on completion, so a
    partial transfer is never named ``*.mcap`` and correctly fails this gate.
    """
    return any(is_regular_file(p) for p in bag_path.glob(_PAYLOAD_GLOB))


def is_valid_bag_dir(bag_path: Path) -> bool:
    """True iff ``bag_path`` is a real, finalized bag dir WITH a real MCAP payload (F-01).

    The one predicate both pipeline boundaries use, so "is this a real bag" cannot drift between
    them again. Never raises: a missing or symlinked path is simply not a valid bag.
    """
    if bag_path.is_symlink() or not bag_path.is_dir():
        return False
    return has_finalize_marker(bag_path) and has_mcap_payload(bag_path)
