"""Re-export of the shared bag-layout validator for the ingest container (PR #16 / F-01).

The canonical implementation lives in the neutral top-level ``_shared.bag_layout`` module (shared
with the upload daemon, so the two boundaries cannot drift apart on "is this a real bag"). This thin
re-export keeps ``ingest.bag_layout`` importable inside the ingest container — the Dockerfile
``COPY``s both ``docker/ingest/`` and ``analysis/_shared/`` onto ``/opt/ingest`` so this
``from _shared.bag_layout import …`` resolves there without ``analysis/`` on the image path —
mirroring ``ingest.bounded_seen`` and ``ingest.positive_interval``.
"""

from __future__ import annotations

from _shared.bag_layout import is_regular_file, is_valid_bag_dir

__all__ = ["is_regular_file", "is_valid_bag_dir"]
