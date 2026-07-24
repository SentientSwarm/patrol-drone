"""Re-export of the shared ``_BoundedSeen`` for the ingest container (M8 / F-09, PR #16 / F-03).

The canonical implementation now lives in the neutral top-level ``_shared.bounded_seen`` module
(F-03 moved it there so the upload daemon imports it without a cross-tree ``sys.path`` hop). This
thin re-export keeps ``ingest.bounded_seen`` importable inside the ingest container — the Dockerfile
``COPY``s both ``docker/ingest/`` and ``analysis/_shared/`` onto ``/opt/ingest`` so this ``from
_shared.bounded_seen import …`` resolves there without ``analysis/`` on the image path.
"""

from __future__ import annotations

from _shared.bounded_seen import _BoundedSeen

__all__ = ["_BoundedSeen"]
