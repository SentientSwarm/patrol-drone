"""Re-export of the shared ``positive_interval`` validator for the ingest container (PR #16 / F-05).

The canonical implementation lives in the neutral top-level ``_shared.positive_interval`` module
(shared with the upload daemon). This thin re-export keeps ``ingest.positive_interval`` importable
inside the ingest container — the Dockerfile ``COPY``s both ``docker/ingest/`` and
``analysis/_shared/`` onto ``/opt/ingest`` so this ``from _shared.positive_interval import …``
resolves there without ``analysis/`` on the image path — mirroring ``ingest.bounded_seen``.
"""

from __future__ import annotations

from _shared.positive_interval import positive_interval

__all__ = ["positive_interval"]
