"""Re-export of the shared argparse validators for the ingest container (PR #16 / F-05, F-04).

The canonical implementations live in the neutral top-level ``_shared.positive_interval`` module
(shared with the upload daemon). This thin re-export keeps ``ingest.positive_interval`` importable
inside the ingest container — the Dockerfile ``COPY``s both ``docker/ingest/`` and
``analysis/_shared/`` onto ``/opt/ingest`` so this ``from _shared.positive_interval import …``
resolves there without ``analysis/`` on the image path — mirroring ``ingest.bounded_seen``.

``positive_int`` rides the same shim for ``manifest_query --recent`` (F-04).
"""

from __future__ import annotations

from _shared.positive_interval import positive_int, positive_interval

__all__ = ["positive_int", "positive_interval"]
