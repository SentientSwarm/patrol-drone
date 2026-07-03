"""Neutral shared helpers for the M8 replay pipeline (design §4.2.3/§4.2.4).

A dependency-free home for code both the dev-host upload daemon (``analysis/upload_daemon``) and the
DGX ingest service (``docker/ingest``) need, so neither has to reach across the other's tree. Homed
under ``analysis/`` (already on the test pythonpath and the daemon's runtime path); the ingest
container ships it via its own ``COPY analysis/_shared/`` onto ``/opt/ingest`` (PR #16 / F-03).
"""
