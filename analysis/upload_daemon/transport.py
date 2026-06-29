"""Pluggable transfer mechanism for the upload daemon (design §4.2.3a, SWM-73 / T8.1).

A :class:`Transport` knows only how to ``send`` one local path to one remote path and report
success — it knows nothing about bags, sidecars, or the manifest. This keeps the producer dumb
(design §3.4) and lets CI swap a local stand-in target for the real DGX (OQ-7).

:class:`RsyncSshTransport` is the Phase-1 default (rsync over SSH; resumable, dependency-light,
OQ-8). :class:`S3Transport` is an interface-parity stub for the OQ-8 S3 alternative — present so a
future phase can drop it in, but **not implemented** in Phase 1; calling it fails loudly.
"""

from __future__ import annotations

import subprocess
from collections.abc import Callable
from pathlib import Path
from typing import Protocol, runtime_checkable

# A runner abstracts the subprocess call so unit tests can inject a fake and stay host-independent.
# It takes the argv and returns the process return code.
Runner = Callable[[list[str]], int]


def _default_runner(argv: list[str]) -> int:
    """Run ``argv`` and return its exit code (no shell, output inherited)."""
    return subprocess.run(argv, check=False).returncode


@runtime_checkable
class Transport(Protocol):
    """The single contract every transport satisfies: copy ``local_path`` to ``remote_path``."""

    def send(self, local_path: Path, remote_path: str) -> bool:
        """Return True on a confirmed transfer, False on a recoverable failure."""
        ...


class RsyncSshTransport:
    """rsync ``-a`` over SSH — the Phase-1 default transport (OQ-8).

    Archive mode (``-a``) preserves metadata and makes repeated sends resumable/idempotent, so a
    retry after a partial transfer completes rather than re-copies. The transfer succeeds iff rsync
    exits 0.
    """

    def __init__(self, runner: Runner = _default_runner) -> None:
        self._runner = runner

    def send(self, local_path: Path, remote_path: str) -> bool:
        argv = ["rsync", "-a", str(local_path), remote_path]
        return self._runner(argv) == 0


class S3Transport:
    """OQ-8 S3-compatible alternative — interface parity only, NOT implemented in Phase 1.

    Present so the daemon's ``--transport`` selection has a real type to name and a future phase can
    implement it without reshaping the daemon. Until then it fails loudly rather than silently
    no-op'ing a transfer the operator believes succeeded.
    """

    def send(self, local_path: Path, remote_path: str) -> bool:
        raise NotImplementedError(
            "S3Transport is an OQ-8 parity stub, not implemented in Phase 1 "
            "(rsync/SSH is the shipping transport)."
        )
