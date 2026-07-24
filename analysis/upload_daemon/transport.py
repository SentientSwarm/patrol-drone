"""Pluggable transfer mechanism for the upload daemon (design §4.2.3a, SWM-73 / T8.1).

A :class:`Transport` knows only how to ``send`` one local path to one remote path and report
success — it knows nothing about bags, sidecars, or the manifest. This keeps the producer dumb
(design §3.4) and lets CI swap a local stand-in target for the real DGX (OQ-7).

:class:`RsyncSshTransport` is the Phase-1 default (rsync over SSH; resumable, dependency-light,
OQ-8). :class:`S3Transport` is an interface-parity stub for the OQ-8 S3 alternative — present so a
future phase can drop it in, but **not implemented** in Phase 1; calling it fails loudly.
"""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
from collections.abc import Callable
from pathlib import Path
from typing import Protocol, runtime_checkable

# A runner abstracts the subprocess call so unit tests can inject a fake and stay host-independent.
# It takes the argv plus the transfer timeout in seconds, and returns the process return code.
Runner = Callable[[list[str], float], int]

# A ~100 MiB bag is seconds over a LAN and minutes over a slow link, so the cap is generous: it
# exists to bound a WEDGED transfer (a half-open TCP connection to a rebooted DGX, a partition that
# drops rather than resets), not to police a slow one. Without it a stuck rsync parks inside
# Transport.send forever, so the daemon's retry/backoff — which sits OUTSIDE that call — never runs
# and every completed bag behind it stops being processed while the process still looks healthy
# (Mira Medium, review 4752923085). Mirrors bag_reader._BAG_INFO_TIMEOUT_S, the same guard on the
# other serial loop; overridable per-run via `--transfer-timeout`.
#
# SCOPE: this bounds ONE Transport.send, not the per-bag total. _send_with_retry makes
# max_retries+1 (=4) attempts and on_bag_complete sends two paths, so a fully wedged target still
# occupies the serial loop for up to ~4 x this value plus backoff (~2 h at the default). That is the
# deliberate trade: the cap is sized for a legitimate ~100 MiB transfer on a slow link (a real bag
# would need to average <30 KB/s to reach it), and the loop now MAKES PROGRESS at all, which it
# could not before. Operators on a fast link should lower --transfer-timeout.
_TRANSFER_TIMEOUT_S = 1800.0


def _kill_transfer_group(proc: subprocess.Popen[bytes]) -> None:
    """SIGKILL a timed-out transfer's whole process GROUP, then reap it.

    ``rsync -e ssh`` forks an ``ssh`` child, so killing only the direct child leaves the grandchild
    holding the connection open — which is the wedge we are trying to clear. The transfer is started
    with ``start_new_session=True`` so it owns its group and the whole tree is reclaimed in one call
    (the same process-group discipline the record-path finalize fix uses for recorder teardown).
    SIGKILL rather than a SIGTERM ladder: the transfer already had its full timeout, and rsync has
    nothing to clean up — the producer keeps the bag until a transfer is CONFIRMED (§4.4.5).
    A group that is already gone (exited between the timeout and the signal) or not ours to signal is
    simply nothing left to reclaim, hence the suppress.
    """
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
    proc.wait()


def _default_runner(argv: list[str], timeout_s: float) -> int:
    """Run ``argv`` under a ``timeout_s`` cap and return its exit code (no shell, output inherited).

    Raises ``subprocess.TimeoutExpired`` on expiry, after reclaiming the process group; the caller
    treats that as a RETRYABLE failure (see :meth:`RsyncSshTransport.send`).
    """
    proc = subprocess.Popen(argv, start_new_session=True)
    try:
        return proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        _kill_transfer_group(proc)
        raise


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

    def __init__(
        self, runner: Runner = _default_runner, *, timeout_s: float = _TRANSFER_TIMEOUT_S
    ) -> None:
        self._runner = runner
        self._timeout_s = timeout_s

    def send(self, local_path: Path, remote_path: str) -> bool:
        # `--` terminates rsync option parsing so an option-looking path (e.g. one starting with `-`,
        # or `--rsync-path=…`) is treated as a source file, never a flag (PR #16 / F-04 hardening).
        argv = ["rsync", "-a", "--", str(local_path), remote_path]
        try:
            return self._runner(argv, self._timeout_s) == 0
        except subprocess.TimeoutExpired:
            # A wedged transfer is a RECOVERABLE failure, reported exactly like a non-zero rsync exit
            # so the daemon's existing _send_with_retry backoff handles it. Deliberate choice between
            # the review's two options: retry semantics stay in ONE place (the daemon), rather than
            # being split between the transport and the watch loop's fault set.
            return False


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
