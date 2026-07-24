"""Layer-A unit tests for the ROS-free upload Transport (docset 05-logging-replay, M8 / T8.1).

Covers `upload_daemon.transport` — the pluggable transfer mechanism the UploadDaemon uses to move
a finished bag (+ its sidecar) from the dev host to the DGX (design §4.2.3a, SWM-73):

  * ``RsyncSshTransport.send`` — the Phase-1 default (rsync over SSH). Builds the ``rsync`` argv
    and reports success/failure from the subprocess return code; the actual rsync is mocked here so
    the test stays ROS-free and host-independent (DoD AC-3 transport half).
  * ``S3Transport`` — the deferred OQ-8 parity stub: it satisfies the ``Transport`` protocol but is
    explicitly NOT implemented in Phase 1; calling ``send`` must fail loudly, never silently no-op.

The real rsync/SSH round-trip (a bag actually landing on a target) is the integration concern,
exercised against a local stand-in in test_upload_daemon / the stand-in integration test — not here.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from upload_daemon.transport import (
    _TRANSFER_TIMEOUT_S,
    RsyncSshTransport,
    S3Transport,
    Transport,
)


class _RecordingRunner:
    """A stand-in for the real runner: records the argv + timeout and returns a chosen return code."""

    def __init__(self, returncode: int) -> None:
        self.returncode = returncode
        self.calls: list[list[str]] = []
        self.timeouts: list[float] = []

    def __call__(self, argv: list[str], timeout_s: float) -> int:
        self.calls.append(list(argv))
        self.timeouts.append(timeout_s)
        return self.returncode


class _TimingOutRunner:
    """A runner that always expires — models a wedged rsync/SSH the timeout had to kill (F-03)."""

    def __call__(self, argv: list[str], timeout_s: float) -> int:
        raise subprocess.TimeoutExpired(cmd=argv, timeout=timeout_s)


# TS-1: RsyncSshTransport.send copies a file to target; returns True on success.
def test_rsync_send_returns_true_on_success() -> None:
    runner = _RecordingRunner(returncode=0)
    transport = RsyncSshTransport(runner=runner)

    ok = transport.send(Path("/bags/patrol_x_0.mcap"), "dgx:/data/bags/")

    assert ok is True
    assert len(runner.calls) == 1
    argv = runner.calls[0]
    assert argv[0] == "rsync"
    assert "/bags/patrol_x_0.mcap" in argv
    assert "dgx:/data/bags/" in argv


# F-04: the rsync argv includes a `--` option terminator immediately before the source path, so an
# option-looking path (e.g. "--rsync-path=sh") is treated as a file, not an rsync flag.
def test_rsync_send_inserts_option_terminator() -> None:
    runner = _RecordingRunner(returncode=0)
    transport = RsyncSshTransport(runner=runner)

    transport.send(Path("--rsync-path=sh"), "dgx:/data/bags/")

    argv = runner.calls[0]
    assert "--" in argv
    assert argv.index("--") == argv.index("--rsync-path=sh") - 1


# TS-1: the rsync argv is resumable/archive (rsync -a) — the design's "resumable, dependency-light".
def test_rsync_send_uses_archive_flag() -> None:
    runner = _RecordingRunner(returncode=0)
    transport = RsyncSshTransport(runner=runner)

    transport.send(Path("/bags/b.mcap"), "dgx:/data/")

    argv = runner.calls[0]
    assert any(flag == "-a" or flag.startswith("-a") for flag in argv), argv


# TS-2: Transport.send returns False on transfer failure (non-zero rsync exit).
def test_rsync_send_returns_false_on_failure() -> None:
    runner = _RecordingRunner(returncode=23)  # rsync partial-transfer error
    transport = RsyncSshTransport(runner=runner)

    ok = transport.send(Path("/bags/b.mcap"), "dgx:/data/")

    assert ok is False


# F-03 (Mira Medium, review 4752923085): the transfer is BOUNDED — the configured timeout reaches the
# runner, so a wedged rsync/SSH can be killed instead of parking the serial watch loop forever.
def test_rsync_send_passes_the_configured_timeout() -> None:
    runner = _RecordingRunner(returncode=0)
    transport = RsyncSshTransport(runner=runner, timeout_s=42.0)

    transport.send(Path("/bags/b.mcap"), "dgx:/data/")

    assert runner.timeouts == [42.0]


# F-03: the default is the module constant, so an operator who sets nothing still gets a bound.
def test_rsync_send_defaults_to_the_module_timeout() -> None:
    runner = _RecordingRunner(returncode=0)

    RsyncSshTransport(runner=runner).send(Path("/bags/b.mcap"), "dgx:/data/")

    assert runner.timeouts == [_TRANSFER_TIMEOUT_S]


# F-03: an expired transfer is a RECOVERABLE failure — reported exactly like a non-zero rsync exit
# (False), never propagated. That keeps retry semantics in ONE place: the daemon's _send_with_retry
# backoff handles it, rather than TimeoutExpired escaping into the watch loop and killing the daemon.
def test_rsync_send_reports_timeout_as_a_recoverable_failure() -> None:
    transport = RsyncSshTransport(runner=_TimingOutRunner())

    ok = transport.send(Path("/bags/b.mcap"), "dgx:/data/")

    assert ok is False


# RsyncSshTransport satisfies the Transport protocol.
def test_rsync_is_a_transport() -> None:
    assert isinstance(RsyncSshTransport(), Transport)


# S3Transport is a parity stub (OQ-8) — present but explicitly NOT implemented; send must fail loud.
def test_s3_transport_send_raises_not_implemented() -> None:
    transport = S3Transport()

    with pytest.raises(NotImplementedError):
        transport.send(Path("/bags/b.mcap"), "s3://bucket/bags/")
