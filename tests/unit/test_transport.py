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

from pathlib import Path

import pytest

from upload_daemon.transport import RsyncSshTransport, S3Transport, Transport


class _RecordingRunner:
    """A stand-in for subprocess.run that records the argv and returns a chosen return code."""

    def __init__(self, returncode: int) -> None:
        self.returncode = returncode
        self.calls: list[list[str]] = []

    def __call__(self, argv: list[str]) -> int:
        self.calls.append(list(argv))
        return self.returncode


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


# RsyncSshTransport satisfies the Transport protocol.
def test_rsync_is_a_transport() -> None:
    assert isinstance(RsyncSshTransport(), Transport)


# S3Transport is a parity stub (OQ-8) — present but explicitly NOT implemented; send must fail loud.
def test_s3_transport_send_raises_not_implemented() -> None:
    transport = S3Transport()

    with pytest.raises(NotImplementedError):
        transport.send(Path("/bags/b.mcap"), "s3://bucket/bags/")
