"""Layer-A unit tests for the upload watch-loop fault tolerance (M8 / F-08, design §4.4.5).

`upload_daemon.__main__._try_upload` wraps `UploadDaemon.on_bag_complete` in `except _UPLOAD_FAULTS`
so a bag whose transport *raises* a transient fault (rsync binary absent, SSH failure) is skipped +
retried instead of crashing the long-running daemon — symmetric with the ingest loop's `_try_index`.
`__main__.py` is a coverage-omitted I/O shell, but the _UPLOAD_FAULTS membership + the skip-not-crash
behaviour is load-bearing first-party logic. NotImplementedError is deliberately NOT a documented
upload fault (F-03): the S3 stub is unimplemented, a permanent condition, so `--transport s3` aborts
at startup rather than being caught + retried forever.

The daemon is driven for real (not faked) via a transport whose ``send`` raises — so the actual
``on_bag_complete`` → ``_send_with_retry`` → ``transport.send`` path is what raises the fault.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from upload_daemon.__main__ import (
    _UPLOAD_FAULTS,
    _BoundedSeen,
    _drain_once,
    _make_transport,
    _try_upload,
    main,
)
from upload_daemon.transport import RsyncSshTransport
from upload_daemon.upload_daemon import UploadDaemon, upload_marker_for


class _RaisingTransport:
    """A Transport whose send always raises — models a missing rsync binary / SSH failure / S3 stub."""

    def __init__(self, exc: BaseException) -> None:
        self._exc = exc

    def send(self, _local_path: Path, _remote_path: str) -> bool:
        raise self._exc


def _daemon_that_raises(exc: BaseException) -> UploadDaemon:
    return UploadDaemon(transport=_RaisingTransport(exc), target="dgx:/data/bags/")


def _make_complete_bag(tmp_path: Path) -> Path:
    """A finalized bag dir (metadata.yaml) + its sidecar — so is_complete() passes and send is reached."""
    bag = tmp_path / "patrol_x_20260629_120000"
    bag.mkdir()
    (bag / "patrol_x_20260629_120000_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    (tmp_path / "patrol_x_20260629_120000.meta.json").write_text("{}")
    return bag


@pytest.mark.parametrize(
    "exc",
    [
        FileNotFoundError("rsync: command not found"),  # OSError subclass — missing binary
        OSError("ssh: connect to host ... : Connection refused"),
    ],
)
def test_try_upload_skips_transport_fault_without_propagating(
    tmp_path: Path, exc: BaseException
) -> None:
    bag = _make_complete_bag(tmp_path)
    assert _try_upload(_daemon_that_raises(exc), bag) is False  # skipped, did NOT crash


def test_transport_faults_are_in_documented_upload_fault_set() -> None:
    assert OSError in _UPLOAD_FAULTS
    # F-03: NotImplementedError is NOT a transient transport fault — a stub must abort at startup,
    # never be caught and retried forever. It is deliberately absent from the retryable-fault set.
    assert NotImplementedError not in _UPLOAD_FAULTS


def test_make_transport_rsync_returns_rsync() -> None:
    assert isinstance(_make_transport("rsync"), RsyncSshTransport)


def test_make_transport_s3_aborts_rather_than_constructing_a_stub() -> None:
    # F-03: selecting the unimplemented s3 transport must fail fast (SystemExit) at construction —
    # not return an S3Transport whose send() raises NotImplementedError inside the retry loop forever.
    with pytest.raises(SystemExit):
        _make_transport("s3")


def test_main_transport_s3_exits_without_entering_the_watch_loop(tmp_path: Path) -> None:
    # F-03: `--transport s3` exits non-zero before the poll loop (no time.sleep to mock — never
    # reached), so an operator who selects the stub gets an immediate abort, not a silent busy-retry.
    with pytest.raises(SystemExit):
        main(["--watch", str(tmp_path), "--target", "dgx:/data/bags/", "--transport", "s3"])


def test_drain_once_leaves_a_faulting_bag_out_of_uploaded_for_retry(tmp_path: Path) -> None:
    # A complete bag whose upload raises must NOT be marked uploaded (so it retries next poll).
    _make_complete_bag(tmp_path)

    uploaded = _BoundedSeen()
    _drain_once(_daemon_that_raises(OSError("boom")), tmp_path, uploaded)

    assert len(uploaded) == 0  # nothing marked done → retried on the next poll


# F-05: a complete bag that ALREADY has a <bag>.uploaded marker is skipped by _drain_once — the
# transport is never touched (a raising daemon would blow up if it were), so retention-scale bags
# aren't re-rsynced each poll even after the in-memory seen-set evicts them.
def test_drain_once_skips_bag_with_upload_marker(tmp_path: Path) -> None:
    bag = _make_complete_bag(tmp_path)
    upload_marker_for(
        bag
    ).touch()  # durable 'already uploaded' signal from a prior confirmed transfer

    uploaded = _BoundedSeen()
    _drain_once(_daemon_that_raises(OSError("must not be called")), tmp_path, uploaded)  # no raise

    assert bag in uploaded  # recorded as handled this run, without re-uploading
