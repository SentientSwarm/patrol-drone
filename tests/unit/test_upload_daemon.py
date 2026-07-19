"""Layer-A unit tests for the ROS-free UploadDaemon core (docset 05-logging-replay, M8 / T8.2).

Covers `upload_daemon.upload_daemon.UploadDaemon` — the dumb dev-host producer that watches the
recorder's output dir and ships each *completed* bag to the DGX (design §4.2.3, SWM-74):

  * the "complete" guard — a bag is only shipped when BOTH the finalized ``.mcap`` AND its
    ``<bag>.meta.json`` sidecar are present (the atomic completion marker, design §4.4.5). A bag
    without its sidecar (recorder killed mid-run) is never uploaded (DoD AC-3).
  * the transfer — on a complete bag, both the bag and the sidecar are sent via the injected
    :class:`~upload_daemon.transport.Transport`, and never anything else (dumb producer, §3.4).
  * retry/backoff — a failed transfer is retried; the bag stays on disk until the send is confirmed
    (never deleted before confirmation — §4.4.5 network-dependency recovery).

The real filesystem watch loop and the ≤30 s wall-clock target are the integration concern
(stand-in integration test); here the daemon is driven directly via ``on_bag_complete`` with a
fake transport so the logic is host- and ROS-independent.
"""

from __future__ import annotations

from pathlib import Path

from upload_daemon.upload_daemon import UploadDaemon, iter_bag_dirs, upload_marker_for


class _FakeTransport:
    """Records every send and returns a scripted sequence of success/failure results."""

    def __init__(self, results: list[bool] | None = None) -> None:
        # Default: every send succeeds.
        self._results = list(results) if results is not None else []
        self.sent: list[tuple[str, str]] = []

    def send(self, local_path: Path, remote_path: str) -> bool:
        self.sent.append((str(local_path), remote_path))
        if self._results:
            return self._results.pop(0)
        return True


def _make_bag(tmp_path: Path, *, with_sidecar: bool, with_metadata: bool = True) -> Path:
    """Create a rosbag2 bag directory under tmp_path; return the bag-directory path.

    The recorder writes each run as a directory ``<name>/`` (the ``ros2 bag record -o`` URI) holding
    a nested ``<name>_0.mcap`` and — only on a clean finalize — a ``metadata.yaml``. The sidecar is a
    *sibling* of the directory: ``<name>.meta.json``. ``with_metadata=False`` models a recorder
    killed before finalize (no ``metadata.yaml`` ⇒ not a complete bag).
    """
    bag = tmp_path / "patrol_x_20260629_120000"
    bag.mkdir()
    (bag / "patrol_x_20260629_120000_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    if with_metadata:
        (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    if with_sidecar:
        (tmp_path / "patrol_x_20260629_120000.meta.json").write_text("{}")
    return bag


def _daemon(transport: _FakeTransport, *, target: str = "dgx:/data/bags/", **kwargs):
    return UploadDaemon(transport=transport, target=target, **kwargs)


# iter_bag_dirs returns [] when the watch dir doesn't exist yet — the daemon may start before the
# first recording run creates it (greptile P1; mirrors the ingest sibling's guard). Must not raise.
def test_iter_bag_dirs_missing_watch_dir_returns_empty(tmp_path: Path) -> None:
    assert iter_bag_dirs(tmp_path / "not_created_yet") == []


# iter_bag_dirs enumerates only the bag *directories* under the watch dir, sorted, ignoring files.
def test_iter_bag_dirs_lists_bag_directories(tmp_path: Path) -> None:
    bag = _make_bag(tmp_path, with_sidecar=True)  # creates a bag dir + a sibling .meta.json file

    assert iter_bag_dirs(tmp_path) == [bag]


# TS-3: the "complete" guard REJECTS a bag without its sidecar — nothing is sent.
def test_bag_without_sidecar_is_not_uploaded(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=False)

    uploaded = _daemon(transport).on_bag_complete(bag)

    assert uploaded is False
    assert transport.sent == []


# TS-3: a bag dir not yet finalized (no metadata.yaml — recorder killed pre-finalize) is NOT complete.
def test_unfinalized_bag_dir_is_not_uploaded(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True, with_metadata=False)

    uploaded = _daemon(transport).on_bag_complete(bag)

    assert uploaded is False
    assert transport.sent == []


# TS-4: a complete bag (bag + sidecar) is transferred — BOTH files, to the configured target.
def test_complete_bag_transfers_bag_and_sidecar(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    uploaded = _daemon(transport, target="dgx:/data/bags/").on_bag_complete(bag)

    assert uploaded is True
    sent_locals = [local for local, _remote in transport.sent]
    assert str(bag) in sent_locals
    assert str(bag) + ".meta.json" in sent_locals
    assert all(remote == "dgx:/data/bags/" for _local, remote in transport.sent)


# TS-4: the daemon transfers ONLY the bag + sidecar — no indexing/extra side effects (dumb producer).
def test_complete_bag_transfers_exactly_two_files(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    _daemon(transport).on_bag_complete(bag)

    assert len(transport.sent) == 2


# TS-5: a failed transfer is retried; on a later success the bag is confirmed uploaded.
def test_failed_transfer_is_retried_then_succeeds(tmp_path: Path) -> None:
    # First bag-send fails, retry of the bag succeeds, then the sidecar succeeds.
    transport = _FakeTransport(results=[False, True, True])
    bag = _make_bag(tmp_path, with_sidecar=True)

    uploaded = _daemon(transport, max_retries=3, backoff_s=0).on_bag_complete(bag)

    assert uploaded is True
    # The bag was attempted at least twice (initial failure + retry).
    bag_attempts = [local for local, _ in transport.sent if local == str(bag)]
    assert len(bag_attempts) >= 2


# TS-5: when every retry fails, the daemon reports failure and leaves the bag on disk.
def test_persistent_failure_reports_false_and_keeps_bag(tmp_path: Path) -> None:
    transport = _FakeTransport(results=[False, False, False, False])
    bag = _make_bag(tmp_path, with_sidecar=True)

    uploaded = _daemon(transport, max_retries=3, backoff_s=0).on_bag_complete(bag)

    assert uploaded is False
    assert bag.exists()  # never deleted before a confirmed transfer (§4.4.5)


# F-05: a CONFIRMED transfer drops a local <bag>.uploaded marker — the durable 'already handled'
# signal that survives the in-memory LRU's eviction (so old bags aren't re-rsynced at retention scale).
def test_confirmed_transfer_writes_upload_marker(tmp_path: Path) -> None:
    transport = _FakeTransport()  # every send succeeds
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport).on_bag_complete(bag) is True
    assert upload_marker_for(bag).is_file()


# F-05: a FAILED transfer leaves NO marker, so the bag correctly retries on a later poll.
def test_failed_transfer_writes_no_upload_marker(tmp_path: Path) -> None:
    transport = _FakeTransport(results=[False, False, False, False])
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, max_retries=3, backoff_s=0).on_bag_complete(bag) is False
    assert not upload_marker_for(bag).exists()


# Mira Medium (review 4728294643): a SYMLINKED bag dir — even one pointing at a genuinely complete
# bag — is refused at discovery AND by the complete-guard when called directly: a symlink planted
# in the watch dir would redirect the rsync source outside the watched tree.
def test_symlinked_bag_dir_is_rejected(tmp_path: Path) -> None:
    transport = _FakeTransport()
    real_root = tmp_path / "elsewhere"
    real_root.mkdir()
    real_bag = _make_bag(real_root, with_sidecar=True)  # a genuinely complete bag, outside watch
    watch_dir = tmp_path / "watch"
    watch_dir.mkdir()
    link = watch_dir / real_bag.name
    link.symlink_to(real_bag)
    (watch_dir / (link.name + ".meta.json")).write_text("{}")  # sidecar beside the link, real file

    assert iter_bag_dirs(watch_dir) == []  # excluded from discovery
    assert _daemon(transport).on_bag_complete(link) is False  # refused even when called directly
    assert transport.sent == []


# Mira Medium (review 4728294643): a bag whose SIDECAR is a symlink is not complete — the sidecar
# read/transfer would follow the link outside the watched tree, so the bag is never shipped.
def test_symlinked_sidecar_is_not_complete(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=False)  # finalized bag dir, no real sidecar
    real_sidecar = tmp_path / "redirect-target.meta.json"
    real_sidecar.write_text("{}")
    (tmp_path / (bag.name + ".meta.json")).symlink_to(real_sidecar)

    assert _daemon(transport).on_bag_complete(bag) is False
    assert transport.sent == []
