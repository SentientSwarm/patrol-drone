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

import json
import os
from pathlib import Path

import pytest

from upload_daemon.upload_daemon import (
    UploadDaemon,
    is_already_uploaded,
    is_complete,
    iter_bag_dirs,
    sidecar_path_for,
    upload_marker_for,
)

_TARGET = "dgx:/data/bags/"


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


def _make_bag(
    tmp_path: Path,
    *,
    with_sidecar: bool,
    with_metadata: bool = True,
    mcap: str = "real",
) -> Path:
    """Create a rosbag2 bag directory under tmp_path; return the bag-directory path.

    The recorder writes each run as a directory ``<name>/`` (the ``ros2 bag record -o`` URI) holding
    a nested ``<name>_0.mcap`` and — only on a clean finalize — a ``metadata.yaml``. The sidecar is a
    *sibling* of the directory: ``<name>.meta.json``. ``with_metadata=False`` models a recorder
    killed before finalize (no ``metadata.yaml`` ⇒ not a complete bag). ``mcap`` selects the payload
    shape for the F-01 cases: ``"real"`` (the normal bag), ``"none"`` (a metadata-only directory —
    recorder killed before the MCAP flushed, or a half-landed rsync) or ``"symlink"`` (a payload that
    redirects outside the watched tree).
    """
    bag = tmp_path / "patrol_x_20260629_120000"
    bag.mkdir()
    payload = bag / "patrol_x_20260629_120000_0.mcap"
    if mcap == "real":
        payload.write_bytes(b"\x89MCAP0\r\n")
    elif mcap == "symlink":
        redirect = tmp_path / "payload-elsewhere.mcap"
        redirect.write_bytes(b"\x89MCAP0\r\n")
        payload.symlink_to(redirect)
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


# F-04 (guide §1.B) read side: a planted SYMLINK named <bag>.uploaded pointing at any regular file
# must NOT count as "already uploaded" — a symlink-following is_file() would suppress the transfer
# of a never-shipped bag forever (a false-success). The daemon trusts only real, non-symlink markers.
def test_symlinked_upload_marker_is_not_already_uploaded(tmp_path: Path) -> None:
    bag = _make_bag(tmp_path, with_sidecar=True)
    redirect_target = tmp_path / "some-real-file"
    redirect_target.write_text("x")
    upload_marker_for(bag).symlink_to(redirect_target)

    assert is_already_uploaded(bag, _TARGET) is False


# F-04 write side: the daemon only trusts markers it EXCLUSIVELY creates. A pre-planted file or
# symlink at the marker path makes the post-transfer O_CREAT|O_EXCL|O_NOFOLLOW create fail LOUDLY
# (FileExistsError) instead of silently adopting a foreign marker; the driver loop's _UPLOAD_FAULTS
# boundary (⊇ OSError) logs it and the bag stays un-marked by us, so it is retried, not lost.
@pytest.mark.parametrize(
    "plant", ["regular_file", "symlink"], ids=["planted-file", "planted-symlink"]
)
def test_preplanted_marker_fails_the_confirm_loudly(tmp_path: Path, plant: str) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)
    marker = upload_marker_for(bag)
    redirect_target = tmp_path / "attacker-target"
    redirect_target.write_text("untouched")
    if plant == "regular_file":
        marker.write_text("squatter")
    else:
        marker.symlink_to(redirect_target)

    with pytest.raises(FileExistsError):
        _daemon(transport).on_bag_complete(bag)

    assert redirect_target.read_text() == "untouched"  # never written through a planted symlink


# F-04 happy path: the durable-skip loop stays intact — a confirmed transfer writes a real,
# non-symlink marker and the NEXT poll's is_already_uploaded sees it.
def test_confirmed_marker_is_seen_by_the_next_poll(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport).on_bag_complete(bag) is True
    assert not upload_marker_for(bag).is_symlink()
    assert is_already_uploaded(bag, _TARGET) is True


# F-02 (review 4748505221) core: a stale/pre-planted EMPTY plain marker — a bare `touch` or a
# backup-restored sentinel — must NOT read as "already uploaded". The old code trusted any regular
# non-symlink file; now the marker must be a valid receipt, so an empty one correctly re-uploads.
def test_empty_stale_marker_is_not_already_uploaded(tmp_path: Path) -> None:
    bag = _make_bag(tmp_path, with_sidecar=True)
    upload_marker_for(bag).write_text("")  # a bare `touch` — real file, but no receipt

    assert is_already_uploaded(bag, _TARGET) is False


# F-02: any marker that is not a well-formed receipt for THIS bag (empty, non-JSON, missing the
# files list, or naming other files) reads as "not uploaded" so the bag retries — a stale/pre-planted
# marker can no longer silently suppress the transfer, regardless of its exact bad shape.
# NB: the two JSON bodies carry the CANONICAL target ("dgx:/data/bags", no trailing slash) on purpose.
# The target gate now runs BEFORE the files gate, so a raw "dgx:/data/bags/" body would be rejected on
# the target and these two rows would stop testing the `files` defect their ids name.
@pytest.mark.parametrize(
    "body",
    [
        "",
        "not json",
        '{"target": "dgx:/data/bags"}',
        '{"target": "dgx:/data/bags", "files": ["other"]}',
    ],
    ids=["empty", "non-json", "missing-files", "wrong-files"],
)
def test_malformed_marker_is_not_already_uploaded(tmp_path: Path, body: str) -> None:
    bag = _make_bag(tmp_path, with_sidecar=True)
    upload_marker_for(bag).write_text(body)

    assert is_already_uploaded(bag, _TARGET) is False


# F-02: a CONFIRMED transfer writes a valid, self-describing receipt — the CANONICAL transfer target
# plus the sorted covered filenames (bag dir + sidecar) — so is_already_uploaded is bound to a real
# transfer, not a bare sentinel. This is the write-side pair to the malformed-marker rejection above.
def test_confirmed_marker_is_a_valid_receipt(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, target="dgx:/data/bags/").on_bag_complete(bag) is True

    receipt = json.loads(upload_marker_for(bag).read_text())
    assert (
        receipt["target"] == "dgx:/data/bags"
    )  # canonicalized on the way in (trailing / stripped)
    assert receipt["files"] == sorted([bag.name, sidecar_path_for(bag).name])


# F-01 (Mira High, review 4752923085): a finalized-LOOKING bag dir with no real MCAP payload must not
# ship. Both shapes the review names are covered: a metadata+sidecar-only directory (a recorder killed
# before the MCAP flushed, or a half-landed rsync) and a directory whose only .mcap is a symlink
# redirecting outside the watched tree. Either would otherwise upload and then index as a
# fully-populated manifest row for an unreplayable bag.
@pytest.mark.parametrize("mcap", ["none", "symlink"], ids=["no-mcap", "symlinked-mcap"])
def test_bag_without_a_real_mcap_payload_is_not_uploaded(tmp_path: Path, mcap: str) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True, mcap=mcap)

    assert is_complete(bag) is False
    assert _daemon(transport).on_bag_complete(bag) is False
    assert transport.sent == []


# F-02 (review 4752923085), the destination-reconfiguration case the review names: a receipt written
# for target A must NOT read as "already uploaded" once the operator repoints the daemon at target B,
# or every bag already on disk at cutover is silently skipped forever.
def test_receipt_for_a_different_target_is_not_already_uploaded(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, target="dgx1:/data/bags/").on_bag_complete(bag) is True

    assert is_already_uploaded(bag, "dgx2:/data/bags/") is False


# F-02 inverse: the SAME target still reads as uploaded, so the H-09/F-05 durable-skip guarantee does
# not silently regress into re-rsyncing every bag on every poll.
def test_receipt_for_the_same_target_is_still_already_uploaded(tmp_path: Path) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, target="dgx1:/data/bags/").on_bag_complete(bag) is True

    assert is_already_uploaded(bag, "dgx1:/data/bags/") is True


# F-02: the comparison is CANONICAL, not literal — the same destination written with/without a
# trailing slash or with stray whitespace must still match, or a cosmetic --target edit would
# re-upload the entire retention window. The write and read sides share one normalizer, so they
# cannot drift.
@pytest.mark.parametrize(
    "configured",
    ["dgx:/data/bags", "dgx:/data/bags/", "dgx:/data/bags//", "  dgx:/data/bags/  "],
    ids=["bare", "trailing-slash", "double-slash", "whitespace"],
)
def test_trailing_slash_variants_match_the_same_target(tmp_path: Path, configured: str) -> None:
    transport = _FakeTransport()
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, target="dgx:/data/bags/").on_bag_complete(bag) is True

    assert is_already_uploaded(bag, configured) is True


# F-04 (Mira Medium, review 4754192970): an INTERRUPTED marker publication must leave no marker at
# all, not a zero-length one. The old O_EXCL-on-the-marker write published the NAME before the body
# and never fsynced, so a crash mid-write left an empty marker that: fails receipt validation → reads
# as "not uploaded" → the bag is re-transferred → the exclusive create hits the leftover name →
# FileExistsError → caught by _UPLOAD_FAULTS → the bag is never marked → re-transfers EVERY poll,
# forever. Publishing via a temp + fsync + os.link makes the name and its durable body appear
# together, so the observable outcome of a crash is "no marker", and the next poll recovers cleanly.
def test_interrupted_marker_publication_leaves_no_marker_and_recovers(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    bag = _make_bag(tmp_path, with_sidecar=True)

    def _crash_mid_publish(_fd: int) -> None:
        raise OSError("simulated crash while the receipt was being made durable")

    monkeypatch.setattr(os, "fsync", _crash_mid_publish)

    with pytest.raises(OSError, match="simulated crash"):
        _daemon(_FakeTransport()).on_bag_complete(bag)

    assert not upload_marker_for(bag).exists()  # no zero-length marker to livelock on
    assert not list(tmp_path.glob("*.tmp"))  # and no temp crumb masquerading as an artifact

    monkeypatch.undo()  # the crash is over; the next poll must just work
    assert _daemon(_FakeTransport()).on_bag_complete(bag) is True
    # A real, valid receipt — recovery, not the FileExistsError livelock.
    assert is_already_uploaded(bag, _TARGET) is True


# F-03 (Mira Medium, review 4754192970), the paired constructor half of the CLI rejection in
# test_upload_main.py: the receipt is written against the CANONICAL target while the transport is
# handed the RAW one, so a target differing only by surrounding whitespace would write one receipt but
# rsync to a different destination. Validating at the CLI alone would leave a programmatic
# UploadDaemon(...) able to re-introduce exactly that divergence — hardening one half of a boundary is
# what authored this finding in the first place — so construction refuses it too.
@pytest.mark.parametrize(
    "target",
    ["dgx:/data/bags ", " dgx:/data/bags", ""],
    ids=["trailing-space", "leading-space", "empty"],
)
def test_constructing_with_a_whitespace_target_is_rejected(target: str) -> None:
    with pytest.raises(ValueError, match="surrounding whitespace"):
        _daemon(_FakeTransport(), target=target)


# F-03 (Mira Medium, review 4752923085): a WEDGED transfer must be retried like any other recoverable
# failure — the retry loop actually runs (it sits OUTSIDE the send that used to never return) and the
# bag is left un-marked so a later poll tries again. The fake reports failure exactly the way
# RsyncSshTransport reports an expired timeout (a False return, not a raise).
def test_timed_out_transfer_is_retried_then_left_unmarked(tmp_path: Path) -> None:
    transport = _FakeTransport(results=[False, False, False, False])
    bag = _make_bag(tmp_path, with_sidecar=True)

    assert _daemon(transport, max_retries=3, backoff_s=0).on_bag_complete(bag) is False

    bag_attempts = [local for local, _ in transport.sent if local == str(bag)]
    assert len(bag_attempts) >= 2  # retry/backoff ran rather than being parked inside one send
    assert not upload_marker_for(bag).exists()  # no receipt → retried on a later poll
