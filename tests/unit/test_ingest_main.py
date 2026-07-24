"""Layer-A unit tests for the ingest watch-loop fault tolerance (M8 / F-01, design §4.4.5).

`ingest.__main__._try_index` wraps `IngestService.index` in `except _INGEST_FAULTS` so one bad bag
is skipped + retried (returns False, stays out of `indexed`) instead of crashing the long-running
service — "one bad bag can't starve the rest". `docker/ingest/__main__.py` is an I/O shell
(coverage-omitted; its happy path is the stand-in integration test), but the *membership* of
`_INGEST_FAULTS` is load-bearing first-party logic: a fault the set omits crash-loops the service.
This pins the contract for the bag-fact reader faults that travel the injected `bag_facts` seam —
notably the `ValueError` `parse_bag_info` raises when `ros2 bag info` runs (exit 0) but prints no
`Duration:` line (bag_reader.py), the F-01 gap.
"""

from __future__ import annotations

import sqlite3
import subprocess
from pathlib import Path

import pytest
import yaml
from ingest.__main__ import _INGEST_FAULTS, _drain_once, _try_index
from ingest.bag_reader import read_bag_facts
from ingest.bounded_seen import _BoundedSeen
from ingest.ingest_service import BagFacts, BagFactsReader, IngestService
from ingest.manifest_store import ManifestRow, ManifestStore


def _make_finalized_bag(parent: Path, name: str) -> Path:
    """A finalized rosbag2 bag dir: the nested ``<name>_0.mcap`` payload + the metadata.yaml marker.

    The payload is part of the shape now, not decoration: the shared bag-layout validator
    (``_shared.bag_layout``) requires a real ``.mcap``, so a metadata-only dir would be refused at
    the guard and these tests would pass for the wrong reason — never reaching the fault they exist
    to pin.
    """
    bag = parent / name
    bag.mkdir()
    (bag / f"{name}_0.mcap").write_bytes(b"\x89MCAP0\r\n")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    return bag


def _service_with_failing_reader(
    tmp_path: Path, exc: Exception
) -> tuple[IngestService, Path, Path]:
    """An IngestService whose injected bag-fact reader raises ``exc`` from inside ``index``.

    Builds a finalized bag dir (payload + metadata.yaml) + a valid sidecar so ``index`` gets past its
    own guards and reaches ``self._bag_facts(bag_path)`` — the seam where reader faults originate.
    """
    bag = _make_finalized_bag(tmp_path, "patrol_unparseable_20260629_120000")
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text(
        '{"mission_id": "standin", "bag_uri": "x", "started_utc": "2026-06-29T12:00:00+00:00", '
        '"ended_utc": "2026-06-29T12:02:22+00:00", "recorded_topics": [], "mission_config_ref": "x"}'
    )

    def reader(_bag_path: Path) -> BagFacts:
        raise exc

    return IngestService(ManifestStore(tmp_path / "m.db"), bag_facts=reader), bag, sidecar


# F-01: a bag-fact reader fault in _INGEST_FAULTS is skipped (False, no crash, nothing indexed),
# so a corrected bag retries on a later poll. ValueError is the reopened-by-F-04 gap; the other
# rows are the regression guard for the documented members on the same seam.
@pytest.mark.parametrize(
    "exc",
    [
        ValueError("could not parse Duration from ros2 bag info output"),  # the F-01 gap
        FileNotFoundError("bag vanished mid-index"),
        # F-03: a hung `ros2 bag info` (TimeoutExpired from the reader) is skipped, not propagated —
        # so one stuck CLI can't wedge the serial ingest loop.
        subprocess.TimeoutExpired(cmd=["ros2", "bag", "info"], timeout=120.0),
        # F-04: a present-but-unreadable metadata file (chmod 000 / owned by another user mid-sync)
        # makes the reader raise PermissionError; a transient/stale-mount read makes it raise a
        # generic OSError. Both are recoverable — skip + retry, never crash the daemon on one bag.
        PermissionError("metadata.yaml: permission denied"),
        OSError("stale NFS file handle reading metadata.yaml"),
    ],
)
def test_try_index_skips_reader_fault_without_propagating(tmp_path: Path, exc: Exception) -> None:
    service, bag, sidecar = _service_with_failing_reader(tmp_path, exc)

    assert _try_index(service, bag, sidecar) is False  # skipped, did NOT propagate / crash
    assert service._store.query_recent(10) == []  # nothing half-indexed


def _fixed_facts_reader() -> BagFactsReader:
    """A reader returning fixed facts — the F-03 faults under test happen before it's ever reached."""

    def reader(_bag_path: Path) -> BagFacts:
        return BagFacts(duration_s=1.0, topic_counts={})

    return reader


# F-03: the two sidecar-shape faults that escape the loop if the set omits them. These arise in
# IngestService.index BEFORE the injected reader (at read_text / sidecar["mission_id"]), so they
# need a real bad sidecar rather than a failing reader.
def _service_with_bad_sidecar(
    tmp_path: Path, sidecar_bytes: bytes
) -> tuple[IngestService, Path, Path]:
    """A finalized bag dir + a sidecar whose *bytes* are written verbatim (may be non-UTF-8/non-dict)."""
    bag = _make_finalized_bag(tmp_path, "patrol_badsidecar_20260629_120000")
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_bytes(sidecar_bytes)
    service = IngestService(ManifestStore(tmp_path / "m.db"), bag_facts=_fixed_facts_reader())
    return service, bag, sidecar


# The documented fault set's membership is load-bearing: a fault it omits crash-loops the service.
# One parametrized guard pins every member the watch loop relies on (folded from what were separate
# per-fault tests, so adding a member is one row, not a new copied block — CodeScene duplication):
#   * ValueError — parse failure: no Duration line, or F-02's zero-topics guard (bag_reader.py).
#   * subprocess.CalledProcessError / TimeoutExpired — `ros2 bag info` exited non-zero / hung (F-03).
#   * TypeError / UnicodeDecodeError — non-object / non-UTF-8 sidecar (arise before the reader).
#   * sqlite3.IntegrityError — a required sidecar field is JSON null → DB NOT NULL reject (greptile P1).
#   * sqlite3.OperationalError — a transient locked/busy manifest (concurrent reader/writer) (F-02).
#   * PermissionError / OSError — a present-but-unreadable metadata/sidecar file (F-04).
@pytest.mark.parametrize(
    "fault",
    [
        ValueError,
        subprocess.CalledProcessError,
        subprocess.TimeoutExpired,
        TypeError,
        UnicodeDecodeError,
        sqlite3.IntegrityError,
        yaml.YAMLError,  # F-03/greptile P1: a malformed metadata.yaml (yaml.safe_load raises)
        sqlite3.OperationalError,  # F-02: a locked/busy manifest → skip + retry, don't crash the loop
        PermissionError,  # F-04: metadata/sidecar present but unreadable (chmod 000 / cross-owner)
        OSError,  # F-04: other intended file-read failure (transient I/O, stale NFS handle)
    ],
)
def test_documented_ingest_fault_set_members(fault: type[BaseException]) -> None:
    assert fault in _INGEST_FAULTS


# F-02 crash-normalization contract: AttributeError must NOT be caught here. Widening the set to catch
# it would silence the very exception that signals a genuine programming bug; instead bag_reader
# validates metadata shape and raises the caught ValueError, so a malformed bag is skipped + retried
# while a real defect still surfaces loudly. Kept separate from the membership param list above so the
# assertion body stays a single `not in` (no inverted-branch CodeScene shape).
def test_attribute_error_is_not_swallowed_by_the_ingest_fault_set() -> None:
    assert AttributeError not in _INGEST_FAULTS


# F-03: a valid-JSON-but-non-object sidecar (TypeError on sidecar["mission_id"]) is skipped, not raised.
def test_try_index_skips_non_object_sidecar(tmp_path: Path) -> None:
    service, bag, sidecar = _service_with_bad_sidecar(tmp_path, b"[1, 2, 3]")
    assert _try_index(service, bag, sidecar) is False
    assert service._store.query_recent(10) == []


# F-03: a non-UTF-8 sidecar (UnicodeDecodeError on read_text) is skipped, not raised.
def test_try_index_skips_non_utf8_sidecar(tmp_path: Path) -> None:
    service, bag, sidecar = _service_with_bad_sidecar(tmp_path, b"\xff\xfe not utf-8")
    assert _try_index(service, bag, sidecar) is False
    assert service._store.query_recent(10) == []


# greptile P1 (now caught earlier by F-02's schema validation): a required field explicitly JSON
# null used to travel all the way to the DB NOT NULL column (IntegrityError); the boundary validator
# now rejects it as a TypeError before any store call. Either way it must be skipped (returns
# False), not raised — IntegrityError stays in _INGEST_FAULTS as belt-and-suspenders.
def test_try_index_skips_null_required_field_sidecar(tmp_path: Path) -> None:
    sidecar_bytes = b'{"mission_id": null, "started_utc": "2026-06-29T12:00:00+00:00"}'
    service, bag, sidecar = _service_with_bad_sidecar(tmp_path, sidecar_bytes)
    assert _try_index(service, bag, sidecar) is False
    assert service._store.query_recent(10) == []


# F-03 (the live greptile P1): a bag whose metadata.yaml exists but is MALFORMED makes the REAL
# read_bag_facts reader's yaml.safe_load raise yaml.YAMLError. Unlike the sidecar-shape faults above,
# this arises inside the injected reader — so the service is built with bag_facts=read_bag_facts (not a
# stub). It must be skipped (False, nothing indexed), else the loop crash-loops on the corrupt bag.
def test_try_index_skips_malformed_metadata_yaml(tmp_path: Path) -> None:
    bag = _make_finalized_bag(tmp_path, "patrol_corruptmeta_20260629_120000")
    # Overwrite the marker AFTER the payload exists: the bag must pass the shared layout guard so the
    # YAML fault is what fails, not a missing payload.
    (bag / "metadata.yaml").write_text(":\n  - [unterminated")  # invalid YAML → yaml.YAMLError
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text(
        f'{{"mission_id": "standin", "bag_uri": "{bag.name}", '
        '"started_utc": "2026-06-29T12:00:00+00:00"}'
    )
    service = IngestService(ManifestStore(tmp_path / "m.db"), bag_facts=read_bag_facts)

    assert _try_index(service, bag, sidecar) is False  # skipped, did NOT propagate / crash
    assert service._store.query_recent(10) == []  # nothing half-indexed


def _raise_if_read(_bag_path: Path) -> BagFacts:
    """A bag-fact reader that fails if ever called — proves _drain_once skipped the read entirely."""
    raise AssertionError("read_bag_facts must NOT be called for a bag already in the manifest")


def _put_watched_bag(watch_dir: Path, name: str, sidecar_json: str) -> Path:
    """A finalized bag dir + sibling sidecar under ``watch_dir`` (the drain-loop input shape)."""
    bag = _make_finalized_bag(watch_dir, name)
    (watch_dir / (name + ".meta.json")).write_text(sidecar_json)
    return bag


# F-02 isolation proof (Mira High, review 4728294643): ONE malformed bag (list-valued mission_id —
# the exact daemon-killer shape) must not stop a valid sibling from indexing in the same drain
# pass. The malformed bag is skipped (schema validation raises a caught TypeError) and stays out of
# `indexed` so it retries once corrected; the valid bag lands in the manifest. Sorted discovery
# processes the malformed bag FIRST, so this proves the loop continues past the fault.
def test_drain_once_indexes_valid_bag_despite_malformed_sibling(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    watch_dir.mkdir()
    bad = _put_watched_bag(
        watch_dir,
        "patrol_badfield_20260629_120000",
        '{"mission_id": ["a"], "bag_uri": "patrol_badfield_20260629_120000", '
        '"started_utc": "2026-06-29T12:00:00+00:00"}',
    )
    good_name = "patrol_good_20260629_130000"
    _put_watched_bag(
        watch_dir,
        good_name,
        f'{{"mission_id": "standin", "bag_uri": "{good_name}", '
        '"started_utc": "2026-06-29T13:00:00+00:00"}',
    )
    store = ManifestStore(tmp_path / "m.db")
    service = IngestService(store, bag_facts=_fixed_facts_reader())
    indexed = _BoundedSeen()

    _drain_once(service, watch_dir, indexed)  # must NOT raise — the malformed bag is skipped

    assert [r.bag_id for r in store.query_recent(10)] == [good_name]  # exactly the valid bag
    assert bad not in indexed  # malformed bag retries on a later poll once corrected


# Mira Medium (review 4728294643), mirror of the upload side: the ingest loop refuses symlinked
# entries in its landing dir — a symlinked bag dir and a bag whose sidecar is a symlink are each
# skipped (nothing indexed, no raise), since a planted symlink would redirect the metadata/sidecar
# reads outside the DGX landing tree, whose writers are remote by design.
def test_drain_once_skips_symlinked_bag_dir_and_sidecar(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    watch_dir.mkdir()
    real_root = tmp_path / "elsewhere"
    real_root.mkdir()
    linked_name = "patrol_dirlink_20260629_120000"
    real_bag = _put_watched_bag(
        real_root,
        linked_name,
        f'{{"mission_id": "standin", "bag_uri": "{linked_name}", '
        '"started_utc": "2026-06-29T12:00:00+00:00"}',
    )
    (watch_dir / linked_name).symlink_to(real_bag)  # symlinked bag dir in the landing dir
    (watch_dir / (linked_name + ".meta.json")).write_text("{}")  # real sidecar beside the link
    sidelink_name = "patrol_sidecarlink_20260629_130000"
    # A real, fully-valid bag dir whose SIDECAR is a symlink — the symlink must be the only defect.
    _make_finalized_bag(watch_dir, sidelink_name)
    real_sidecar = tmp_path / "redirect-target.meta.json"
    real_sidecar.write_text(
        f'{{"mission_id": "standin", "bag_uri": "{sidelink_name}", '
        '"started_utc": "2026-06-29T13:00:00+00:00"}'
    )
    (watch_dir / (sidelink_name + ".meta.json")).symlink_to(real_sidecar)
    service = IngestService(ManifestStore(tmp_path / "m.db"), bag_facts=_fixed_facts_reader())

    _drain_once(service, watch_dir, _BoundedSeen())  # must NOT raise — both entries are refused

    assert service._store.query_recent(10) == []  # nothing indexed through a symlink


# F-05: a bag whose bag_id is ALREADY in the manifest is skipped by _drain_once — the reader is never
# re-invoked and no duplicate row is written — even with a fresh in-memory seen-set (the durable skip
# survives eviction). This is the retention-scale reprocessing regression the fix closes.
def test_drain_once_skips_bag_already_in_manifest(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    watch_dir.mkdir()
    name = "patrol_already_20260629_120000"
    _make_finalized_bag(watch_dir, name)
    (watch_dir / (name + ".meta.json")).write_text(
        f'{{"mission_id": "standin", "bag_uri": "{name}", '
        '"started_utc": "2026-06-29T12:00:00+00:00"}'
    )
    store = ManifestStore(tmp_path / "m.db")
    store.upsert(
        ManifestRow(
            bag_id=name,
            mission_id="standin",
            recorded_utc="2026-06-29T12:00:00+00:00",
            duration_s=1.0,
            topics_json="{}",
            metadata_json="{}",
            ingested_utc="2026-06-29T12:05:00+00:00",
        )
    )
    service = IngestService(store, bag_facts=_raise_if_read)

    _drain_once(service, watch_dir, _BoundedSeen())  # must not raise (reader never called)

    assert len(store.query_recent(10)) == 1  # still exactly one row, not re-upserted


# F-04: a sidecar file that is PRESENT but unreadable (its read_text raises PermissionError) is
# skipped, not raised — one chmod-000 bag can't kill the daemon. Exercised through the real index
# path so the PermissionError originates at the sidecar read (ingest_service.index), inside _try_index.
def test_try_index_skips_unreadable_sidecar(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    bag = _make_finalized_bag(tmp_path, "patrol_unreadable_20260629_120000")
    sidecar = tmp_path / (bag.name + ".meta.json")
    sidecar.write_text('{"mission_id": "standin", "bag_uri": "x", "started_utc": "x"}')
    service = IngestService(ManifestStore(tmp_path / "m.db"), bag_facts=_fixed_facts_reader())

    real_read_text = Path.read_text

    def _refuse_sidecar(self: Path, *args: object, **kwargs: object) -> str:
        if self == sidecar:
            raise PermissionError(f"permission denied reading {self}")
        return real_read_text(self, *args, **kwargs)  # type: ignore[arg-type]

    monkeypatch.setattr(Path, "read_text", _refuse_sidecar)

    assert _try_index(service, bag, sidecar) is False  # skipped, did NOT propagate / crash
    assert service._store.query_recent(10) == []  # nothing half-indexed


# F-02: a transient locked manifest during the PRE-index dedup check (already_indexed → SQLite) must
# be caught too — not just around index — or it escapes _drain_once and terminates the watch loop. A
# skipped bag stays out of the manifest AND out of `indexed`, so it is retried on a later poll.
def test_drain_once_skips_bag_when_dedup_check_locks(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    watch_dir.mkdir()
    name = "patrol_lockeddb_20260629_120000"
    bag = _make_finalized_bag(watch_dir, name)
    (watch_dir / (name + ".meta.json")).write_text(
        f'{{"mission_id": "standin", "bag_uri": "{name}", '
        '"started_utc": "2026-06-29T12:00:00+00:00"}'
    )

    class _LockingService(IngestService):
        def already_indexed(self, _bag_path: Path) -> bool:
            raise sqlite3.OperationalError("database is locked")

    service = _LockingService(ManifestStore(tmp_path / "m.db"), bag_facts=_raise_if_read)
    indexed = _BoundedSeen()

    _drain_once(service, watch_dir, indexed)  # must NOT raise — the locked dedup check is caught

    assert service._store.query_recent(10) == []  # nothing indexed
    assert bag not in indexed  # left out of `indexed` so it retries on a later poll
