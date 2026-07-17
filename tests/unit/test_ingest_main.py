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


def _service_with_failing_reader(
    tmp_path: Path, exc: Exception
) -> tuple[IngestService, Path, Path]:
    """An IngestService whose injected bag-fact reader raises ``exc`` from inside ``index``.

    Builds a finalized bag dir (metadata.yaml present) + a valid sidecar so ``index`` gets past its
    own guards and reaches ``self._bag_facts(bag_path)`` — the seam where reader faults originate.
    """
    bag = tmp_path / "patrol_unparseable_20260629_120000"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
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
    bag = tmp_path / "patrol_badsidecar_20260629_120000"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
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


# greptile P1: a sidecar with a required field explicitly JSON null passes json.loads, the key check,
# and ManifestRow construction, then trips the DB NOT NULL column → IntegrityError. It must be skipped
# (returns False), not raised, else the watch loop crash-loops on the same bag after every restart.
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
    bag = tmp_path / "patrol_corruptmeta_20260629_120000"
    bag.mkdir()
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


# F-05: a bag whose bag_id is ALREADY in the manifest is skipped by _drain_once — the reader is never
# re-invoked and no duplicate row is written — even with a fresh in-memory seen-set (the durable skip
# survives eviction). This is the retention-scale reprocessing regression the fix closes.
def test_drain_once_skips_bag_already_in_manifest(tmp_path: Path) -> None:
    watch_dir = tmp_path / "bags"
    watch_dir.mkdir()
    name = "patrol_already_20260629_120000"
    bag = watch_dir / name
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
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
