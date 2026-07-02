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
from pathlib import Path

import pytest
from ingest.__main__ import _INGEST_FAULTS, _try_index
from ingest.ingest_service import BagFacts, BagFactsReader, IngestService
from ingest.manifest_store import ManifestStore


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
    ],
)
def test_try_index_skips_reader_fault_without_propagating(tmp_path: Path, exc: Exception) -> None:
    service, bag, sidecar = _service_with_failing_reader(tmp_path, exc)

    assert _try_index(service, bag, sidecar) is False  # skipped, did NOT propagate / crash
    assert service._store.query_recent(10) == []  # nothing half-indexed


# F-01 regression contract: ValueError is explicitly a member of the documented fault set, so the
# parse-failure path is caught structurally (not only via the behavioural test above).
def test_value_error_is_in_documented_ingest_fault_set() -> None:
    assert ValueError in _INGEST_FAULTS


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


# F-03: TypeError (non-object sidecar) and UnicodeDecodeError (non-UTF-8 sidecar) are members of the
# documented fault set — so the watch loop skips-and-retries them instead of crash-looping.
@pytest.mark.parametrize("fault", [TypeError, UnicodeDecodeError])
def test_new_sidecar_faults_are_in_documented_ingest_fault_set(fault: type[BaseException]) -> None:
    assert fault in _INGEST_FAULTS


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


# greptile P1: sqlite3.IntegrityError is a member of the documented fault set, so a null required
# field (which passes every pre-DB guard) is skipped-and-retried, not crash-looping the service.
def test_null_field_sidecar_fault_is_in_documented_ingest_fault_set() -> None:
    assert sqlite3.IntegrityError in _INGEST_FAULTS


# greptile P1: a sidecar with a required field explicitly JSON null passes json.loads, the key check,
# and ManifestRow construction, then trips the DB NOT NULL column → IntegrityError. It must be skipped
# (returns False), not raised, else the watch loop crash-loops on the same bag after every restart.
def test_try_index_skips_null_required_field_sidecar(tmp_path: Path) -> None:
    sidecar_bytes = b'{"mission_id": null, "started_utc": "2026-06-29T12:00:00+00:00"}'
    service, bag, sidecar = _service_with_bad_sidecar(tmp_path, sidecar_bytes)
    assert _try_index(service, bag, sidecar) is False
    assert service._store.query_recent(10) == []
