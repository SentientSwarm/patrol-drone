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

from pathlib import Path

import pytest
from ingest.__main__ import _INGEST_FAULTS, _try_index
from ingest.ingest_service import BagFacts, IngestService
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
