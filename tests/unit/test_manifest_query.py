"""Layer-A unit tests for the manifest_query CLI (docset 05-logging-replay, M8 / T8.5, SWM-77).

Covers `ingest.manifest_query` — the operator "list recent runs + their topic sets" surface over
the ManifestStore (design §4.2.4, LR-4):

  * ``--recent N`` lists the N most-recent bags with their LR-4 fields (mission/time/duration/topics).
  * ``--mission <id>`` filters to one mission.
  * each rendered line carries the bag id, mission, duration, and a topic summary so the operator
    can identify a run without opening the bag.

The CLI is driven through its ``run`` function against a real (tmp) SQLite store so the rendering +
arg routing are tested without a process boundary; argv parsing is the thin shell.
"""

from __future__ import annotations

from pathlib import Path

from ingest.manifest_query import render_rows, run
from ingest.manifest_store import ManifestRow, ManifestStore


def _seed(store: ManifestStore) -> None:
    store.upsert(
        ManifestRow(
            bag_id="patrol_a_20260626_080740.mcap",
            mission_id="patrol",
            recorded_utc="2026-06-26T08:07:40+00:00",
            duration_s=142.0,
            topics_json='{"/patrol/mission_state": 1420, "/tf": 9000}',
            metadata_json="{}",
            ingested_utc="2026-06-26T09:00:00+00:00",
        )
    )
    store.upsert(
        ManifestRow(
            bag_id="survey_b_20260626_090000.mcap",
            mission_id="survey",
            recorded_utc="2026-06-26T09:00:00+00:00",
            duration_s=60.0,
            topics_json='{"/patrol/mission_state": 600}',
            metadata_json="{}",
            ingested_utc="2026-06-26T09:05:00+00:00",
        )
    )


# TS-12: --recent N returns the N most recent rows with mission/duration/topics visible.
def test_recent_lists_rows(tmp_path: Path, capsys) -> None:
    store = ManifestStore(tmp_path / "m.db")
    _seed(store)

    rc = run(["--recent", "5"], store=store)

    out = capsys.readouterr().out
    assert rc == 0
    assert "patrol_a_20260626_080740.mcap" in out
    assert "survey_b_20260626_090000.mcap" in out
    assert "patrol" in out
    assert "142" in out  # duration surfaced


# TS-13: --mission <id> filters to one mission's bags.
def test_mission_filter(tmp_path: Path, capsys) -> None:
    store = ManifestStore(tmp_path / "m.db")
    _seed(store)

    rc = run(["--mission", "survey"], store=store)

    out = capsys.readouterr().out
    assert rc == 0
    assert "survey_b_20260626_090000.mcap" in out
    assert "patrol_a_20260626_080740.mcap" not in out


# TS-12: each rendered line names the bag, mission, duration and a topic count summary.
def test_render_rows_includes_topic_summary(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "m.db")
    _seed(store)
    rows = store.query_by_mission("patrol")

    lines = render_rows(rows)

    assert len(lines) == 1
    line = lines[0]
    assert "patrol_a_20260626_080740.mcap" in line
    assert "patrol" in line
    assert "2 topics" in line  # /patrol/mission_state + /tf


# An empty manifest renders a clear "no bags" line rather than nothing.
def test_empty_manifest_renders_notice(tmp_path: Path, capsys) -> None:
    store = ManifestStore(tmp_path / "m.db")

    rc = run(["--recent", "5"], store=store)

    out = capsys.readouterr().out
    assert rc == 0
    assert "no bags" in out.lower()
