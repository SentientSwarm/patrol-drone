"""Layer-A unit tests for the manifest_query CLI (docset 05-logging-replay, M8 / T8.5, SWM-77).

Covers `ingest.manifest_query` — the operator "list recent runs + their topic sets" surface over
the ManifestStore (design §4.2.4, LR-4):

  * ``--recent N`` lists the N most-recent bags with their LR-4 fields (mission/time/duration/topics).
  * ``--mission <id>`` filters to one mission.
  * ``--all`` lists every indexed bag under a bounded cap (F-04's explicit all-results mode).
  * each rendered line carries the bag id, mission, duration, and a topic summary so the operator
    can identify a run without opening the bag.

The CLI is driven through its ``run`` function against a real (tmp) SQLite store so the rendering +
arg routing are tested without a process boundary; argv parsing is the thin shell.
"""

from __future__ import annotations

from pathlib import Path

import pytest
from ingest.manifest_query import render_rows, run
from ingest.manifest_store import ManifestRow, ManifestStore


def _row(bag_id: str, *, recorded_utc: str, ingested_utc: str) -> ManifestRow:
    return ManifestRow(
        bag_id=bag_id,
        mission_id="patrol",
        recorded_utc=recorded_utc,
        duration_s=10.0,
        topics_json='{"/patrol/mission_state": 1}',
        metadata_json="{}",
        ingested_utc=ingested_utc,
    )


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


# TS-12: listing modes return both seeded bags with mission/duration visible. Parametrized over the
# two modes so the identical seed → assert structure is one test, not a copied block (CodeScene).
@pytest.mark.parametrize(
    ("argv", "extra_assertions"),
    [
        (["--recent", "5"], True),
        (["--all"], False),
    ],
    ids=["recent", "all"],
)
def test_listing_mode_returns_seeded_bags(
    tmp_path: Path, capsys, argv: list[str], extra_assertions: bool
) -> None:
    store = ManifestStore(tmp_path / "m.db")
    _seed(store)

    rc = run(argv, store=store)

    out = capsys.readouterr().out
    assert rc == 0
    assert "patrol_a_20260626_080740.mcap" in out
    assert "survey_b_20260626_090000.mcap" in out
    if extra_assertions:
        assert "patrol" in out
        assert "142" in out  # duration surfaced


# F-03: --recent orders by RECORD (flown) time, not ingest time, so a rebuild can't float old bags.
# Seed two rows whose record order is the reverse of their ingest order, then assert the operator
# output lists the most-recently-FLOWN bag first.
def test_recent_orders_by_record_time(tmp_path: Path, capsys) -> None:
    store = ManifestStore(tmp_path / "m.db")
    store.upsert(
        _row(
            "flown_first.mcap",
            recorded_utc="2026-06-20T10:00:00+00:00",
            ingested_utc="2026-06-26T09:05:00+00:00",
        )  # flown first, ingested last
    )
    store.upsert(
        _row(
            "flown_last.mcap",
            recorded_utc="2026-06-25T10:00:00+00:00",
            ingested_utc="2026-06-26T09:00:00+00:00",
        )  # flown last, ingested first
    )

    rc = run(["--recent", "5"], store=store)

    out = capsys.readouterr().out
    assert rc == 0
    # The most-recently-flown bag is printed before the older one.
    assert out.index("flown_last.mcap") < out.index("flown_first.mcap")


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


# F-04 (Mira Low, review 4752923085): `--recent` below 1 is rejected at PARSE time. SQLite reads a
# negative LIMIT as *no limit*, so `--recent -1` silently dumped the entire manifest (one line per bag
# ever recorded on a long-retention DGX) and `--recent 0` returned nothing. argparse `type=` failure
# exits non-zero with an actionable message instead.
@pytest.mark.parametrize("raw", ["-1", "0", "-10"])
def test_recent_rejects_non_positive(tmp_path: Path, raw: str) -> None:
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(SystemExit):
        run(["--recent", raw], store=store)


# F-04 boundary: 1 is still a usable value — the guard rejects "below one", not "small".
def test_recent_accepts_one(tmp_path: Path, capsys) -> None:
    store = ManifestStore(tmp_path / "m.db")
    _seed(store)

    rc = run(["--recent", "1"], store=store)

    out = capsys.readouterr().out
    assert rc == 0
    assert len(out.strip().splitlines()) == 1


# `--all` joins the existing mutually-exclusive group, so combining it with `--recent` is an argparse
# error rather than a silent precedence rule the operator has to guess.
def test_all_is_mutually_exclusive_with_recent(tmp_path: Path) -> None:
    store = ManifestStore(tmp_path / "m.db")

    with pytest.raises(SystemExit):
        run(["--all", "--recent", "5"], store=store)
