"""The manifest query CLI: list recent runs + their topic sets (design §4.2.4, LR-4 / SWM-77 / T8.5).

The operator surface over :class:`~ingest.manifest_store.ManifestStore` — "list recent runs and
what each bag contains" instead of grepping a directory of bags. ``--recent N`` lists the N newest
indexed bags (N >= 1); ``--mission <id>`` filters to one mission; ``--all`` lists everything under a
bounded cap. Each line names the bag, mission, duration, and a topic-count summary so a run is
identifiable without opening the bag.

Run as ``python -m ingest.manifest_query --recent 5 [--db <path>]`` on the DGX (or CI stand-in).
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from ingest.manifest_store import ManifestRow, ManifestStore
from ingest.positive_interval import positive_int

_DEFAULT_DB_ENV = "PATROL_MANIFEST_DB"

# An "all results" request is still a BOUNDED query — the point of the F-04 fix is that no code path
# reaches SQLite with an unbounded LIMIT. The cap is far above any realistic DGX retention window, so
# it never truncates in practice; it exists so "everything" can never mean "unbounded".
_ALL_ROWS_CAP = 10_000


def render_rows(rows: list[ManifestRow]) -> list[str]:
    """Render each manifest row to a one-line operator summary (bag · mission · duration · topics)."""
    lines: list[str] = []
    for r in rows:
        topic_count = len(json.loads(r.topics_json))
        lines.append(
            f"{r.bag_id}  mission={r.mission_id}  "
            f"{r.duration_s:.0f}s  {topic_count} topics  ({r.recorded_utc})"
        )
    return lines


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="manifest_query", description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--recent",
        type=positive_int,
        metavar="N",
        # SQLite reads a NEGATIVE `LIMIT` as *no limit*, so a bare `type=int` let `--recent -1`
        # silently dump the whole manifest (and `--recent 0` return nothing). Rejecting < 1 at parse
        # time turns an operator typo into an actionable error (Mira Low, review 4752923085).
        help="list the N most-recent bags (N >= 1)",
    )
    group.add_argument("--mission", metavar="ID", help="list bags for one mission id")
    group.add_argument(
        "--all",
        action="store_true",
        help=f"list every indexed bag (bounded at {_ALL_ROWS_CAP})",
    )
    parser.add_argument(
        "--db",
        type=Path,
        default=os.environ.get(_DEFAULT_DB_ENV),
        help=f"manifest SQLite path (default: ${_DEFAULT_DB_ENV})",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None, *, store: ManifestStore) -> int:
    """Execute the query against ``store`` and print results; returns a process exit code."""
    args = _parse_args(argv)
    if args.mission is not None:
        rows = store.query_by_mission(args.mission)
    elif args.all:
        rows = store.query_recently_recorded(_ALL_ROWS_CAP)
    else:
        rows = store.query_recently_recorded(args.recent if args.recent is not None else 10)

    lines = render_rows(rows)
    if not lines:
        print("no bags in manifest")
        return 0
    for line in lines:
        print(line)
    return 0


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.db is None:
        raise SystemExit(
            f"no manifest db: pass --db <path> or set ${_DEFAULT_DB_ENV}",
        )
    return run(argv, store=ManifestStore(args.db))


if __name__ == "__main__":
    raise SystemExit(main())
