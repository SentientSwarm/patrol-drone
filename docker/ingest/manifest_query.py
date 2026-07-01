"""The manifest query CLI: list recent runs + their topic sets (design §4.2.4, LR-4 / SWM-77 / T8.5).

The operator surface over :class:`~ingest.manifest_store.ManifestStore` — "list recent runs and
what each bag contains" instead of grepping a directory of bags. ``--recent N`` lists the N newest
indexed bags; ``--mission <id>`` filters to one mission. Each line names the bag, mission, duration,
and a topic-count summary so a run is identifiable without opening the bag.

Run as ``python -m ingest.manifest_query --recent 5 [--db <path>]`` on the DGX (or CI stand-in).
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from ingest.manifest_store import ManifestRow, ManifestStore

_DEFAULT_DB_ENV = "PATROL_MANIFEST_DB"


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
    group.add_argument("--recent", type=int, metavar="N", help="list the N most-recent bags")
    group.add_argument("--mission", metavar="ID", help="list bags for one mission id")
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
