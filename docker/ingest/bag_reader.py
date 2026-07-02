"""Derive bag facts from ``ros2 bag info`` (design §3.4, §4.2.4 / SWM-76 / T8.4).

The IngestService trusts duration + per-topic counts derived from the bag, never from the sidecar.
This module provides that derivation in two halves so the logic stays testable:

  * :func:`parse_bag_info` — a pure-text parser over ``ros2 bag info`` output (ROS-free, unit-tested
    against a real v1.17 capture).
  * :func:`read_bag_facts` — the default :data:`~ingest.ingest_service.BagFactsReader`: shells out
    to ``ros2 bag info`` and feeds the parser. This is the integration boundary (needs a sourced
    ROS env), exercised by the stand-in integration test.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

from ingest.ingest_service import BagFacts

# "Duration:          142.317327555s"
_DURATION_RE = re.compile(r"^\s*Duration:\s*([0-9.]+)s", re.MULTILINE)
# "Topic: /name | Type: ... | Count: 1420 | Serialization Format: cdr"
_TOPIC_RE = re.compile(r"Topic:\s*(\S+)\s*\|.*?Count:\s*(\d+)")

# `ros2 bag info` should return in well under a second for a finalized bag; cap it generously so a
# hung/pathological CLI can't wedge the serial ingest loop forever (PR #16 / F-03). On timeout,
# subprocess raises TimeoutExpired, which _INGEST_FAULTS catches → the bag is skipped + retried.
_BAG_INFO_TIMEOUT_S = 120.0


def parse_bag_info(text: str) -> BagFacts:
    """Parse ``ros2 bag info`` ``text`` into derived :class:`BagFacts` (duration + topic counts)."""
    duration_match = _DURATION_RE.search(text)
    if duration_match is None:
        raise ValueError("could not parse Duration from ros2 bag info output")
    topic_counts = {name: int(count) for name, count in _TOPIC_RE.findall(text)}
    if not topic_counts:
        raise ValueError("ros2 bag info produced no parseable topics (empty topic set)")
    return BagFacts(duration_s=float(duration_match.group(1)), topic_counts=topic_counts)


def read_bag_facts(bag_path: Path, timeout_s: float = _BAG_INFO_TIMEOUT_S) -> BagFacts:
    """Default reader: run ``ros2 bag info <bag>`` and parse it (needs a sourced ROS env).

    ``timeout_s`` bounds the CLI call so a stuck ``ros2`` can't block the serial ingest loop; on
    expiry ``subprocess.TimeoutExpired`` propagates and the watch loop skips-and-retries the bag.
    """
    completed = subprocess.run(
        ["ros2", "bag", "info", str(bag_path)],
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    return parse_bag_info(completed.stdout)
