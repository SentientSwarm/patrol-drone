"""Derive bag facts from the bag itself (design §3.4, §4.2.4 / SWM-76 / T8.4).

The IngestService trusts duration + per-topic counts derived from the bag, never from the sidecar.
This module provides that derivation so the logic stays testable:

  * :func:`parse_bag_metadata` — a pure-text parser over the bag's structured ``metadata.yaml``
    (ROS-free; the stable finalized-bag contract, so it is the primary source).
  * :func:`parse_bag_info` — a pure-text parser over ``ros2 bag info`` output (ROS-free, unit-tested
    against a real v1.17 capture); the fallback for a bag with no readable ``metadata.yaml``.
  * :func:`read_bag_facts` — the default :data:`~ingest.ingest_service.BagFactsReader`: prefers the
    bag's ``metadata.yaml`` and falls back to shelling out to ``ros2 bag info`` (needs a sourced ROS
    env). This is the integration boundary, exercised by the stand-in integration test.
"""

from __future__ import annotations

import re
import subprocess
from pathlib import Path

import yaml

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


def parse_bag_metadata(text: str) -> BagFacts:
    """Parse a rosbag2 ``metadata.yaml`` document into derived :class:`BagFacts`.

    The finalized-bag structured contract (``rosbag2_bagfile_information``): ``duration.nanoseconds``
    and one ``topics_with_message_count`` entry per topic. More stable across ROS releases than the
    human ``ros2 bag info`` display format, so it is the primary source; :func:`parse_bag_info`
    remains the fallback for a bag that has no readable ``metadata.yaml``.
    """
    info = (yaml.safe_load(text) or {}).get("rosbag2_bagfile_information")
    if not isinstance(info, dict):
        raise ValueError("metadata.yaml missing rosbag2_bagfile_information")
    duration_ns = info.get("duration", {}).get("nanoseconds")
    if duration_ns is None:
        raise ValueError("metadata.yaml missing duration.nanoseconds")
    topic_counts = {
        entry["topic_metadata"]["name"]: int(entry["message_count"])
        for entry in info.get("topics_with_message_count", [])
    }
    if not topic_counts:
        raise ValueError("metadata.yaml produced no parseable topics (empty topic set)")
    return BagFacts(duration_s=float(duration_ns) / 1e9, topic_counts=topic_counts)


def read_bag_facts(bag_path: Path, timeout_s: float = _BAG_INFO_TIMEOUT_S) -> BagFacts:
    """Default reader: prefer the bag's structured ``metadata.yaml``; fall back to ``ros2 bag info``.

    ``metadata.yaml`` is a stable structured contract that needs no ROS env; the ``ros2 bag info``
    parse (which needs a sourced ROS env and is ``timeout_s``-bounded so a stuck ``ros2`` can't block
    the serial ingest loop) is the fallback for a bag missing/with an unreadable metadata file. Both
    paths derive facts *from the bag* — never the sidecar (the dumb-producer invariant, design §3.4).
    """
    metadata = bag_path / "metadata.yaml"
    if metadata.is_file():
        return parse_bag_metadata(metadata.read_text())
    completed = subprocess.run(
        ["ros2", "bag", "info", str(bag_path)],
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout_s,
    )
    return parse_bag_info(completed.stdout)
