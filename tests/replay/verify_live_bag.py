#!/usr/bin/env python3
"""Manual/e2e live-bag witness — RTF-robust rate check (docset 05-logging-replay, M8 / LR-8).

``analysis/e2e_check.md`` step 4 witnesses that the *freshly-recorded live* bag carries every asserted
topic at its expected rate. On a GUI-loaded host that check cannot trust ``ros2 bag info`` count /
duration: the sim sags below real-time and the record path can double-deliver rendered frames while
the MCAP summary is written inconsistently, so a raw ``ros2 bag info`` rate reads ~2x high and trips
the band with a false failure (measured: camera 30.3 Hz vs a true 15.15 Hz — see ADR-0013).

This tool measures the TRUE rate from each topic's own de-duplicated message timestamps, but only
after a **consistency guard** confirms the bag is trustworthy — a demonstrably inconsistent /
duplicated / non-finalized bag is a hard fail with a "re-record" reason, never a silent PASS.

Run under system python with ROS sourced (NOT the uv .venv — the CLAUDE.md numpy/uv boundary). It
reads only raw CDR bytes + log_time, so it never triggers a full rosidl deserialize (no numpy):

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 tests/replay/verify_live_bag.py --bag ~/patrol_bags/<bag>

Exit 0 iff the bag is consistent AND every asserted topic passes presence + RTF-robust rate.
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

# Self-bootstrap the dirs this script imports first-party modules from (mirrors
# test_replay_regression.py): tests/replay → rate_report/replay_assertions ; docker → ingest.bag_reader.
_HERE = Path(__file__).resolve().parent
for _p in (_HERE, _HERE.parents[1] / "docker"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from ingest.bag_reader import read_bag_facts  # noqa: E402  (after the sys.path bootstrap above)
from rate_report import (  # noqa: E402  (after the sys.path bootstrap above)
    TopicSample,
    consistency_verdict,
    evaluate_true_rates,
    unique_stamp_rate,
)
from replay_assertions import load_specs  # noqa: E402  (after the sys.path bootstrap above)

_ASSERTIONS = _HERE / "assertions.yaml"

# Types whose CDR payload begins with a std_msgs/Header, so the sim-time stamp is the (sec int32,
# nanosec uint32) pair right after the 4-byte CDR encapsulation. Only these get a sim-stamp dup guard;
# every other type (header-less std_msgs, px4 — no std Header, /tf — leads with an array) passes no
# sim stamps and is de-duplicated for rate purposes on log_time alone.
_HEADER_LEADING_TYPES = frozenset(
    {"sensor_msgs/msg/CompressedImage", "apriltag_msgs/msg/AprilTagDetectionArray"}
)
_ENCAPSULATION_BYTES = 4


def _header_stamp_ns(data: bytes) -> int | None:
    """The sim-time stamp of a Header-leading message, or None if the buffer is too short to hold one."""
    if len(data) < _ENCAPSULATION_BYTES + 8:
        return None
    sec, nsec = struct.unpack_from("<iI", data, _ENCAPSULATION_BYTES)
    return sec * 1_000_000_000 + nsec


def _open_reader(bag: Path):
    """Open a rosbag2 SequentialReader over ``bag`` and return (reader, {topic: type})."""
    # rosbag2_py is ROS-only (absent under the uv .venv), so it is imported here, not at module top,
    # to keep this file importable by ruff/formatters off a sourced ROS env.
    from rosbag2_py import (  # noqa: PLC0415
        ConverterOptions,
        SequentialReader,
        StorageOptions,
    )

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def _read_samples(bag: Path, topics: set[str]) -> dict[str, TopicSample]:
    """Read ``bag`` once; per asserted topic collect reader rows, log_time (rate key) + sim stamps."""
    reader, types = _open_reader(bag)
    rows: dict[str, int] = dict.fromkeys(topics, 0)
    log_times: dict[str, list[int]] = {t: [] for t in topics}
    sim_stamps: dict[str, list[int]] = {t: [] for t in topics}
    header_topics = {t for t in topics if types.get(t) in _HEADER_LEADING_TYPES}

    while reader.has_next():
        topic, data, log_time = reader.read_next()
        if topic not in topics:
            continue
        rows[topic] += 1
        log_times[topic].append(log_time)
        if topic in header_topics and (stamp := _header_stamp_ns(data)) is not None:
            sim_stamps[topic].append(stamp)

    info_counts = read_bag_facts(bag).topic_counts
    return {
        t: TopicSample(
            t,
            info_count=info_counts.get(t, 0),
            reader_rows=rows[t],
            rate_stamps_ns=log_times[t],
            sim_stamps_ns=sim_stamps[t],
        )
        for t in topics
    }


def _report(samples: dict[str, TopicSample]) -> None:
    """Print the per-topic read summary (rows / info / rate) for the operator's log."""
    print("topic                                        info   rows   rate(Hz)")
    for s in sorted(samples.values(), key=lambda x: x.topic):
        rate = unique_stamp_rate(s.rate_stamps_ns)
        print(f"  {s.topic:42}{s.info_count:>6}{s.reader_rows:>7}{rate:>10.2f}")


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--bag", type=Path, required=True, help="path to the recorded bag directory"
    )
    parser.add_argument(
        "--assertions",
        type=Path,
        default=_ASSERTIONS,
        help="assertions.yaml (default: the shipped one)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    specs = load_specs(args.assertions)
    samples_by_topic = _read_samples(args.bag, {s.topic for s in specs})
    samples = list(samples_by_topic.values())

    _report(samples_by_topic)

    ok, reasons = consistency_verdict(samples)
    if not ok:
        print(
            "\nBAG CONSISTENCY: FAIL — this bag is untrustworthy; re-record once the sim holds "
            "real-time (do not trust its rates):"
        )
        for reason in reasons:
            print(f"  - {reason}")
        return 2
    print(
        "\nBAG CONSISTENCY: OK (ros2 bag info counts match the message stream; no frame duplication)"
    )

    result = evaluate_true_rates(specs, samples)
    if result.passed:
        print(
            "REPLAY RATE CHECK: PASS — every asserted topic present at its expected rate (RTF-robust)"
        )
        return 0
    print("REPLAY RATE CHECK: FAIL:")
    for failure in result.failures:
        print(f"  - {failure}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
