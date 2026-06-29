"""Generate the trimmed replay reference bag (docset 05-logging-replay, M8 / T8.7, SWM-79).

Reads a full M7 patrol bag and writes a small, deterministic MCAP slice the replay regression test
can play in CI within the ≤90 s budget (design OQ-6) and that Git LFS can hold cheaply (OQ-4). This
script IS the documented regeneration procedure (LR-9) — re-run it when the recorded topic set
legitimately changes so the replay baseline doesn't rot.

Trim levers, applied together to get from ~99 MB → single-digit MB:
  * topic filter — keep only the asserted subset (design §4.2.5) + /tf for Foxglove pose.
  * time window — keep ``--seconds`` of messages, anchored so the window CONTAINS the first
    ``/patrol/checkpoint_capture`` (the replay test asserts that topic's count > 0). The window
    starts ``--lead`` seconds before the first capture so the approach + telemetry are present too.
  * camera downsample — keep every Nth CompressedImage so imagery is present (asserted) but small.

Pick a source bag whose patrol actually captured (post-ADR-0012); e.g. patrol_20260627T140037Z
has 15 captures in 161 s. Needs a sourced ROS env (rosbag2_py). Run:
    python3 tests/replay/reference/make_reference_bag.py \\
        --source ~/patrol_bags/patrol_20260627T140037Z_20260627_140037 \\
        --out tests/replay/reference/patrol_reference --seconds 20 --camera-every 5
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import rosbag2_py

_ASSERTED_TOPICS = {
    "/patrol/mission_state",
    "/patrol/current_waypoint",
    "/patrol/checkpoint_capture",
    "/drone/camera/image_raw/compressed",
    "/fmu/out/vehicle_local_position_v1",
    "/tf",
}
_CAMERA_TOPIC = "/drone/camera/image_raw/compressed"
_ANCHOR_TOPIC = (
    "/patrol/checkpoint_capture"  # the window must contain ≥1 of these (replay asserts >0)
)


def _reader(uri: str) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return reader


def _writer(uri: str) -> rosbag2_py.SequentialWriter:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    return writer


def _first_capture_stamp(source: str) -> int:
    """Return the timestamp (ns) of the first ``/patrol/checkpoint_capture`` message in ``source``."""
    reader = _reader(source)
    while reader.has_next():
        topic, _data, stamp = reader.read_next()
        if topic == _ANCHOR_TOPIC:
            return stamp
    raise ValueError(
        f"{source} has no {_ANCHOR_TOPIC} messages — pick a source bag that actually captured"
    )


def make_reference_bag(
    source: str, out: str, seconds: float, camera_every: int, lead: float = 8.0
) -> None:
    """Write a trimmed slice of ``source`` to ``out`` (overwrites ``out`` if present).

    The window is anchored ``lead`` seconds before the first checkpoint capture and spans
    ``seconds`` total, so the reference bag is guaranteed to contain a capture (replay asserts > 0).
    """
    out_path = Path(out)
    if out_path.exists():
        shutil.rmtree(out_path)

    start_ns = _first_capture_stamp(source) - int(lead * 1e9)
    end_ns = start_ns + int(seconds * 1e9)

    reader = _reader(source)
    kept_types = {
        t.name: t for t in reader.get_all_topics_and_types() if t.name in _ASSERTED_TOPICS
    }
    writer = _writer(out)
    for topic in kept_types.values():
        writer.create_topic(topic)

    camera_seen = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic not in kept_types or stamp < start_ns:
            continue
        if stamp > end_ns:
            break
        if topic == _CAMERA_TOPIC:
            camera_seen += 1
            if camera_seen % camera_every != 0:
                continue
        writer.write(topic, data, stamp)
    del writer  # flush/finalize the MCAP


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="full M7 patrol bag dir to trim from")
    parser.add_argument("--out", required=True, help="output reference bag dir")
    parser.add_argument("--seconds", type=float, default=20.0, help="time window from bag start")
    parser.add_argument("--camera-every", type=int, default=5, help="keep every Nth camera frame")
    parser.add_argument("--lead", type=float, default=8.0, help="seconds before first capture")
    args = parser.parse_args()
    make_reference_bag(args.source, args.out, args.seconds, args.camera_every, args.lead)


if __name__ == "__main__":
    main()
