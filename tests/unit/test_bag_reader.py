"""Layer-A unit tests for the bag-fact parsers (docset 05-logging-replay, M8 / T8.4, SWM-76).

Covers `ingest.bag_reader.parse_bag_metadata` (the structured ``metadata.yaml`` primary path) and
`parse_bag_info` (the ``ros2 bag info`` text fallback) — both turn a bag into the
:class:`~ingest.ingest_service.BagFacts` the IngestService trusts (design §3.4, §4.2.4). Parsing is
separated from the subprocess/file call so it is ROS-free and unit-testable; the ``ros2 bag info``
invocation itself is the integration boundary.

Sample inputs are trimmed real captures of the M7 reference bag so each parser is tested against the
actual format (``Duration: <float>s`` / ``Topic: … | Count: N``; and the ``metadata.yaml``
``rosbag2_bagfile_information`` schema rosbag2 v9 writes).
"""

from __future__ import annotations

import pytest
from ingest.bag_reader import parse_bag_info, parse_bag_metadata

_SAMPLE = """
Files:             patrol_patrol_20260626_080740_0.mcap
Bag size:          98.5 MiB
Storage id:        mcap
ROS Distro:        jazzy
Duration:          142.317327555s
Start:             Jun 26 2026 11:07:40.958558939 (1782461260.958558939)
End:               Jun 26 2026 11:10:03.275886494 (1782461403.275886494)
Messages:          76780
Topic information: Topic: /drone/camera/image_raw/compressed | Type: sensor_msgs/msg/CompressedImage | Count: 6465 | Serialization Format: cdr
                   Topic: /fmu/out/vehicle_local_position_v1 | Type: px4_msgs/msg/VehicleLocalPosition | Count: 7116 | Serialization Format: cdr
                   Topic: /patrol/mission_state | Type: std_msgs/msg/String | Count: 1420 | Serialization Format: cdr
                   Topic: /patrol/checkpoint_capture | Type: patrol_interfaces/msg/CheckpointCapture | Count: 9 | Serialization Format: cdr
"""


# TS-9 (parser half): Duration is parsed from the "Duration: <float>s" line.
def test_parse_duration_seconds() -> None:
    facts = parse_bag_info(_SAMPLE)
    assert abs(facts.duration_s - 142.317327555) < 1e-6


# TS-9 (parser half): per-topic counts are parsed from each "Topic: ... | Count: N" line.
def test_parse_topic_counts() -> None:
    facts = parse_bag_info(_SAMPLE)
    assert facts.topic_counts["/drone/camera/image_raw/compressed"] == 6465
    assert facts.topic_counts["/fmu/out/vehicle_local_position_v1"] == 7116
    assert facts.topic_counts["/patrol/mission_state"] == 1420
    assert facts.topic_counts["/patrol/checkpoint_capture"] == 9


# All four topics in the sample are captured (no lines dropped).
def test_parse_captures_all_topics() -> None:
    facts = parse_bag_info(_SAMPLE)
    assert len(facts.topic_counts) == 4


# Guard: output with no Duration line fails loudly (a corrupt/truncated bag-info is not silently 0s).
def test_parse_raises_when_duration_absent() -> None:
    with pytest.raises(ValueError, match="Duration"):
        parse_bag_info("Files: x.mcap\nStorage id: mcap\n")


# F-02: a parseable Duration but a topic section the regex never matches must fail loud, not
# silently write an empty topic set to the manifest (the ValueError slots into _INGEST_FAULTS).
def test_parse_raises_when_no_topics_parsed() -> None:
    text = "Duration:          1.0s\nTopic information: (unparseable topic section)\n"
    with pytest.raises(ValueError, match="no parseable topics"):
        parse_bag_info(text)


# A trimmed real reference metadata.yaml (rosbag2 v9 `rosbag2_bagfile_information` schema): two
# topics, duration in nanoseconds. Full schema (QoS blocks, hashes) omitted — the parser only
# reads duration.nanoseconds + each topics_with_message_count entry's name/message_count.
_METADATA_SAMPLE = """
rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  duration:
    nanoseconds: 19991536152
  message_count: 503
  topics_with_message_count:
    - topic_metadata:
        name: /patrol/mission_state
        type: std_msgs/msg/String
      message_count: 200
    - topic_metadata:
        name: /drone/camera/image_raw/compressed
        type: sensor_msgs/msg/CompressedImage
      message_count: 303
"""


# TS-9 (metadata half): duration.nanoseconds is converted to seconds.
def test_parse_metadata_duration_seconds() -> None:
    facts = parse_bag_metadata(_METADATA_SAMPLE)
    assert abs(facts.duration_s - 19.991536152) < 1e-6


# TS-9 (metadata half): per-topic counts come from each topics_with_message_count entry.
def test_parse_metadata_topic_counts() -> None:
    facts = parse_bag_metadata(_METADATA_SAMPLE)
    assert facts.topic_counts == {
        "/patrol/mission_state": 200,
        "/drone/camera/image_raw/compressed": 303,
    }


# F-02 fail-loud guards: each malformed metadata.yaml raises ValueError (→ _INGEST_FAULTS → skip +
# retry) rather than writing a wrong/empty manifest. Parametrized to avoid duplicate `raises` blocks.
@pytest.mark.parametrize(
    ("text", "match"),
    [
        ("some_other_root: {}\n", "rosbag2_bagfile_information"),
        ("rosbag2_bagfile_information:\n  message_count: 1\n", "duration.nanoseconds"),
        (
            "rosbag2_bagfile_information:\n"
            "  duration:\n    nanoseconds: 1000\n"
            "  topics_with_message_count: []\n",
            "no parseable topics",
        ),
    ],
    ids=["missing-root", "missing-duration", "empty-topics"],
)
def test_parse_metadata_raises_on_malformed(text: str, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        parse_bag_metadata(text)
