"""Layer-A unit tests for the bag-fact parser (docset 05-logging-replay, M8 / T8.4, SWM-76).

Covers `ingest.bag_reader.parse_bag_info` — the pure-text parser that turns ``ros2 bag info``
output into the :class:`~ingest.ingest_service.BagFacts` the IngestService trusts (design §3.4,
§4.2.4). Parsing is separated from the subprocess call so it is ROS-free and unit-testable; the
``ros2 bag info`` invocation itself is the integration boundary.

Sample input is a trimmed real ``ros2 bag info`` capture (the M7 reference bag) so the parser is
tested against the actual v1.17 format (``Duration: <float>s``; ``Topic: <name> | ... | Count: N``).
"""

from __future__ import annotations

import pytest
from ingest.bag_reader import parse_bag_info

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
