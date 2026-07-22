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

from pathlib import Path

import pytest
from ingest.bag_reader import parse_bag_info, parse_bag_metadata, read_bag_facts

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
# The valid-YAML/schema-invalid rows (list root, scalar duration, non-list/non-dict topics) are the
# F-02 core: before the fix, list-root and scalar-duration raised an UNCAUGHT AttributeError that
# crashed the ingest watcher; scalar-topics / non-dict-entry raised TypeError. All now normalize to
# ValueError so the contract is uniform: any structurally-invalid metadata → skip + retry.
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
        ("- a\n- b\n", "root has unexpected shape"),
        ("5\n", "root has unexpected shape"),
        (
            "rosbag2_bagfile_information:\n  duration: 5\n  topics_with_message_count: []\n",
            "duration has unexpected shape",
        ),
        (
            "rosbag2_bagfile_information:\n"
            "  duration:\n    nanoseconds: 5\n"
            "  topics_with_message_count: 7\n",
            "topics_with_message_count has unexpected shape",
        ),
        (
            "rosbag2_bagfile_information:\n"
            "  duration:\n    nanoseconds: 5\n"
            "  topics_with_message_count:\n    - 7\n",
            "topics_with_message_count entry has unexpected shape",
        ),
    ],
    ids=[
        "missing-root",
        "missing-duration",
        "empty-topics",
        "list-root",
        "scalar-root",
        "scalar-duration",
        "scalar-topics",
        "nondict-topic-entry",
    ],
)
def test_parse_metadata_raises_on_malformed(text: str, match: str) -> None:
    with pytest.raises(ValueError, match=match):
        parse_bag_metadata(text)


# --- Fix 13 / F-01 sub-clause (Mira High, review 4752923085): "validates metadata references where
# available". The shared bag-layout validator proves *a* payload exists; read_bag_facts additionally
# proves the payloads THIS document declares are the ones on disk, so facts are never derived from a
# description of storage files that aren't there. Ingest-only by design: bag_reader already imports
# yaml and already parses metadata.yaml, whereas the producer side is deliberately stdlib-pure.

# The shared _METADATA_SAMPLE above deliberately omits relative_file_paths (it is a trimmed capture,
# and the existing parse tests assert against it) — so the reference-declaring shape gets its own
# constant rather than mutating one every other test depends on.
_METADATA_WITH_PAYLOAD_REF = """
rosbag2_bagfile_information:
  version: 9
  storage_identifier: mcap
  relative_file_paths:
    - patrol_ref_0.mcap
  duration:
    nanoseconds: 19991536152
  message_count: 200
  topics_with_message_count:
    - topic_metadata:
        name: /patrol/mission_state
        type: std_msgs/msg/String
      message_count: 200
"""


def _bag_declaring_a_payload(tmp_path: Path, declared: str = "patrol_ref_0.mcap") -> Path:
    """A bag dir whose metadata.yaml DECLARES ``declared``; the payload itself is the variable.

    ``declared`` defaults to the well-behaved relative name; the escaping-path cases below substitute
    an absolute / traversing one so the containment guard is exercised through the real reader.
    """
    bag = tmp_path / "patrol_ref"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        _METADATA_WITH_PAYLOAD_REF.replace("patrol_ref_0.mcap", declared)
    )
    return bag


# A declared payload that is missing (a half-landed rsync) or symlinked (a redirect out of the
# landing tree) makes the structured branch fail loudly. ValueError is an _INGEST_FAULTS member, so
# the watch loop logs + skips + retries rather than indexing facts nothing on disk backs.
@pytest.mark.parametrize("payload", ["missing", "symlink"], ids=["missing", "symlinked"])
def test_read_bag_facts_raises_when_a_declared_payload_is_not_a_real_file(
    tmp_path: Path, payload: str
) -> None:
    bag = _bag_declaring_a_payload(tmp_path)
    if payload == "symlink":
        redirect = tmp_path / "payload-elsewhere.mcap"
        redirect.write_bytes(b"\x89MCAP0\r\n")
        (bag / "patrol_ref_0.mcap").symlink_to(redirect)

    with pytest.raises(ValueError, match="declares payload"):
        read_bag_facts(bag)


# A declared payload path that ESCAPES the bag dir is refused BEFORE the join (PR #16 / F-01, Mira
# High, review 4754192970). `bag_path / "/etc/hostname"` is `/etc/hostname` — the left operand is
# discarded — so the old guard happily "validated" a real file outside the bag and then derived the
# manifest's duration + topic counts from a document describing it. A real sibling .mcap is written
# so the rejection is provably caused by the DECLARATION, not by an empty directory.
@pytest.mark.parametrize(
    "declared",
    ["/etc/hostname", "../../other_bag/x_0.mcap"],
    ids=["absolute", "traversal"],
)
def test_read_bag_facts_rejects_a_declared_payload_outside_the_bag(
    tmp_path: Path, declared: str
) -> None:
    bag = _bag_declaring_a_payload(tmp_path, declared)
    (bag / "patrol_ref_0.mcap").write_bytes(b"\x89MCAP0\r\n")

    with pytest.raises(ValueError, match="outside its bag directory"):
        read_bag_facts(bag)


# The happy path: a declared payload that IS on disk parses normally, so the guard costs nothing for
# a genuine bag (this is the shape the checked-in LFS reference bag has).
def test_read_bag_facts_accepts_a_declared_payload_that_exists(tmp_path: Path) -> None:
    bag = _bag_declaring_a_payload(tmp_path)
    (bag / "patrol_ref_0.mcap").write_bytes(b"\x89MCAP0\r\n")

    facts = read_bag_facts(bag)

    assert facts.topic_counts == {"/patrol/mission_state": 200}


# The review's "where available" hedge: a metadata.yaml with NO relative_file_paths is a legitimate
# no-op, not a fault — an older/trimmed document must not be newly rejected by this guard.
def test_read_bag_facts_accepts_metadata_without_relative_file_paths(tmp_path: Path) -> None:
    bag = tmp_path / "patrol_norefs"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(_METADATA_SAMPLE)

    facts = read_bag_facts(bag)

    assert facts.topic_counts  # non-empty: the document still parsed


# A non-list relative_file_paths is a SHAPE fault, normalized to ValueError like every other guard in
# this module (never an uncaught AttributeError that would crash the whole ingest watcher).
def test_read_bag_facts_raises_on_non_list_relative_file_paths(tmp_path: Path) -> None:
    bag = tmp_path / "patrol_badrefs"
    bag.mkdir()
    (bag / "metadata.yaml").write_text(
        _METADATA_WITH_PAYLOAD_REF.replace(
            "  relative_file_paths:\n    - patrol_ref_0.mcap\n", "  relative_file_paths: 7\n"
        )
    )

    with pytest.raises(ValueError, match="relative_file_paths has unexpected shape"):
        read_bag_facts(bag)
