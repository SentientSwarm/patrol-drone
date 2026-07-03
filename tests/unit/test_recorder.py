"""Layer-A unit tests for the ROS-free recorder core (docset 05-logging-replay, M7).

Covers the three pieces of `patrol_logging.recorder` that carry no ROS dependency and therefore
run on the per-PR pure-Python tier (<5 s, no rclpy/rosbag2/Gazebo):

  * ``bag_name`` — the ``patrol_<missionId>_<timestamp>`` naming contract (DoD AC-1).
  * ``build_record_argv`` — the ``ros2 bag record --storage mcap`` argv (MCAP not sqlite3; the
    broad topic set as positional topics + ``--regex`` patterns) (DoD AC-2).
  * ``BagSidecar`` / ``build_sidecar`` / ``write_sidecar`` — the JSON metadata sidecar that
    identifies + correlates a run (DoD AC-2, design §4.2.2 + OQ-10).

The launch/subprocess plumbing (record.launch.py spawning the process, SIGINT-finalize) is the
thin ROS layer, verified by colcon build + the nightly SITL bag-producing check — not here.
"""

from __future__ import annotations

import json
from datetime import UTC, datetime

import pytest
from patrol_logging.recorder import (
    BagSidecar,
    RecordingRun,
    bag_name,
    build_record_argv,
    build_sidecar,
    finalize_sidecar_from_staging,
    read_sidecar_inputs,
    sidecar_inputs_path,
    write_sidecar,
    write_sidecar_inputs,
)

# A fixed instant so every name/timestamp assertion is deterministic (no wall-clock).
_STARTED = datetime(2026, 6, 26, 14, 5, 9, tzinfo=UTC)
_ENDED = datetime(2026, 6, 26, 14, 10, 9, tzinfo=UTC)
_TS = "20260626_140509"  # bag_name's %Y%m%d_%H%M%S rendering of _STARTED

_NAMED_TOPICS = [
    "/patrol/mission_state",
    "/patrol/current_waypoint",
    "/patrol/checkpoint_capture",
    "/drone/camera/image_raw/compressed",
    "/tf",
    "/tf_static",
]
_REGEXES = ["/fmu/out/.*"]


# --- bag_name (DoD AC-1: patrol_<missionId>_<timestamp>) -------------------------------------


@pytest.mark.parametrize(
    ("mission_id", "expected"),
    [
        ("alpha", f"patrol_alpha_{_TS}"),
        ("patrol_mission", f"patrol_patrol_mission_{_TS}"),
        ("run-7", f"patrol_run-7_{_TS}"),
    ],
)
def test_bag_name_matches_convention(mission_id: str, expected: str) -> None:
    assert bag_name(mission_id, _STARTED) == expected


@pytest.mark.parametrize(
    ("raw_mission_id", "expected_segment"),
    [
        ("a/b", "a_b"),  # path separator can't escape the output dir
        ("a b", "a_b"),  # whitespace -> underscore
        ("a:b*c", "a_b_c"),  # fs-hostile chars sanitized (mirrors M6 checkpoint_id fs-safety)
    ],
)
def test_bag_name_sanitizes_fs_hostile_mission_id(
    raw_mission_id: str, expected_segment: str
) -> None:
    assert bag_name(raw_mission_id, _STARTED) == f"patrol_{expected_segment}_{_TS}"


def test_bag_name_rejects_empty_mission_id() -> None:
    with pytest.raises(ValueError, match="mission_id"):
        bag_name("", _STARTED)


# --- build_record_argv (DoD AC-2: MCAP, broad set, -o path) ----------------------------------


@pytest.fixture
def argv(tmp_path) -> list[str]:
    return build_record_argv(
        output_dir=tmp_path,
        bag_basename=f"patrol_alpha_{_TS}",
        topics=_NAMED_TOPICS,
        regexes=_REGEXES,
    )


@pytest.mark.parametrize(
    "expected_flag_pair",
    [
        ("--storage", "mcap"),  # MCAP plugin, NOT sqlite3 (settled constraint)
    ],
)
def test_argv_selects_mcap_storage(argv: list[str], expected_flag_pair: tuple[str, str]) -> None:
    flag, value = expected_flag_pair
    assert flag in argv
    assert argv[argv.index(flag) + 1] == value
    assert "sqlite3" not in argv


def test_argv_is_a_ros2_bag_record_invocation(argv: list[str]) -> None:
    assert argv[:3] == ["ros2", "bag", "record"]


def test_argv_output_path_is_under_output_dir(argv: list[str], tmp_path) -> None:
    out = argv[argv.index("-o") + 1]
    assert out == str(tmp_path / f"patrol_alpha_{_TS}")


@pytest.mark.parametrize("topic", _NAMED_TOPICS)
def test_argv_records_each_named_topic(argv: list[str], topic: str) -> None:
    assert topic in argv


def test_argv_passes_named_topics_via_topics_flag(argv: list[str]) -> None:
    # Jazzy deprecates bare positional topics; they must follow an explicit --topics flag so the
    # invocation stays forward-compatible (no ros2bag deprecation warning).
    assert "--topics" in argv
    topics_idx = argv.index("--topics")
    for topic in _NAMED_TOPICS:
        assert argv.index(topic) > topics_idx


@pytest.mark.parametrize("regex", _REGEXES)
def test_argv_records_each_regex_pattern(argv: list[str], regex: str) -> None:
    assert "--regex" in argv
    assert regex in argv


def test_argv_requires_at_least_one_topic_or_regex(tmp_path) -> None:
    with pytest.raises(ValueError, match="topic"):
        build_record_argv(output_dir=tmp_path, bag_basename="patrol_x_y", topics=[], regexes=[])


# --- sidecar (DoD AC-2 / design §4.2.2 / OQ-10 JSON) -----------------------------------------


def _sample_run() -> RecordingRun:
    return RecordingRun(
        mission_id="alpha",
        bag_uri=f"patrol_alpha_{_TS}",  # bag DIRECTORY (rosbag2 -o URI), no .mcap extension
        started=_STARTED,
        mission_config_ref="/abs/patrol_mission.yaml",
    )


def _sample_sidecar() -> BagSidecar:
    return build_sidecar(_sample_run(), _ENDED, _NAMED_TOPICS + _REGEXES)


@pytest.mark.parametrize(
    ("field", "expected"),
    [
        ("mission_id", "alpha"),
        ("bag_uri", f"patrol_alpha_{_TS}"),
        ("started_utc", "2026-06-26T14:05:09+00:00"),
        ("ended_utc", "2026-06-26T14:10:09+00:00"),
        ("mission_config_ref", "/abs/patrol_mission.yaml"),
    ],
)
def test_sidecar_carries_identity_fields(field: str, expected: str) -> None:
    assert getattr(_sample_sidecar(), field) == expected


def test_sidecar_bag_uri_is_a_directory_not_an_mcap_file() -> None:
    # rosbag2's -o makes a bag DIRECTORY; the .mcap storage file is nested inside (F-02). The sidecar
    # records the directory URI, never a top-level .mcap that doesn't exist.
    assert not _sample_sidecar().bag_uri.endswith(".mcap")


def test_sidecar_records_the_requested_topic_set() -> None:
    assert _sample_sidecar().recorded_topics == _NAMED_TOPICS + _REGEXES


def test_write_sidecar_round_trips_as_json(tmp_path) -> None:
    sidecar = _sample_sidecar()
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"

    write_sidecar(path, sidecar)

    loaded = json.loads(path.read_text())
    assert loaded["mission_id"] == "alpha"
    assert loaded["bag_uri"] == f"patrol_alpha_{_TS}"
    assert loaded["started_utc"] == "2026-06-26T14:05:09+00:00"
    assert loaded["recorded_topics"] == _NAMED_TOPICS + _REGEXES
    assert loaded["mission_config_ref"] == "/abs/patrol_mission.yaml"


# --- staging file + caller-independent finalize (the missing-sidecar fix) ---------------------


def test_sidecar_inputs_round_trip(tmp_path) -> None:
    # write -> read reconstructs the run identity + requested topic set (the record-START facts).
    path = sidecar_inputs_path(tmp_path / f"patrol_alpha_{_TS}")
    write_sidecar_inputs(path, _sample_run(), _NAMED_TOPICS + _REGEXES)

    run, recorded_topics = read_sidecar_inputs(path)

    assert run == _sample_run()  # dataclass equality: every identity field survives the round-trip
    assert recorded_topics == _NAMED_TOPICS + _REGEXES


@pytest.fixture
def staged_bag_dir(tmp_path):
    """A finalized bag dir (metadata.yaml present) with its record-start staging file beside it."""
    bag_dir = tmp_path / f"patrol_alpha_{_TS}"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    write_sidecar_inputs(sidecar_inputs_path(bag_dir), _sample_run(), _NAMED_TOPICS + _REGEXES)
    return bag_dir


def test_finalize_writes_sidecar_from_staging(staged_bag_dir) -> None:
    sidecar_path = finalize_sidecar_from_staging(staged_bag_dir)

    assert sidecar_path is not None
    assert sidecar_path == staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json")
    loaded = json.loads(sidecar_path.read_text())
    assert loaded["mission_id"] == "alpha"
    assert loaded["bag_uri"] == f"patrol_alpha_{_TS}"
    assert loaded["started_utc"] == "2026-06-26T14:05:09+00:00"  # the STAGED start, not "now"
    assert loaded["recorded_topics"] == _NAMED_TOPICS + _REGEXES
    assert loaded["mission_config_ref"] == "/abs/patrol_mission.yaml"


def test_finalize_removes_the_staging_crumb(staged_bag_dir) -> None:
    finalize_sidecar_from_staging(staged_bag_dir)
    assert not sidecar_inputs_path(staged_bag_dir).exists()


def test_finalize_is_idempotent(staged_bag_dir) -> None:
    # First call writes it; a second (e.g. runner after the OnProcessExit handler already wrote it)
    # must be a no-op that neither errors nor rewrites — returns None.
    first = finalize_sidecar_from_staging(staged_bag_dir)
    assert first is not None
    assert finalize_sidecar_from_staging(staged_bag_dir) is None


def test_finalize_no_ops_when_sidecar_already_present(staged_bag_dir) -> None:
    # The happy SIGINT path already wrote the sidecar; the runner must not clobber it, but should
    # still clear the staging crumb.
    sidecar_path = staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json")
    write_sidecar(sidecar_path, _sample_sidecar())

    assert finalize_sidecar_from_staging(staged_bag_dir) is None
    assert not sidecar_inputs_path(staged_bag_dir).exists()  # crumb cleared


@pytest.mark.parametrize(
    "missing",
    [
        "metadata.yaml",  # no finalized bag -> a failed recording must not get a blessing sidecar
        "staging",  # no staging crumb (recorder ran without our launch write) -> nothing to finalize
    ],
)
def test_finalize_no_ops_and_writes_nothing_when_a_precondition_is_missing(
    staged_bag_dir, missing: str
) -> None:
    if missing == "metadata.yaml":
        (staged_bag_dir / "metadata.yaml").unlink()
    else:
        sidecar_inputs_path(staged_bag_dir).unlink()

    assert finalize_sidecar_from_staging(staged_bag_dir) is None
    assert not staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json").exists()
