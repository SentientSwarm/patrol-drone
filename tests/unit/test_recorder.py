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
import os
from dataclasses import asdict
from datetime import UTC, datetime
from pathlib import Path

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


def test_write_sidecar_bytes_are_stable_pretty_json(tmp_path) -> None:
    # Byte-parity guard (F-01): the atomic write must leave the FINAL file byte-identical to the
    # pre-atomic form (indent=2, sort_keys=True, trailing newline). The OnProcessExit path and the
    # staging-finalize path both go through write_sidecar, so this pins the serialized bytes so a
    # future refactor can't silently drift the format the two paths must agree on byte-for-byte.

    sidecar = _sample_sidecar()
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"

    write_sidecar(path, sidecar)

    assert path.read_text() == json.dumps(asdict(sidecar), indent=2, sort_keys=True) + "\n"


# --- atomic publish (F-01): the uploader keys "bag complete" on <bag>.meta.json merely existing, so
# an observer must never see a truncated/partial file. The write is synchronous + single-threaded, so
# these tests assert the invariant the atomic publish guarantees — the bytes that LAND are always a
# whole sidecar, never a prefix — by wrapping os.replace; they are not a live-concurrency race test.


def test_write_sidecar_only_ever_lands_whole_json(tmp_path, monkeypatch) -> None:
    # Wrap os.replace to assert that at the instant of every rename, the temp source is already a
    # COMPLETE, parseable sidecar (never a truncated prefix). Writing twice to the same path exercises
    # the overwrite case too. This proves the temp is finished before it becomes visible at `path`.
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    real_replace = os.replace
    landed: list[dict] = []

    def _checked_replace(src, dst):
        # The source temp must be a whole sidecar before it is atomically swapped into place.
        landed.append(json.loads(Path(src).read_text()))
        return real_replace(src, dst)

    monkeypatch.setattr("patrol_logging.recorder.os.replace", _checked_replace)

    write_sidecar(path, _sample_sidecar())
    write_sidecar(path, build_sidecar(_sample_run(), _ENDED, ["/only/one"]))

    assert len(landed) == 2  # both writes went through the atomic rename
    assert landed[0]["recorded_topics"] == _NAMED_TOPICS + _REGEXES
    assert landed[1]["recorded_topics"] == ["/only/one"]
    assert json.loads(path.read_text())["recorded_topics"] == ["/only/one"]  # last write wins


def test_write_sidecar_fsyncs_parent_dir_after_replace(tmp_path, monkeypatch) -> None:
    # Durability upgrade (F-03): os.replace makes the swap atomic, but the new directory ENTRY is only
    # persisted once the PARENT dir is fsynced. Assert the parent dir is opened and fsynced *after* the
    # rename — a dir fd distinct from the file fd — so a crash right after the replace can't lose the
    # <bag>.meta.json entry (which would make the uploader skip the finalized bag forever).
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    events: list[str] = []
    real_replace = os.replace
    real_fsync = os.fsync

    def _tracked_replace(src, dst):
        events.append("replace")
        return real_replace(src, dst)

    def _tracked_fsync(fd) -> None:
        # A dir fd opened on tmp_path is the parent-dir sync; anything else is the file-content sync.
        events.append(
            "fsync_dir" if os.path.samestat(os.fstat(fd), os.stat(tmp_path)) else "fsync_file"
        )
        return real_fsync(fd)

    monkeypatch.setattr("patrol_logging.recorder.os.replace", _tracked_replace)
    monkeypatch.setattr("patrol_logging.recorder.os.fsync", _tracked_fsync)

    write_sidecar(path, _sample_sidecar())

    assert "fsync_dir" in events, "parent directory must be fsynced for the rename to be durable"
    # The parent-dir fsync must come AFTER the atomic rename (a pre-rename dir sync would be pointless).
    assert events.index("fsync_dir") > events.index("replace")
    assert path.is_file()


def test_write_sidecar_survives_dir_fsync_unsupported(tmp_path, monkeypatch) -> None:
    # The durability upgrade must never become a NEW failure mode: on a platform/filesystem where the
    # directory fd can't be opened/fsynced (Windows has no dir fd; some filesystems reject it), the
    # write still succeeds and the sidecar still lands. Simulate by making the dir open raise OSError.
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    real_open = os.open

    def _open_refusing_dirs(target, flags, *args, **kwargs):
        if os.path.isdir(target):
            raise OSError("simulated: directory fd unsupported")
        return real_open(target, flags, *args, **kwargs)

    monkeypatch.setattr("patrol_logging.recorder.os.open", _open_refusing_dirs)

    write_sidecar(path, _sample_sidecar())  # must NOT raise

    assert json.loads(path.read_text())["mission_id"] == "alpha"  # write still landed intact


def test_write_sidecar_cleans_up_temp_on_success(tmp_path) -> None:
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    write_sidecar(path, _sample_sidecar())
    # The temp sibling was renamed away, not left behind, so no *.tmp crumb masquerades as an artifact.
    assert list(tmp_path.glob("*.tmp")) == []
    assert path.is_file()


def test_write_sidecar_removes_temp_and_leaves_dest_untouched_on_failure(
    tmp_path, monkeypatch
) -> None:
    # Force the inner write to fail after the temp is opened; write_sidecar must re-raise, remove the
    # temp crumb, and leave any pre-existing destination untouched (never a torn final file).
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    write_sidecar(path, _sample_sidecar())  # a valid prior sidecar that must survive the failure
    original = path.read_text()

    def _boom(_fd) -> None:
        raise OSError("simulated fsync failure")

    monkeypatch.setattr("patrol_logging.recorder.os.fsync", _boom)

    with pytest.raises(OSError, match="simulated fsync failure"):
        write_sidecar(path, build_sidecar(_sample_run(), _ENDED, ["/never/written"]))

    assert list(tmp_path.glob("*.tmp")) == []  # crumb cleaned up
    assert path.read_text() == original  # destination untouched (the atomic swap never happened)


@pytest.mark.parametrize(
    "plant", ["regular_file", "symlink"], ids=["planted-file", "planted-symlink"]
)
def test_write_sidecar_refuses_a_preplanted_temp_path(tmp_path, monkeypatch, plant: str) -> None:
    # Interleaving guard (F-03, guide §1.B): the temp is created O_CREAT|O_EXCL|O_NOFOLLOW under an
    # unpredictable secrets.token_hex name, so a pre-planted file OR symlink at the exact temp path
    # must make the write FAIL CLOSED — never written through, never adopted, and never deleted (the
    # cleanup only unlinks temps this call created). Pin the token so the temp path is constructible.
    monkeypatch.setattr("patrol_logging.recorder.secrets.token_hex", lambda _n: "feedface")
    path = tmp_path / f"patrol_alpha_{_TS}.meta.json"
    planted = tmp_path / f"{path.name}.feedface.tmp"
    target = tmp_path / "attacker_target"
    target.write_text("untouched")
    if plant == "regular_file":
        planted.write_text("squatter")
    else:
        planted.symlink_to(target)

    with pytest.raises(FileExistsError):  # O_EXCL: an occupied temp path fails loudly
        write_sidecar(path, _sample_sidecar())

    assert target.read_text() == "untouched"  # nothing was written through a planted symlink
    assert not path.exists()  # the real sidecar never landed
    assert os.path.lexists(planted)  # the foreign plant is evidence — not adopted, not removed


def test_write_sidecar_inputs_cleans_up_temp_on_success(tmp_path) -> None:
    # The staging file goes through the same atomic helper (consistency, F-01) — lock its cleanup too.
    path = sidecar_inputs_path(tmp_path / f"patrol_alpha_{_TS}")
    write_sidecar_inputs(path, _sample_run(), _NAMED_TOPICS + _REGEXES)
    assert list(tmp_path.glob("*.tmp")) == []
    assert path.is_file()


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
    """A finalized bag dir with its record-start staging file beside it.

    "Finalized" is metadata.yaml AND a real ``.mcap`` payload (``recorder._is_finalized_bag_dir``,
    the record-side twin of ``_shared.bag_layout.is_valid_bag_dir``) — a metadata-only dir is NOT a
    bag, so the payload is part of the fixture, not decoration (F-02).
    """
    bag_dir = tmp_path / f"patrol_alpha_{_TS}"
    bag_dir.mkdir()
    (bag_dir / "metadata.yaml").write_text("rosbag2_bagfile_information:\n")
    (bag_dir / f"{bag_dir.name}_0.mcap").write_bytes(b"\x89MCAP0\r\n")
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


# F-02 (Mira Medium, review 4754192970): a metadata-only bag — recorder killed after metadata.yaml
# was written but before the MCAP flushed — is NOT a finalized bag. It used to get a sidecar and lose
# its staging crumb here while the uploader's stricter is_valid_bag_dir skipped it forever: the
# producer reported success on a bag that was silently stranded. Crumb survival is the assertion that
# matters — "returns None" alone would still pass if the crumb had been consumed, and the crumb is
# what makes the run recoverable.
def test_finalize_no_ops_on_a_metadata_only_bag_and_keeps_the_staging_crumb(staged_bag_dir) -> None:
    (staged_bag_dir / f"{staged_bag_dir.name}_0.mcap").unlink()  # marker written, payload never was

    assert finalize_sidecar_from_staging(staged_bag_dir) is None
    assert not staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json").exists()
    assert sidecar_inputs_path(staged_bag_dir).exists()  # recoverable: the crumb survives


def _replace_with_symlink(path: Path) -> None:
    """Swap the real file at ``path`` for a symlink pointing at a same-content decoy (F-02)."""
    decoy = path.with_name(path.name + ".decoy")
    if path.exists():
        path.rename(decoy)
    else:
        decoy.write_text("planted\n")
    path.symlink_to(decoy)


@pytest.mark.parametrize("symlinked", ["metadata.yaml", "staging"])
def test_finalize_no_ops_on_a_symlinked_precondition(staged_bag_dir, symlinked: str) -> None:
    # F-02: a symlinked metadata.yaml or staging file is NOT a finalizable artifact (parity with the
    # uploader's _shared.bag_layout.is_regular_file), so finalize no-ops (None) and writes no REAL
    # <bag>.meta.json —
    # letting the runner's outcome gate fail loudly instead of blessing a symlink-planted bag.
    if symlinked == "metadata.yaml":
        _replace_with_symlink(staged_bag_dir / "metadata.yaml")
    else:
        _replace_with_symlink(sidecar_inputs_path(staged_bag_dir))

    real_sidecar = staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json")
    assert finalize_sidecar_from_staging(staged_bag_dir) is None
    assert not real_sidecar.exists()  # no sidecar written for a symlink-tampered bag


def test_finalize_overwrites_a_symlinked_sidecar_without_following_it(staged_bag_dir) -> None:
    # F-02: a pre-planted <bag>.meta.json SYMLINK must not count as an already-present sidecar (that
    # would let a planted link suppress finalize). _is_regular_file rejects it, so finalize proceeds
    # and writes a REAL sidecar — and the atomic os.replace REPLACES the symlink rather than following
    # it, so the decoy target is never written through.
    real_sidecar = staged_bag_dir.with_name(staged_bag_dir.name + ".meta.json")
    decoy = staged_bag_dir.with_name("decoy_target")
    decoy.write_text("attacker-owned\n")
    real_sidecar.symlink_to(decoy)

    written = finalize_sidecar_from_staging(staged_bag_dir)

    assert written == real_sidecar
    assert real_sidecar.is_file()  # a real file now lives at the sidecar path...
    assert not real_sidecar.is_symlink()  # ...the planted symlink was replaced, not followed
    assert decoy.read_text() == "attacker-owned\n"  # the symlink's target was NOT written through
    assert json.loads(real_sidecar.read_text())["mission_id"] == "alpha"  # a genuine sidecar
