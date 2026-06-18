"""Unit tests for mission YAML parse/validate (design §4.2.5, MC-3, INF-M3).

Layer-A: ROS-free, deterministic. Fail-loud config is the contract — a bad
config must raise at load time so a bad mission never flies.

M1 scope was top-level params + defaults + inline waypoints + fail-loud. M4 (T2.2) adds
`checkpoint_id` resolution against 03's `checkpoints.yaml` (path parameterized, OQ-2) — a
referenced id resolves to its ENU position, and an unresolvable id (or a missing file when
one is referenced) fails loud.
"""

from pathlib import Path

import pytest
from patrol_mission.config import (
    AbortConfig,
    Completion,
    MissionConfig,
    load_mission_config,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
MISSION_BASIC = REPO_ROOT / "ros2_ws/src/patrol_bringup/config/mission_basic.yaml"
PATROL_MISSION = REPO_ROOT / "ros2_ws/src/patrol_bringup/config/patrol_mission.yaml"
CHECKPOINTS = REPO_ROOT / "sim/config/checkpoints.yaml"

# Shared YAML building blocks — kept here once so the tests vary only the part under test
# (and don't repeat the scaffold, which trips duplication detectors).
_HEAD = "takeoff_alt_m: 5\nhover_time_s: 10\n"
_HOME_ENU = "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\n"
_NO_WAYPOINTS = "waypoints: []\n"
# Two checkpoints in the confirmed Appendix C.1 schema (cp_north@ENU(10,0,2), cp_east@ENU(0,10,2)).
_CHECKPOINTS = (
    "- checkpoint_id: cp_north\n  position: {x: 10, y: 0, z: 2}\n  tag_family: tag36h11\n  tag_id: 0\n"
    "- checkpoint_id: cp_east\n  position: {x: 0, y: 10, z: 2}\n  tag_family: tag36h11\n  tag_id: 1\n"
)


def _write(tmp_path: Path, text: str) -> str:
    p = tmp_path / "mission.yaml"
    p.write_text(text)
    return str(p)


def _write_checkpoints(tmp_path: Path, text: str = _CHECKPOINTS) -> str:
    p = tmp_path / "checkpoints.yaml"
    p.write_text(text)
    return str(p)


def _wp_checkpoint(cid: str, dwell_s: float = 3.0) -> str:
    return f"waypoints:\n  - checkpoint_id: {cid}\n    dwell_s: {dwell_s}\n"


# TS-9: the shipped mission_basic.yaml (waypoints: []) loads and is well-typed.
def test_shipped_mission_basic_loads():
    cfg = load_mission_config(str(MISSION_BASIC))
    assert isinstance(cfg, MissionConfig)
    assert cfg.takeoff_alt_m == 5.0
    assert cfg.hover_time_s == 10.0
    assert cfg.waypoints == ()
    assert cfg.home_frame == "enu"
    assert cfg.home_position == (0.0, 0.0, 2.0)  # M4: home at 2 m ENU so RTH settles before land


# TS-9: completion/abort defaults (OQ-4 / OQ-6) apply when omitted.
def test_defaults_applied_when_sections_omitted(tmp_path):
    cfg = load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _NO_WAYPOINTS))
    assert cfg.completion == Completion(tolerance_m=0.5, hold_time_s=2.0)
    assert cfg.abort == AbortConfig(low_battery_threshold=0.20)


# TS-9: explicit completion/abort values override the defaults.
def test_explicit_overrides(tmp_path):
    overrides = (
        "completion: {tolerance_m: 0.25, hold_time_s: 3.0}\nabort: {low_battery_threshold: 0.35}\n"
    )
    cfg = load_mission_config(_write(tmp_path, _HEAD + overrides + _HOME_ENU + _NO_WAYPOINTS))
    assert cfg.completion.tolerance_m == 0.25
    assert cfg.abort.low_battery_threshold == 0.35


# TS-9: an inline waypoint parses with its frame and dwell.
def test_inline_waypoint(tmp_path):
    waypoints = "waypoints:\n  - position: {x: -10, y: 0, z: 2}\n    frame: enu\n    dwell_s: 3.0\n"
    cfg = load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + waypoints))
    assert len(cfg.waypoints) == 1
    wp = cfg.waypoints[0]
    assert wp.position == (-10.0, 0.0, 2.0)
    assert wp.frame == "enu"
    assert wp.dwell_s == 3.0
    assert wp.checkpoint_id is None


# TS-9: fail-loud paths — missing field, unknown frame (waypoint + home). Each must raise at
# load time. (The M1 checkpoint_id-deferral case is gone — checkpoint_id now resolves; see
# test_unresolvable_checkpoint_id_raises for its M4 fail-loud path.)
@pytest.mark.parametrize(
    ("body", "match"),
    [
        ("hover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS, "takeoff_alt_m"),
        (
            _HEAD
            + _HOME_ENU
            + "waypoints:\n  - position: {x: 1, y: 1, z: 1}\n    frame: lla\n    dwell_s: 1.0\n",
            "frame",
        ),
        (_HEAD + "home: {position: {x: 0, y: 0, z: 2}, frame: lla}\n" + _NO_WAYPOINTS, "frame"),
    ],
    ids=[
        "missing_required_field",
        "unknown_waypoint_frame",
        "unknown_home_frame",
    ],
)
def test_fail_loud(tmp_path, body, match):
    with pytest.raises((KeyError, ValueError, NotImplementedError), match=match):
        load_mission_config(_write(tmp_path, body))


# TS-C1: a checkpoint_id waypoint resolves against checkpoints.yaml to its ENU position + id.
def test_checkpoint_id_resolves(tmp_path):
    cps = _write_checkpoints(tmp_path)
    cfg = load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)
    assert len(cfg.waypoints) == 1
    wp = cfg.waypoints[0]
    assert wp.checkpoint_id == "cp_north"
    assert wp.position == (10.0, 0.0, 2.0)
    assert wp.frame == "enu"
    assert wp.dwell_s == 3.0


# TS-C2: an unresolvable checkpoint_id fails loud (names the id).
def test_unresolvable_checkpoint_id_raises(tmp_path):
    cps = _write_checkpoints(tmp_path)
    with pytest.raises(ValueError, match="cp_missing"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_missing")), cps)


# TS-C2b: a checkpoints file entry missing its position fails loud (contracted ValueError).
def test_malformed_checkpoint_entry_raises(tmp_path):
    cps = _write_checkpoints(tmp_path, "- checkpoint_id: cp_north\n  tag_id: 0\n")
    with pytest.raises(ValueError, match="position"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)


# TS-C2d: a checkpoints entry missing checkpoint_id fails loud with the contracted ValueError
# (not a bare KeyError) — symmetric with the missing-position case.
def test_checkpoint_entry_missing_id_raises(tmp_path):
    cps = _write_checkpoints(tmp_path, "- position: {x: 1, y: 2, z: 3}\n  tag_id: 0\n")
    with pytest.raises(ValueError, match="checkpoint_id"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)


# TS-C2c: a checkpoints file that is not a list (e.g. a mapping) fails loud when referenced.
def test_non_list_checkpoints_file_raises(tmp_path):
    cps = _write_checkpoints(tmp_path, "cp_north: {x: 1, y: 2, z: 3}\n")  # mapping, not a list
    with pytest.raises(ValueError, match="list of checkpoints"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)


# TS-C3: a route mixing checkpoint_id and inline waypoints resolves all of them in order.
def test_mixed_checkpoint_and_inline(tmp_path):
    cps = _write_checkpoints(tmp_path)
    wps = (
        "waypoints:\n"
        "  - checkpoint_id: cp_east\n    dwell_s: 2.0\n"
        "  - position: {x: -10, y: 0, z: 2}\n    frame: enu\n    dwell_s: 1.5\n"
    )
    cfg = load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + wps), cps)
    assert [w.checkpoint_id for w in cfg.waypoints] == ["cp_east", None]
    assert cfg.waypoints[0].position == (0.0, 10.0, 2.0)
    assert cfg.waypoints[1].position == (-10.0, 0.0, 2.0)


# TS-C4: the checkpoints path is a parameter (OQ-2) — a non-default location is honored.
def test_checkpoints_path_override(tmp_path):
    p = tmp_path / "custom_checkpoints.yaml"
    p.write_text(_CHECKPOINTS)
    cfg = load_mission_config(
        _write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), str(p)
    )
    assert cfg.waypoints[0].position == (10.0, 0.0, 2.0)


# TS-C5: a checkpoint_id reference with no checkpoints file fails loud (not silently empty).
def test_missing_checkpoints_file_when_referenced(tmp_path):
    missing = str(tmp_path / "nope.yaml")
    with pytest.raises((ValueError, FileNotFoundError), match="checkpoint"):
        load_mission_config(
            _write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), missing
        )


# TS-C5c (Hermes Medium): a checkpoint_id reference with NO checkpoints path supplied fails loud
# with guidance — the file is 03-owned and has no CWD-relative default to silently fall back on.
def test_checkpoint_reference_without_path_fails_loud(tmp_path):
    with pytest.raises(ValueError, match="checkpoint_id"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")))


# TS-C5b: a missing checkpoints file is harmless when NO waypoint references a checkpoint_id
# (the basic mission must still load even if 03's file is absent).
def test_missing_checkpoints_file_ignored_when_unreferenced(tmp_path):
    missing = str(tmp_path / "nope.yaml")
    cfg = load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _NO_WAYPOINTS), missing)
    assert cfg.waypoints == ()


# TS-C6: the shipped patrol_mission.yaml resolves against the shipped interim checkpoints.yaml.
def test_shipped_patrol_mission_loads():
    cfg = load_mission_config(str(PATROL_MISSION), str(CHECKPOINTS))
    assert len(cfg.waypoints) == 4
    assert [w.checkpoint_id for w in cfg.waypoints] == ["cp_north", "cp_east", None, None]
    assert cfg.waypoints[0].frame == "enu"


# Shared completion/abort/waypoint scaffolds so the range cases vary only the field under test.
def _completion(tolerance_m: float, hold_time_s: float) -> str:
    return f"completion: {{tolerance_m: {tolerance_m}, hold_time_s: {hold_time_s}}}\n"


def _waypoint(dwell_s: float) -> str:
    return (
        f"waypoints:\n  - position: {{x: 1, y: 1, z: 1}}\n    frame: enu\n    dwell_s: {dwell_s}\n"
    )


# TS-9 (Hermes Medium #2): a numerically well-typed but semantically impossible value fails loud at
# load time — a config that would immediately land, never complete, or carry an out-of-range future
# abort threshold must never reach the node.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        ("takeoff_alt_m: 0\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS, "takeoff_alt_m"),
        ("takeoff_alt_m: -5\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS, "takeoff_alt_m"),
        ("takeoff_alt_m: 5\nhover_time_s: -1\n" + _HOME_ENU + _NO_WAYPOINTS, "hover_time_s"),
        (_HEAD + _completion(0, 2.0) + _HOME_ENU + _NO_WAYPOINTS, "tolerance_m"),
        (_HEAD + _completion(0.5, -1) + _HOME_ENU + _NO_WAYPOINTS, "hold_time_s"),
        (_HEAD + "abort: {low_battery_threshold: 1.5}\n" + _HOME_ENU + _NO_WAYPOINTS, "battery"),
        (_HEAD + "abort: {low_battery_threshold: -0.1}\n" + _HOME_ENU + _NO_WAYPOINTS, "battery"),
        (_HEAD + _HOME_ENU + _waypoint(-1), "dwell_s"),
    ],
    ids=[
        "takeoff_alt_zero",
        "takeoff_alt_negative",
        "hover_time_negative",
        "tolerance_zero",
        "hold_time_negative",
        "battery_threshold_above_one",
        "battery_threshold_below_zero",
        "dwell_negative",
    ],
)
def test_fail_loud_out_of_range(tmp_path, body, match):
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, body))


# Review #3: an optional section (completion / abort) that is present-but-null, not a mapping, or
# carries an unknown/misspelled key must fail loud with a ValueError (field context), not leak the
# bare TypeError that `Completion(**...)` / `AbortConfig(**...)` would otherwise raise.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        (_HEAD + "completion:\n" + _HOME_ENU + _NO_WAYPOINTS, "completion.*null"),
        (_HEAD + "abort:\n" + _HOME_ENU + _NO_WAYPOINTS, "abort.*null"),
        (_HEAD + "completion: 0.5\n" + _HOME_ENU + _NO_WAYPOINTS, "completion.*mapping"),
        (
            _HEAD + "completion: {tolerance_m: 0.5, bogus: 1}\n" + _HOME_ENU + _NO_WAYPOINTS,
            "completion",
        ),
        (_HEAD + "abort: {nope: 1}\n" + _HOME_ENU + _NO_WAYPOINTS, "abort"),
    ],
    ids=[
        "completion_null",
        "abort_null",
        "completion_not_a_mapping",
        "completion_unknown_key",
        "abort_unknown_key",
    ],
)
def test_section_fail_loud(tmp_path, body, match):
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, body))


# Review #3: an explicit empty mapping (`completion: {}`) is NOT an error — it means "use defaults",
# the same as omitting the section entirely.
def test_empty_section_mapping_uses_defaults(tmp_path):
    cfg = load_mission_config(
        _write(tmp_path, _HEAD + "completion: {}\n" + _HOME_ENU + _NO_WAYPOINTS)
    )
    assert cfg.completion == Completion(tolerance_m=0.5, hold_time_s=2.0)


# Boundary values that ARE valid must load: zero hover/hold (no wait) and the [0, 1] battery
# threshold endpoints are accepted — the guard rejects out-of-range, not the legal boundary.
@pytest.mark.parametrize(
    "body",
    [
        "takeoff_alt_m: 5\nhover_time_s: 0\n" + _HOME_ENU + _NO_WAYPOINTS,
        _HEAD + _completion(0.5, 0) + _HOME_ENU + _NO_WAYPOINTS,
        _HEAD + "abort: {low_battery_threshold: 0.0}\n" + _HOME_ENU + _NO_WAYPOINTS,
        _HEAD + "abort: {low_battery_threshold: 1.0}\n" + _HOME_ENU + _NO_WAYPOINTS,
        _HEAD + _HOME_ENU + _waypoint(0),
    ],
    ids=["hover_zero", "hold_zero", "battery_zero", "battery_one", "dwell_zero"],
)
def test_valid_boundaries_load(tmp_path, body):
    assert isinstance(load_mission_config(_write(tmp_path, body)), MissionConfig)
