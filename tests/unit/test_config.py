"""Unit tests for mission YAML parse/validate (design §4.2.5, MC-3, INF-M3).

Layer-A: ROS-free, deterministic. Fail-loud config is the contract — a bad
config must raise at load time so a bad mission never flies.

M1 scope: top-level params + defaults + inline waypoints + fail-loud. The
`checkpoint_id` resolution path against 03's checkpoints.yaml lands in M4; M1
guards it with a loud, testable error.
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

# Shared YAML building blocks — kept here once so the tests vary only the part under test
# (and don't repeat the scaffold, which trips duplication detectors).
_HEAD = "takeoff_alt_m: 5\nhover_time_s: 10\n"
_HOME_ENU = "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\n"
_NO_WAYPOINTS = "waypoints: []\n"


def _write(tmp_path: Path, text: str) -> str:
    p = tmp_path / "mission.yaml"
    p.write_text(text)
    return str(p)


# TS-9: the shipped mission_basic.yaml (waypoints: []) loads and is well-typed.
def test_shipped_mission_basic_loads():
    cfg = load_mission_config(str(MISSION_BASIC))
    assert isinstance(cfg, MissionConfig)
    assert cfg.takeoff_alt_m == 5.0
    assert cfg.hover_time_s == 10.0
    assert cfg.waypoints == ()
    assert cfg.home_frame == "enu"
    assert cfg.home_position == (0.0, 0.0, 0.0)


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


# TS-9: fail-loud paths — missing field, unknown frame (waypoint + home), and the M1
# checkpoint_id guard (resolution lands in M4). Each must raise at load time.
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
        (
            _HEAD + _HOME_ENU + "waypoints:\n  - checkpoint_id: cp_north\n    dwell_s: 3.0\n",
            "checkpoint_id",
        ),
        (_HEAD + "home: {position: {x: 0, y: 0, z: 2}, frame: lla}\n" + _NO_WAYPOINTS, "frame"),
    ],
    ids=[
        "missing_required_field",
        "unknown_waypoint_frame",
        "checkpoint_id_deferred_m4",
        "unknown_home_frame",
    ],
)
def test_fail_loud(tmp_path, body, match):
    with pytest.raises((KeyError, ValueError, NotImplementedError), match=match):
        load_mission_config(_write(tmp_path, body))


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
