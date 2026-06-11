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
    assert cfg.home_position == (0.0, 0.0, 2.0)


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
    assert wp.position_enu == (-10.0, 0.0, 2.0)
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
