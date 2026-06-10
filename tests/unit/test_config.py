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
    cfg = load_mission_config(
        _write(
            tmp_path,
            "takeoff_alt_m: 3.0\nhover_time_s: 4.0\n"
            "home: {position: {x: 0, y: 0, z: 1}, frame: ned}\nwaypoints: []\n",
        )
    )
    assert cfg.completion == Completion(tolerance_m=0.5, hold_time_s=2.0)
    assert cfg.abort == AbortConfig(low_battery_threshold=0.20)


# TS-9: explicit completion/abort values override the defaults.
def test_explicit_overrides(tmp_path):
    cfg = load_mission_config(
        _write(
            tmp_path,
            "takeoff_alt_m: 5\nhover_time_s: 10\n"
            "completion: {tolerance_m: 0.25, hold_time_s: 3.0}\n"
            "abort: {low_battery_threshold: 0.35}\n"
            "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\nwaypoints: []\n",
        )
    )
    assert cfg.completion.tolerance_m == 0.25
    assert cfg.abort.low_battery_threshold == 0.35


# TS-9: an inline waypoint parses with its frame and dwell.
def test_inline_waypoint(tmp_path):
    cfg = load_mission_config(
        _write(
            tmp_path,
            "takeoff_alt_m: 5\nhover_time_s: 10\n"
            "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\n"
            "waypoints:\n  - position: {x: -10, y: 0, z: 2}\n    frame: enu\n    dwell_s: 3.0\n",
        )
    )
    assert len(cfg.waypoints) == 1
    wp = cfg.waypoints[0]
    assert wp.position_enu == (-10.0, 0.0, 2.0)
    assert wp.frame == "enu"
    assert wp.dwell_s == 3.0
    assert wp.checkpoint_id is None


# TS-9: fail loud on a missing required top-level field.
def test_missing_required_field_raises(tmp_path):
    with pytest.raises((KeyError, ValueError)):
        load_mission_config(
            _write(
                tmp_path,
                "hover_time_s: 10\nhome: {position: {x: 0, y: 0, z: 2}, frame: enu}\nwaypoints: []\n",
            )
        )


# TS-9: fail loud on an unknown waypoint frame.
def test_unknown_waypoint_frame_raises(tmp_path):
    with pytest.raises(ValueError, match="frame"):
        load_mission_config(
            _write(
                tmp_path,
                "takeoff_alt_m: 5\nhover_time_s: 10\n"
                "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\n"
                "waypoints:\n  - position: {x: 1, y: 1, z: 1}\n    frame: lla\n    dwell_s: 1.0\n",
            )
        )


# TS-9: a checkpoint_id waypoint fails loud in M1 (resolution lands in M4).
def test_checkpoint_id_waypoint_deferred_m4(tmp_path):
    with pytest.raises((ValueError, NotImplementedError), match="checkpoint_id"):
        load_mission_config(
            _write(
                tmp_path,
                "takeoff_alt_m: 5\nhover_time_s: 10\n"
                "home: {position: {x: 0, y: 0, z: 2}, frame: enu}\n"
                "waypoints:\n  - checkpoint_id: cp_north\n    dwell_s: 3.0\n",
            )
        )


# TS-9: fail loud on an unknown home frame.
def test_unknown_home_frame_raises(tmp_path):
    with pytest.raises(ValueError, match="frame"):
        load_mission_config(
            _write(
                tmp_path,
                "takeoff_alt_m: 5\nhover_time_s: 10\n"
                "home: {position: {x: 0, y: 0, z: 2}, frame: lla}\nwaypoints: []\n",
            )
        )
