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


# Hermes Medium: a malformed top-level shape — a null/scalar mission document, a null/scalar `home`
# section, a null/scalar `waypoints` list, or a non-mapping waypoint entry — must fail loud with
# field/index context (the contracted ValueError), not leak the bare TypeError that the downstream
# `key in node` membership / `node[...]` subscript would otherwise raise.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        ("", "mission config must be a mapping"),
        ("- a\n- b\n", "mission config must be a mapping"),
        (_HEAD + "home:\n" + _NO_WAYPOINTS, "home.*mapping"),
        (_HEAD + _HOME_ENU + "waypoints:\n", "waypoints.*list"),
        (_HEAD + _HOME_ENU + "waypoints: 5\n", "waypoints.*list"),
        (_HEAD + _HOME_ENU + "waypoints:\n  - 123\n", r"waypoint\[0\].*mapping"),
    ],
    ids=[
        "empty_document",
        "non_mapping_document",
        "null_home",
        "null_waypoints",
        "scalar_waypoints",
        "non_mapping_waypoint_entry",
    ],
)
def test_malformed_top_level_shape_fail_loud(tmp_path, body, match):
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, body))


# Hermes (review follow-up): a malformed `position` value — a null/scalar node, or one missing or
# non-numeric x/y/z — must fail loud with field context (the contracted ValueError), not leak the
# bare TypeError/KeyError the subscript or float(...) cast would otherwise raise. Exercised through
# both `home` and an inline waypoint, which share the single `_point` boundary.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        (_HEAD + "home: {position: null, frame: enu}\n" + _NO_WAYPOINTS, "home position.*mapping"),
        (_HEAD + "home: {position: 5, frame: enu}\n" + _NO_WAYPOINTS, "home position.*mapping"),
        (
            _HEAD + "home: {position: {x: 0, y: 0}, frame: enu}\n" + _NO_WAYPOINTS,
            r"home position missing required axis 'z'",
        ),
        (
            _HEAD + "home: {position: {x: abc, y: 0, z: 0}, frame: enu}\n" + _NO_WAYPOINTS,
            r"home position axis 'x' must be a number",
        ),
        (
            _HEAD
            + _HOME_ENU
            + "waypoints:\n  - position: null\n    frame: enu\n    dwell_s: 1.0\n",
            r"waypoint\[0\] position.*mapping",
        ),
        (
            _HEAD
            + _HOME_ENU
            + "waypoints:\n  - position: {x: 1, y: 2}\n    frame: enu\n    dwell_s: 1.0\n",
            r"waypoint\[0\] position missing required axis 'z'",
        ),
    ],
    ids=[
        "home_null_position",
        "home_scalar_position",
        "home_missing_axis",
        "home_non_numeric_axis",
        "waypoint_null_position",
        "waypoint_missing_axis",
    ],
)
def test_malformed_position_fail_loud(tmp_path, body, match):
    with pytest.raises(ValueError, match=match):
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


# TS-C2f (Hermes Medium): a checkpoints entry that is not a mapping (a bare scalar `- 123`) fails
# loud with field context rather than leaking a bare TypeError from the `checkpoint_id in entry`
# membership test — symmetric with the non-list-file and missing-field cases above.
def test_non_mapping_checkpoint_entry_raises(tmp_path):
    cps = _write_checkpoints(tmp_path, "- 123\n")
    with pytest.raises(ValueError, match="checkpoints entry must be a mapping"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)


# TS-C2e: a duplicate checkpoint_id fails loud rather than silently overwriting earlier coordinates
# (a plain dict() fold would keep only the last). Names the offending id (Hermes Medium).
def test_duplicate_checkpoint_id_raises(tmp_path):
    cps = _write_checkpoints(
        tmp_path,
        "- checkpoint_id: cp_north\n  position: {x: 10, y: 0, z: 2}\n"
        "- checkpoint_id: cp_north\n  position: {x: 99, y: 9, z: 9}\n",  # same id, different coords
    )
    with pytest.raises(ValueError, match="duplicate checkpoint_id 'cp_north'"):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")), cps)


# TS-C2g (Hermes Medium): a non-string checkpoint_id is rejected on BOTH sides of the shared
# namespace — in the checkpoints map and in a waypoint reference. YAML parses `checkpoint_id: 1` as
# int, which would key the map (and Waypoint.checkpoint_id, declared str) with the wrong type and
# silently break the string-keyed 02/03 contract / risk an int-vs-str key mismatch. Fail loud,
# with field context naming the offending side.
@pytest.mark.parametrize(
    ("checkpoints_text", "waypoints_text", "match"),
    [
        pytest.param(
            "- checkpoint_id: 1\n  position: {x: 1, y: 2, z: 3}\n",
            _wp_checkpoint("cp_north"),  # valid ref, never reached — the map fails to load first
            "checkpoints entry checkpoint_id must be a string",
            id="in_map",
        ),
        pytest.param(
            _CHECKPOINTS,  # a valid map; the int reference itself is the failure
            "waypoints:\n  - checkpoint_id: 1\n    dwell_s: 3.0\n",
            r"waypoint\[0\] checkpoint_id must be a string",
            id="in_reference",
        ),
    ],
)
def test_non_string_checkpoint_id_raises(tmp_path, checkpoints_text, waypoints_text, match):
    cps = _write_checkpoints(tmp_path, checkpoints_text)
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, _HEAD + _HOME_ENU + waypoints_text), cps)


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


# TS-C5d (Hermes Medium): a RELATIVE checkpoints path is rejected when referenced — resolution must
# not depend on the working directory (the durability guarantee this milestone explicitly adds).
def test_relative_checkpoints_path_rejected(tmp_path):
    with pytest.raises(ValueError, match="absolute"):
        load_mission_config(
            _write(tmp_path, _HEAD + _HOME_ENU + _wp_checkpoint("cp_north")),
            "sim/config/checkpoints.yaml",  # relative — must be rejected before any open()
        )


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


# TS-C7 (Hermes Medium): each waypoint must be exactly one shape — checkpoint-based
# {checkpoint_id, dwell_s} or inline {position, frame, dwell_s}. A waypoint that mixes the two,
# is neither, or carries a missing/unexpected key is rejected fail-loud rather than silently honored
# (the old parser took checkpoint_id and dropped any stray inline position/frame). A valid
# checkpoints file is always supplied; it is simply ignored for the inline-only cases.
@pytest.mark.parametrize(
    ("waypoint", "match"),
    [
        (
            "  - checkpoint_id: cp_north\n    position: {x: 1, y: 1, z: 1}\n"
            "    frame: enu\n    dwell_s: 3.0\n",
            "ambiguous",
        ),
        ("  - checkpoint_id: cp_north\n    dwell_s: 3.0\n    altitude: 9\n", "unexpected"),
        (
            "  - position: {x: 1, y: 1, z: 1}\n    frame: enu\n    dwell_s: 3.0\n    speed: 2\n",
            "unexpected",
        ),
        ("  - dwell_s: 3.0\n", "must be checkpoint-based"),
        ("  - position: {x: 1, y: 1, z: 1}\n    dwell_s: 3.0\n", "missing required key"),
        ("  - checkpoint_id: cp_north\n", "missing required key"),
    ],
    ids=[
        "mixes_checkpoint_and_inline",
        "checkpoint_extra_key",
        "inline_extra_key",
        "neither_shape",
        "inline_missing_frame",
        "checkpoint_missing_dwell",
    ],
)
def test_waypoint_shape_fail_loud(tmp_path, waypoint, match):
    cps = _write_checkpoints(tmp_path)
    body = _HEAD + _HOME_ENU + "waypoints:\n" + waypoint
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, body), cps)


# TS-C7b (Hermes Medium): the fail-loud error names the offending waypoint *index* (here wp 1, a
# valid checkpoint wp 0 precedes it) so a malformed entry in a long route is locatable.
def test_waypoint_shape_error_is_indexed(tmp_path):
    cps = _write_checkpoints(tmp_path)
    body = (
        _HEAD
        + _HOME_ENU
        + (
            "waypoints:\n"
            "  - checkpoint_id: cp_north\n    dwell_s: 3.0\n"  # valid wp 0
            "  - position: {x: 1, y: 1, z: 1}\n    dwell_s: 3.0\n"  # wp 1: inline missing frame
        )
    )
    with pytest.raises(ValueError, match=r"waypoint\[1\]"):
        load_mission_config(_write(tmp_path, body), cps)


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


# Hermes polish: a NON-numeric scalar numeric field fails loud with one consistent
# "<field> must be a number" diagnostic — not a bare TypeError from a later range comparison (the
# section fields are built straight into their dataclass un-cast) nor an un-fielded float() ValueError
# (the top-level / dwell casts). Covers a top-level field, both section fields, and a waypoint dwell.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        (
            "takeoff_alt_m: abc\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS,
            "takeoff_alt_m must be a number",
        ),
        (
            "takeoff_alt_m: 5\nhover_time_s: abc\n" + _HOME_ENU + _NO_WAYPOINTS,
            "hover_time_s must be a number",
        ),
        (
            _HEAD
            + "completion: {tolerance_m: abc, hold_time_s: 2.0}\n"
            + _HOME_ENU
            + _NO_WAYPOINTS,
            "completion.tolerance_m must be a number",
        ),
        (
            _HEAD + "abort: {low_battery_threshold: abc}\n" + _HOME_ENU + _NO_WAYPOINTS,
            "abort.low_battery_threshold must be a number",
        ),
        (
            _HEAD
            + _HOME_ENU
            + "waypoints:\n  - position: {x: 1, y: 1, z: 1}\n    frame: enu\n    dwell_s: abc\n",
            r"waypoints\[0\]\.dwell_s must be a number",
        ),
    ],
    ids=[
        "takeoff_alt_nonnumeric",
        "hover_time_nonnumeric",
        "tolerance_nonnumeric",
        "battery_nonnumeric",
        "dwell_nonnumeric",
    ],
)
def test_nonnumeric_scalar_fields_fail_loud(tmp_path, body, match):
    with pytest.raises(ValueError, match=match):
        load_mission_config(_write(tmp_path, body))


# PR #8 post-mortem D: two scalars that float() would silently accept must fail loud at the numeric
# boundary. A YAML boolean (bool is an int subclass, so float(True) == 1.0) would smuggle a 1/0
# magnitude into a numeric field. A non-finite NaN/Inf slips past every range guard (nan/inf <= 0 are
# both False, so _positive/_non_negative accept it) and then poisons a state-machine comparison.
# Both raise the loader's contracted ValueError, across a top-level field, a section field, and a
# waypoint dwell; a quoted "nan" coerces to non-finite and is caught the same way.
@pytest.mark.parametrize(
    ("body", "match"),
    [
        (
            "takeoff_alt_m: true\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS,
            "takeoff_alt_m must be a number",
        ),
        (
            _HEAD + "abort: {low_battery_threshold: true}\n" + _HOME_ENU + _NO_WAYPOINTS,
            "abort.low_battery_threshold must be a number",
        ),
        (
            "takeoff_alt_m: .nan\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS,
            "takeoff_alt_m must be a finite number",
        ),
        (
            "takeoff_alt_m: .inf\nhover_time_s: 10\n" + _HOME_ENU + _NO_WAYPOINTS,
            "takeoff_alt_m must be a finite number",
        ),
        (
            _HEAD
            + 'completion: {tolerance_m: "nan", hold_time_s: 2.0}\n'
            + _HOME_ENU
            + _NO_WAYPOINTS,
            "completion.tolerance_m must be a finite number",
        ),
        (
            _HEAD
            + _HOME_ENU
            + "waypoints:\n  - position: {x: 1, y: 1, z: 1}\n    frame: enu\n    dwell_s: .inf\n",
            r"waypoints\[0\]\.dwell_s must be a finite number",
        ),
    ],
    ids=[
        "bool_top_level",
        "bool_section",
        "nan_top_level",
        "inf_top_level",
        "quoted_nan_section",
        "inf_waypoint_dwell",
    ],
)
def test_non_finite_and_bool_numeric_fields_fail_loud(tmp_path, body, match):
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


# Hermes Medium: a quoted numeric section value (`tolerance_m: "0.25"`) parses to a YAML *string*.
# The loader must coerce it to a float — exactly as it does the top-level numerics — so it cannot
# pass range validation yet later crash a state-machine comparison ('<' between float and str).
def test_quoted_section_numerics_stored_as_floats(tmp_path):
    body = (
        _HEAD
        + 'completion: {tolerance_m: "0.25", hold_time_s: "3.0"}\n'
        + 'abort: {low_battery_threshold: "0.35"}\n'
        + _HOME_ENU
        + _NO_WAYPOINTS
    )
    cfg = load_mission_config(_write(tmp_path, body))
    for value, expected in (
        (cfg.completion.tolerance_m, 0.25),
        (cfg.completion.hold_time_s, 3.0),
        (cfg.abort.low_battery_threshold, 0.35),
    ):
        assert value == expected
        assert isinstance(value, float)  # coerced, not the quoted str that would crash later


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
