"""Unit tests for the M5 World Composer (sim/tools/compose_world.py).

Layer-A: pure stdlib, deterministic, no ROS/Gazebo — mirrors the plan's "don't mock the simulator;
test the construction logic" discipline. Covers the fail-loud Guards (design §4.2.4 / INF-S3) and the
drift check (INF-S2). Shared builders + parametrize keep the cases duplication-free (CodeScene gate).
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import compose_world as cw

# Minimal template with the placeholder; isolates the composer from the full hand-authored world.
TEMPLATE = (
    '<sdf version="1.9">\n  <world name="t">\n    <!-- CHECKPOINT_MARKERS -->\n  </world>\n</sdf>\n'
)

# tag_ids 0/1/2 have real model dirs under sim/models, so the model-dir Guard passes for them.
_FLOW_POS = "{ x: 12.0, y: 8.0, z: 1.5 }"


# Default field values for one checkpoint entry; ordered as they appear in the YAML. Override any
# via _entry(field=...), or pass field=None to omit it (missing-field tests).
_DEFAULT_FIELDS: dict[str, str | None] = {
    "checkpoint_id": '"cp_a"',
    "position": _FLOW_POS,
    "tag_family": '"tag36h11"',
    "tag_id": "0",
}


def _entry(indent: str = "  ", **overrides: str | None) -> str:
    """Build one YAML checkpoint entry; pass ``field=None`` to omit it (missing-field tests)."""
    fields = {**_DEFAULT_FIELDS, **overrides}
    present = [(k, v) for k, v in fields.items() if v is not None]
    return "\n".join(
        f"{indent}{'- ' if i == 0 else '  '}{k}: {v}" for i, (k, v) in enumerate(present)
    )


def _keyed(*entries: str) -> str:
    return "checkpoints:\n" + "\n".join(entries) + "\n"


def _write(tmp_path: Path, text: str, name: str = "checkpoints.yaml") -> str:
    p = tmp_path / name
    p.write_text(text)
    return str(p)


def _compose(tmp_path: Path, config_text: str, template: str = TEMPLATE) -> str:
    cfg = _write(tmp_path, config_text)
    tpl = _write(tmp_path, template, "world.template.sdf")
    out = tmp_path / "world.sdf"
    return cw.compose_world(cfg, tpl, str(out))


def _composed(tmp_path: Path, config_text: str, template: str = TEMPLATE) -> tuple[str, str, str]:
    """Write cfg + template, compose the world, return (cfg_path, tpl_path, out_path)."""
    cfg = _write(tmp_path, config_text)
    tpl = _write(tmp_path, template, "t.sdf")
    out = tmp_path / "world.sdf"
    cw.compose_world(cfg, tpl, str(out))
    return cfg, tpl, str(out)


# --- happy path ----------------------------------------------------------------------------------

_TWO = _keyed(
    _entry(checkpoint_id='"cp_a"', position="{ x: 12.0, y: 8.0, z: 1.5 }", tag_id="0"),
    _entry(checkpoint_id='"cp_b"', position="{ x: 18.0, y: -6.0, z: 1.5 }", tag_id="1"),
)


def test_compose_emits_one_marker_per_checkpoint_at_its_position(tmp_path):
    world = _compose(tmp_path, _TWO)
    root = ET.fromstring(world)
    includes = {
        inc.findtext("name"): (inc.findtext("uri"), inc.findtext("pose"))
        for inc in root.iter("include")
    }
    assert includes["checkpoint_cp_a"] == ("model://apriltag_36h11_0", "12 8 1.5 0 0 0")
    assert includes["checkpoint_cp_b"][0] == "model://apriltag_36h11_1"
    assert (includes["checkpoint_cp_b"][1] or "").startswith("18 -6 1.5")


@pytest.mark.parametrize("indent", ["  ", ""])  # keyed-with-indent and bare-list both parse
def test_parser_accepts_keyed_and_bare_list(tmp_path, indent):
    entry = _entry(indent=indent)
    text = _keyed(entry) if indent else entry + "\n"
    checkpoints = cw.load_checkpoints(_write(tmp_path, text))
    assert [c.checkpoint_id for c in checkpoints] == ["cp_a"]
    assert checkpoints[0].uri == "model://apriltag_36h11_0"


# --- fail-loud Guards ----------------------------------------------------------------------------


@pytest.mark.parametrize(
    "field",
    ["checkpoint_id", "position", "tag_family", "tag_id"],
)
def test_missing_required_field_fails_loud(tmp_path, field):
    base: dict[str, str | None] = {
        "checkpoint_id": '"cp_a"',
        "position": _FLOW_POS,
        "tag_family": '"tag36h11"',
        "tag_id": "0",
    }
    base[field] = None  # omit exactly the field under test
    entry = _entry(
        checkpoint_id=base["checkpoint_id"],
        position=base["position"],
        tag_family=base["tag_family"],
        tag_id=base["tag_id"],
    )
    with pytest.raises(cw.ComposeError, match=field):
        cw.load_checkpoints(_write(tmp_path, _keyed(entry)))


def test_duplicate_tag_id_fails_loud(tmp_path):
    text = _keyed(
        _entry(checkpoint_id='"cp_a"', tag_id="0"), _entry(checkpoint_id='"cp_b"', tag_id="0")
    )
    with pytest.raises(cw.ComposeError, match="duplicate tag_id 0"):
        cw.compose_world(
            _write(tmp_path, text), _write(tmp_path, TEMPLATE, "t.sdf"), str(tmp_path / "o.sdf")
        )


def test_duplicate_checkpoint_id_fails_loud(tmp_path):
    text = _keyed(
        _entry(checkpoint_id='"cp_a"', tag_id="0"), _entry(checkpoint_id='"cp_a"', tag_id="1")
    )
    with pytest.raises(cw.ComposeError, match="duplicate checkpoint_id 'cp_a'"):
        cw.compose_world(
            _write(tmp_path, text), _write(tmp_path, TEMPLATE, "t.sdf"), str(tmp_path / "o.sdf")
        )


def test_tag_id_without_model_dir_fails_loud(tmp_path):
    text = _keyed(_entry(tag_id="99"))  # no sim/models/apriltag_36h11_99
    with pytest.raises(cw.ComposeError, match="model dir"):
        cw.compose_world(
            _write(tmp_path, text), _write(tmp_path, TEMPLATE, "t.sdf"), str(tmp_path / "o.sdf")
        )


@pytest.mark.parametrize(
    "position",
    [
        "{ x: 12.0, y: 8.0 }",  # missing z
        "{ x: foo, y: 8.0, z: 1.5 }",  # non-numeric x
        "{ x: 12abc, y: 8.0, z: 1.5 }",  # trailing junk (F-04)
        "{ x: 1.2.3, y: 8.0, z: 1.5 }",  # multi-dot (F-04)
        "{ x: inf, y: 8.0, z: 1.5 }",  # non-finite (F-04)
        "",  # block-style (empty inline value) — fail loud, not silent misread
    ],
)
def test_malformed_position_fails_loud(tmp_path, position):
    text = _keyed(_entry(position=position))
    with pytest.raises(cw.ComposeError, match="position"):
        cw.load_checkpoints(_write(tmp_path, text))


@pytest.mark.parametrize(
    ("position", "expected"),
    [("{ x: .5, y: 1e3, z: -1.5e-2 }", (0.5, 1000.0, -0.015))],
)
def test_position_accepts_valid_float_forms(tmp_path, position, expected):
    cp = cw.load_checkpoints(_write(tmp_path, _keyed(_entry(position=position))))[0]
    assert (cp.x, cp.y, cp.z) == expected


def test_unsupported_tag_family_fails_loud(tmp_path):
    text = _keyed(_entry(tag_family='"tag25h9"'))
    with pytest.raises(cw.ComposeError, match="tag_family"):
        cw.load_checkpoints(_write(tmp_path, text))


def test_non_int_tag_id_fails_loud(tmp_path):
    text = _keyed(_entry(tag_id='"north"'))
    with pytest.raises(cw.ComposeError, match="tag_id must be an int"):
        cw.load_checkpoints(_write(tmp_path, text))


@pytest.mark.parametrize(
    "bad_id",
    [
        "\"a</name><plugin name='x' filename='y'/>\"",  # XML breakout (the F-01 exploit)
        '"a&b"',  # bare ampersand
        '"cp north"',  # whitespace
        '"north#1"',  # '#' (no leading whitespace, so _strip_comment leaves it intact)
        '"cp/north"',  # path separator into the model:// namespace
        '""',  # empty after quote-strip
    ],
)
def test_checkpoint_id_must_be_allowlisted(tmp_path, bad_id):
    text = _keyed(_entry(checkpoint_id=bad_id))
    with pytest.raises(cw.ComposeError, match="checkpoint_id"):
        cw.load_checkpoints(_write(tmp_path, text))


def test_unexpected_entry_field_fails_loud(tmp_path):
    text = _keyed(_entry(dwell_s="5.0"))  # a stray waypoint field that belongs in the mission YAML
    with pytest.raises(cw.ComposeError, match="unexpected"):
        cw.load_checkpoints(_write(tmp_path, text))


@pytest.mark.parametrize(
    "bad_uri",
    [
        "/abs/path/to/model",
        "~/model",
        "file:///etc/passwd",
        "C:\\models\\tag",
        "../escapes/repo",
    ],
)
def test_non_portable_uri_rejected(bad_uri):
    with pytest.raises(cw.ComposeError, match="non-portable"):
        cw._check_uri_portable(bad_uri)


@pytest.mark.parametrize("good_uri", ["model://apriltag_36h11_0", "models/apriltag_36h11_0"])
def test_portable_uri_accepted(good_uri):
    cw._check_uri_portable(good_uri)  # does not raise


def test_missing_placeholder_in_template_fails_loud(tmp_path):
    with pytest.raises(cw.ComposeError, match="placeholder"):
        _compose(tmp_path, _TWO, template="<sdf><world name='t'></world></sdf>")


def test_duplicate_placeholder_in_template_fails_loud(tmp_path):
    doubled = TEMPLATE.replace("</world>", "    <!-- CHECKPOINT_MARKERS -->\n  </world>")
    with pytest.raises(cw.ComposeError, match="exactly once"):
        _compose(tmp_path, _TWO, template=doubled)


def test_empty_checkpoints_fails_loud(tmp_path):
    with pytest.raises(cw.ComposeError, match="no checkpoints"):
        cw.load_checkpoints(_write(tmp_path, "checkpoints:\n"))


# --- drift check (INF-S2) ------------------------------------------------------------------------


def test_check_drift_passes_when_world_matches_config(tmp_path):
    cfg, tpl, out = _composed(tmp_path, _TWO)
    assert cw.check_drift(cfg, out, template_path=tpl) == []


def test_check_drift_detects_position_mismatch(tmp_path):
    _cfg, tpl, out = _composed(tmp_path, _TWO)
    moved = _write(tmp_path, _TWO.replace("x: 12.0", "x: 99.0"), "moved.yaml")
    problems = cw.check_drift(moved, out, template_path=tpl)
    assert any("position" in p for p in problems)


def test_check_drift_detects_missing_marker(tmp_path):
    _cfg, tpl, out = _composed(tmp_path, _TWO)
    one = _write(tmp_path, _keyed(_entry(checkpoint_id='"cp_c"', tag_id="2")), "one.yaml")
    problems = cw.check_drift(one, out, template_path=tpl)
    assert any("no marker in the world" in p for p in problems)


def test_check_drift_detects_template_body_change(tmp_path):
    cfg, _tpl, out = _composed(tmp_path, _TWO)
    # Edit a non-marker template body line without regenerating the committed world.
    changed = TEMPLATE.replace(
        '<world name="t">', '<world name="t">\n    <gravity>0 0 -9.8</gravity>'
    )
    tpl_b = _write(tmp_path, changed, "b.sdf")
    problems = cw.check_drift(cfg, out, template_path=tpl_b)
    assert any("fresh render" in p for p in problems)


def test_check_drift_runs_model_dir_guard(tmp_path):
    cfg = _write(tmp_path, _keyed(_entry(tag_id="99")))  # no sim/models/apriltag_36h11_99
    with pytest.raises(cw.ComposeError, match="model dir"):
        cw.check_drift(cfg, str(tmp_path / "world.sdf"))


def test_models_dir_threads_through_compose_and_drift(tmp_path):
    # tag_id 99 has NO dir under the live sim/models — so render/compose/check_drift pass ONLY if the
    # SUPPLIED models_dir is honored (the F-03 seam). A regression to the global MODELS_DIR would
    # raise ComposeError, failing this test. No monkeypatch, no dependence on the committed tree.
    models = tmp_path / "models"
    (models / "apriltag_36h11_99").mkdir(parents=True)
    cfg = _write(tmp_path, _keyed(_entry(tag_id="99")))
    tpl = _write(tmp_path, TEMPLATE, "world.template.sdf")
    out = tmp_path / "world.sdf"
    cw.compose_world(cfg, tpl, str(out), models_dir=models)
    assert "<uri>model://apriltag_36h11_99</uri>" in out.read_text()
    assert cw.check_drift(cfg, str(out), template_path=tpl, models_dir=models) == []


# --- shipped assets (integration guard, like test_config's shipped-file tests) -------------------


def test_shipped_world_in_sync_with_shipped_checkpoints():
    assert cw.check_drift() == [], "run: python3 sim/tools/compose_world.py"


def test_shipped_world_is_well_formed_and_has_three_markers():
    root = ET.fromstring(Path(cw.WORLD_PATH).read_text())
    markers = [
        inc
        for inc in root.iter("include")
        if (inc.findtext("name") or "").startswith("checkpoint_")
    ]
    assert len(markers) >= 3


# --- canonical world-design invariants (F-02) ----------------------------------------------------


def _pos(x: float, y: float) -> str:
    return f"{{ x: {x}, y: {y}, z: 1.5 }}"


def _design_cfg(*coords_and_ids: tuple[float, float, int]) -> str:
    """Keyed config of one checkpoint per (x, y, tag_id) triple (ids drive checkpoint_id too)."""
    return _keyed(
        *(
            _entry(checkpoint_id=f'"cp_{tid}"', position=_pos(x, y), tag_id=str(tid))
            for x, y, tid in coords_and_ids
        )
    )


@pytest.mark.parametrize(
    ("cfg_text", "expected"),
    [
        (_design_cfg((0.0, 0.0, 0), (12.0, 0.0, 1)), ">= 3"),  # only 2 checkpoints
        (_design_cfg((99.0, 0.0, 0), (0.0, 10.0, 1), (10.0, -10.0, 2)), "outside"),  # |x| > 20
        (_design_cfg((0.0, 0.0, 0), (1.0, 0.0, 1), (0.0, 12.0, 2)), "apart"),  # a-b 1 m < 8 m
        (_design_cfg((0.0, 0.0, 0), (12.0, 0.0, 1), (0.0, 12.0, 3)), "contiguous"),  # tag_id gap
    ],
)
def test_validate_world_design_flags_invariant_violations(tmp_path, cfg_text, expected):
    problems = cw.validate_world_design(_write(tmp_path, cfg_text))
    assert any(expected in p for p in problems), problems


def test_shipped_checkpoints_satisfy_world_design():
    assert cw.validate_world_design() == [], (
        "edit sim/config/checkpoints.yaml to restore invariants"
    )
