"""Unit tests for scripts/check_manifest_drift.py and a live-repo no-drift regression guard.

This is the repo's first unit suite; it runs ROS-free (London-style, no rclpy) per CLAUDE.md.
The module is loaded by path because scripts/ is intentionally not a Python package.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "check_manifest_drift.py"


def _load_drift() -> Any:
    spec = importlib.util.spec_from_file_location("check_manifest_drift", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


drift = _load_drift()


def test_px4_release_line_extracts_major_minor():
    assert drift.px4_release_line("v1.17.0") == "1.17"
    assert drift.px4_release_line("1.16.2") == "1.16"


def test_alignment_passes_when_branch_matches_version():
    manifest = {"flight_stack": {"px4_version": "v1.17.0", "px4_msgs_ref": "release/1.17"}}
    assert drift.check_px4_msgs_alignment(manifest) == []


def test_alignment_flags_off_release_line_branch():
    manifest = {"flight_stack": {"px4_version": "v1.17.0", "px4_msgs_ref": "release/1.16"}}
    problems = drift.check_px4_msgs_alignment(manifest)
    assert len(problems) == 1
    assert "off px4_version's release line" in problems[0]


def _setup_text_with_all_derivations() -> str:
    """A synthetic setup script that derives every pinned var via manifest_get (the clean case)."""
    return "\n".join(f'{var}="$(manifest_get {key})"' for var, key in drift.DERIVED_VARS.items())


def test_missing_derivations_clean_when_all_derived():
    assert drift.missing_derivations(_setup_text_with_all_derivations()) == []


def test_missing_derivations_flags_inlined_literal():
    # Simulate someone replacing the QGC_SHA256 derivation with a hardcoded literal.
    text = _setup_text_with_all_derivations().replace(
        'QGC_SHA256="$(manifest_get apps.qgc_sha256)"',
        'QGC_SHA256="deadbeef"',
    )
    problems = drift.missing_derivations(text)
    assert len(problems) == 1
    assert "QGC_SHA256" in problems[0]
    assert "apps.qgc_sha256" in problems[0]


def _write_sim_dockerfile(root: Path, contents: str) -> None:
    sim = root / "docker" / "sim"
    sim.mkdir(parents=True, exist_ok=True)
    (sim / "Dockerfile").write_text(contents)


# Minimal manifest the literal check derives its forbidden distro/sim words from.
_LITERAL_MANIFEST = {"middleware": {"ros_distro": "jazzy"}, "simulator": {"gazebo": "harmonic"}}


def test_dockerfile_literals_clean_when_args_used(tmp_path):
    _write_sim_dockerfile(
        tmp_path,
        "ARG PX4_VERSION\n"
        "RUN git clone --branch ${PX4_VERSION} https://example/PX4.git\n"
        'RUN apt-get install -y "gz-${GZ_VERSION}" "ros-${ROS_DISTRO}-foo"\n',
    )
    assert drift.check_dockerfile_no_literals(tmp_path, _LITERAL_MANIFEST) == []


def test_dockerfile_literals_clean_with_inline_comment_mentioning_tokens(tmp_path):
    # Trailing inline comments mentioning harmonic / a version must NOT trip the gate.
    _write_sim_dockerfile(
        tmp_path,
        'RUN apt-get install -y "gz-${GZ_VERSION}"   # the gz-harmonic metapackage, pinned v1.17.0\n'
        "RUN git clone --branch ${PX4_VERSION} https://example/PX4.git   # jazzy-era PX4\n",
    )
    assert drift.check_dockerfile_no_literals(tmp_path, _LITERAL_MANIFEST) == []


def test_dockerfile_literals_clean_for_unrelated_version_token(tmp_path):
    # A non-PX4 version-shaped token (a pinned pip dep, a URL path) must NOT be misflagged.
    _write_sim_dockerfile(
        tmp_path,
        "RUN pip install foo==1.2.3 && curl -fsSL https://example/v2.0/key.gpg -o /k.gpg\n",
    )
    assert drift.check_dockerfile_no_literals(tmp_path, _LITERAL_MANIFEST) == []


def test_dockerfile_literals_flag_hardcoded_px4_branch(tmp_path):
    _write_sim_dockerfile(
        tmp_path,
        "ARG PX4_VERSION\nRUN git clone --branch v9.99.0 https://example/PX4.git\n",
    )
    problems = drift.check_dockerfile_no_literals(tmp_path, _LITERAL_MANIFEST)
    assert len(problems) == 1
    assert "docker/sim/Dockerfile" in problems[0]
    assert "PX4 version" in problems[0]


def test_dockerfile_literals_flag_hardcoded_distro_and_sim(tmp_path):
    _write_sim_dockerfile(
        tmp_path,
        'RUN apt-get install -y gz-harmonic "ros-jazzy-foo"\n',
    )
    problems = drift.check_dockerfile_no_literals(tmp_path, _LITERAL_MANIFEST)
    assert any("ROS distro" in p for p in problems)
    assert any("Gazebo version" in p for p in problems)


def test_live_repo_has_no_manifest_drift():
    assert drift.run_checks(REPO_ROOT) == []
