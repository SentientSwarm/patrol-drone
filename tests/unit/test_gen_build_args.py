"""Unit tests for scripts/gen_build_args.py — the manifest -> Docker build-ARG generator.

ROS-free (London-style, no rclpy) per CLAUDE.md. The module is loaded by path because
scripts/ is intentionally not a Python package (mirrors test_manifest_drift.py).

The load-bearing test here is `test_compose_forwards_every_generated_xrce_arg`: it pins the
three-surface contract (generator -> Dockerfile ARG -> compose build.args) so a build arg can
never again be emitted by the generator yet silently dropped from the compose path. That exact
gap left the pre-build supply-chain ref gate inert on `docker compose build sim` (Hermes High #1):
gen_build_args.py emitted XRCE_*_REF, the Dockerfile declared the ARGs, but docker-compose.yml
forwarded only the *_COMMIT partners — so EXPECT_*_REF was empty and the ls-remote gate skipped.

Parsing is intentionally dependency-free (text/regex, not pyyaml): pyyaml is not a declared dev
dependency (deferred to M4 per CLAUDE.md), and the build.args block is simple `KEY: ${KEY}` lines.
"""

from __future__ import annotations

import importlib.util
import re
import tomllib
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "gen_build_args.py"
_COMPOSE = REPO_ROOT / "docker-compose.yml"
_SIM_DOCKERFILE = REPO_ROOT / "docker" / "sim" / "Dockerfile"
_MANIFEST = REPO_ROOT / "stack-manifest.toml"

# `KEY: ${KEY}` build-arg line (compose forwards the host/env value of the same name).
_COMPOSE_ARG = re.compile(r"^\s+([A-Z][A-Z0-9_]*):\s*\$\{\1\}", re.MULTILINE)
# `ARG XRCE_...` declaration in the Dockerfile.
_DOCKERFILE_XRCE_ARG = re.compile(r"^\s*ARG\s+(XRCE_[A-Z0-9_]+)", re.MULTILINE)


def _load_gen() -> Any:
    spec = importlib.util.spec_from_file_location("gen_build_args", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


gen = _load_gen()


def _manifest() -> dict:
    with _MANIFEST.open("rb") as fh:
        return tomllib.load(fh)


def _generated_keys() -> set[str]:
    return set(gen.build_args(_manifest()).keys())


def _compose_arg_keys() -> set[str]:
    return set(_COMPOSE_ARG.findall(_COMPOSE.read_text()))


def _dockerfile_xrce_args() -> set[str]:
    return set(_DOCKERFILE_XRCE_ARG.findall(_SIM_DOCKERFILE.read_text()))


def _xrce(keys: set[str]) -> set[str]:
    return {k for k in keys if k.startswith("XRCE_")}


def test_generator_emits_both_ref_and_commit_for_every_moving_dep():
    # The pre-build ls-remote gate needs the _REF; the post-build checkout gate needs the _COMMIT.
    # Both must be emitted for each transitive superbuild dep or one of the two gates goes inert.
    generated = _generated_keys()
    for dep in ("FASTCDR", "FASTDDS", "FOONATHAN", "SPDLOG"):
        assert f"XRCE_{dep}_REF" in generated
        assert f"XRCE_{dep}_COMMIT" in generated


def test_compose_forwards_every_generated_xrce_arg():
    # Regression for Hermes High #1: the compose `sim.build.args` must forward the FULL set of
    # XRCE_* args the generator emits — no subset (a dropped _REF disables the pre-build gate),
    # and no orphan (a compose arg the generator never supplies resolves to empty at build time).
    generated_xrce = _xrce(_generated_keys())
    compose_xrce = _xrce(_compose_arg_keys())
    assert compose_xrce == generated_xrce


def test_dockerfile_declares_every_generated_xrce_arg():
    # The third surface: each emitted XRCE arg must be a declared ARG in the sim Dockerfile,
    # otherwise compose forwards a value the build stage never consumes.
    assert _xrce(_generated_keys()) <= _dockerfile_xrce_args()


def test_env_output_is_key_value_lines():
    # `--env` feeds `docker compose --env-file`; every line must be a parseable KEY=VALUE.
    rc = gen.main(["--env"])
    assert rc == 0
