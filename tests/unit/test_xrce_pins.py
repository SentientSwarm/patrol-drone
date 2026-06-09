"""Unit tests for scripts/check_xrce_pins.py + a live-manifest immutable-tag regression guard.

ROS-free (London-style, no rclpy) and network-free per CLAUDE.md: the `git ls-remote` call is
monkeypatched, so the parsing/decision logic is exercised without reaching GitHub (the live network
check is the CI job `xrce-pins`). The module is loaded by path because scripts/ is not a package.

The load-bearing guard here is `test_all_transitive_refs_are_immutable_tags`: it pins the ADR-0007
decision that the uXRCE transitive deps must be IMMUTABLE TAGS, not moving branches. That regression
(Fast-DDS `3.x` advancing past the pin) blocked the M2 bridge build (Hermes High, head 8b85069); a
future edit dropping a ref back to a branch (`3.x`, `2.2.x`) must fail fast in unit CI, not silently
re-open the drift window until the next agent build.
"""

from __future__ import annotations

import importlib.util
import re
import tomllib
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "check_xrce_pins.py"
_MANIFEST = REPO_ROOT / "stack-manifest.toml"

# An immutable git tag pin: a SemVer-ish `vMAJOR...` tag. A moving branch (`3.x`, `2.2.x`, `main`)
# must NOT match — that is exactly the shape that drifts and breaks the pinned build.
_IMMUTABLE_TAG = re.compile(r"^v\d+\.\d")


def _load_pins() -> Any:
    spec = importlib.util.spec_from_file_location("check_xrce_pins", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


pins = _load_pins()


def _bridge() -> dict:
    with _MANIFEST.open("rb") as fh:
        return tomllib.load(fh)["bridge"]


def test_all_transitive_refs_are_immutable_tags():
    # ADR-0007: agent + every transitive superbuild dep is pinned to an immutable tag (no moving
    # branch), so the pre-build ls-remote gate and the superbuild fetch can never drift.
    bridge = _bridge()
    for name, _url, ref, _commit in pins.pinned_refs(bridge):
        assert _IMMUTABLE_TAG.match(ref), (
            f"{name} ref {ref!r} is not an immutable tag (moving branch?)"
        )


def test_pinned_refs_covers_agent_and_four_deps():
    names = {name for name, *_ in pins.pinned_refs(_bridge())}
    assert names == {"Micro-XRCE-DDS-Agent", "fastcdr", "fastdds", "foonathan_memory", "spdlog"}


def _fake_ls_remote(monkeypatch, stdout: str):
    class _Result:
        def __init__(self, out: str):
            self.stdout = out

    def _run(_cmd, **_kwargs):
        return _Result(stdout)

    monkeypatch.setattr(pins.subprocess, "run", _run)


def test_resolve_ref_prefers_peeled_annotated_tag(monkeypatch):
    # Annotated tag: the peeled (^{}) line is the commit the superbuild checks out — prefer it.
    _fake_ls_remote(
        monkeypatch,
        "aaaa\trefs/tags/v3.1.3\nbbbb\trefs/tags/v3.1.3^{}\n",
    )
    assert pins.resolve_ref("url", "v3.1.3") == "bbbb"


def test_resolve_ref_lightweight_tag_uses_plain_line(monkeypatch):
    _fake_ls_remote(monkeypatch, "cccc\trefs/tags/v2.2.4\n")
    assert pins.resolve_ref("url", "v2.2.4") == "cccc"


def test_resolve_ref_ignores_glob_siblings(monkeypatch):
    # ls-remote globs: querying `3.x` also returns `integration/3.x` — only the exact ref counts.
    _fake_ls_remote(
        monkeypatch,
        "dead\trefs/heads/integration/3.x\nbeef\trefs/heads/3.x\n",
    )
    assert pins.resolve_ref("url", "3.x") == "beef"


def test_resolve_ref_returns_none_when_absent(monkeypatch):
    _fake_ls_remote(monkeypatch, "eeee\trefs/tags/v9.9.9\n")
    assert pins.resolve_ref("url", "v3.1.3") is None


def test_check_pin_passes_on_match(monkeypatch):
    _fake_ls_remote(monkeypatch, "1234\trefs/tags/v3.1.3\n")
    assert pins.check_pin("fastdds", "url", "v3.1.3", "1234") is None


def test_check_pin_flags_mismatch(monkeypatch):
    _fake_ls_remote(monkeypatch, "9999\trefs/tags/v3.1.3\n")
    problem = pins.check_pin("fastdds", "url", "v3.1.3", "1234")
    assert problem is not None
    assert "resolves to 9999" in problem
    assert "pins 1234" in problem
