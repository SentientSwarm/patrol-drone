#!/usr/bin/env python3
"""Fail if any pinned uXRCE-DDS git ref no longer resolves to its manifest commit.

The Micro XRCE-DDS Agent and its four transitive superbuild deps (Fast-CDR, Fast-DDS,
foonathan_memory, spdlog) are pinned in stack-manifest.toml [bridge] by REF + COMMIT, and
build_xrce_agent.sh verifies them PRE-build (`git ls-remote`) and POST-build (checked-out HEAD).
That fail-closed gate only runs during the (heavy) agent build, so a drifted/tampered ref used to
surface ONLY in the nightly reviewer — not in PR CI (Hermes High, head 8b85069: the Fast-DDS `3.x`
moving branch advanced past the pin and silently broke the build until a human noticed).

This checker runs the SAME ls-remote resolution as the build's pre-build gate, but standalone and
network-only (no clone, no compile), so it is cheap enough to run on every PR. With the deps now
pinned to immutable TAGS (ADR-0007), it should stay green unless an upstream tag is force-pushed or
the agent pin is bumped without re-capturing a dep commit — both of which are exactly what we want a
PR to fail on. Pure stdlib (tomllib + git) — no uv/deps, mirrors check_manifest_drift.py.

Exit 0 = every pin resolves to its manifest commit; exit 1 = one line per mismatch/unreachable ref.
"""

from __future__ import annotations

import subprocess
import sys
import tomllib
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# Upstream repo URLs for the transitive superbuild deps — kept in lockstep with the URLs
# build_xrce_agent.sh verifies (the manifest stores only the ref/commit, not the dep URLs).
_DEP_URLS = {
    "fastcdr": "https://github.com/eProsima/Fast-CDR.git",
    "fastdds": "https://github.com/eProsima/Fast-DDS.git",
    "foonathan_memory": "https://github.com/foonathan/memory.git",
    "spdlog": "https://github.com/gabime/spdlog.git",
}


def load_bridge(repo_root: Path) -> dict:
    with (repo_root / "stack-manifest.toml").open("rb") as fh:
        return tomllib.load(fh)["bridge"]


def pinned_refs(bridge: dict) -> list[tuple[str, str, str, str]]:
    """(name, url, ref, expected_commit) for the agent + each transitive dep."""
    pins = [
        (
            "Micro-XRCE-DDS-Agent",
            bridge["uxrce_dds_agent_source"],
            bridge["uxrce_dds_agent_version"],
            bridge["uxrce_dds_agent_commit"],
        )
    ]
    for dep, url in _DEP_URLS.items():
        pins.append((dep, url, bridge[f"uxrce_{dep}_ref"], bridge[f"uxrce_{dep}_commit"]))
    return pins


def resolve_ref(url: str, ref: str) -> str | None:
    """The commit `ref` resolves to on `url` RIGHT NOW, or None if unreachable/absent.

    Matches the ref exactly (ls-remote globs: `3.x` would also match `integration/3.x`) and prefers
    the peeled (^{}) line so an annotated tag resolves to its commit — identical to the build's gate.
    """
    try:
        out = subprocess.run(
            ["git", "ls-remote", url, ref, f"refs/tags/{ref}^{{}}"],
            capture_output=True,
            text=True,
            timeout=30,
            check=True,
        ).stdout
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return None
    plain = peeled = None
    for line in out.splitlines():
        sha, _, name = line.partition("\t")
        if name in (f"refs/heads/{ref}", f"refs/tags/{ref}"):
            plain = sha
        elif name == f"refs/tags/{ref}^{{}}":
            peeled = sha
    return peeled or plain


def check_pin(name: str, url: str, ref: str, expected: str) -> str | None:
    """None if the pin resolves to `expected`; otherwise a one-line problem description."""
    resolved = resolve_ref(url, ref)
    if resolved is None:
        return f"{name}: ref {ref!r} not resolvable on {url} (unreachable or deleted)"
    if resolved != expected:
        return f"{name}: ref {ref!r} resolves to {resolved} but [bridge] pins {expected}"
    return None


def main() -> int:
    bridge = load_bridge(REPO_ROOT)
    problems = []
    for name, url, ref, expected in pinned_refs(bridge):
        problem = check_pin(name, url, ref, expected)
        if problem:
            problems.append(problem)
        else:
            print(f"OK: {name} {ref} -> {expected}")
    if problems:
        print("\nuXRCE pin drift detected:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        print(
            "\nRe-resolve (git ls-remote <repo> <ref>) and bump stack-manifest.toml [bridge], "
            "or pin a different immutable tag (ADR-0007).",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
