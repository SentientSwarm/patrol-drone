"""Unit test for docker-compose.ci.yml — the CI-only network override (Hermes Medium #2).

ROS-free and dependency-free (no pyyaml — not a declared dev dep; mirrors the text/regex parsing
in test_gen_build_args.py). The override is a tiny scalar, so a regex assertion is enough.

This pins the trust-boundary contract the override exists to enforce: the SITL nightly must run the
`sim` service on default *bridge* networking, while the local-dev base keeps *host* networking (so
an interactive host QGC/ROS 2 can reach the same /fmu/* bridge). Without this test, deleting the
override file — or flipping the base to bridge — would silently regress CI back onto the runner's
host network namespace, the exact gap Hermes Medium #2 flagged.
"""

from __future__ import annotations

import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_BASE = REPO_ROOT / "docker-compose.yml"
_CI = REPO_ROOT / "docker-compose.ci.yml"

# `network_mode: <value>` scalar (captures up to the first whitespace, so trailing comments drop).
_NETWORK_MODE = re.compile(r"^\s*network_mode:\s*(\S+)", re.MULTILINE)


def test_ci_override_puts_sim_on_bridge_networking():
    modes = _NETWORK_MODE.findall(_CI.read_text())
    assert modes == ["bridge"], f"CI override must set exactly one network_mode=bridge, got {modes}"


def test_base_compose_keeps_host_networking_for_local_dev():
    # The override narrows CI only; interactive local sim still needs host net (agent<->PX4 + a host
    # QGC/ROS 2 on the same loopback), so the base must retain a host-networked service.
    assert "host" in _NETWORK_MODE.findall(_BASE.read_text())
