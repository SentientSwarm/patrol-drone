# ADR-0007: Micro XRCE-DDS Agent is built from source, not apt-installed

**Status:** Accepted
**Date:** 2026-06-05
**Deciders:** Egemen Cankaya (project owner)

## Context

M2 (ROS 2 Jazzy + uXRCE-DDS bridge) requires the **Micro XRCE-DDS Agent** — the native
PX4 ↔ ROS 2 bridge process (`MicroXRCEAgent`). The M1 design and bootstrap assumed it was an
apt package: [`docs/phase1/01-platform/design.md` §4.2.1](../phase1/01-platform/design.md)
sketched `apt install ros-${ROS_DISTRO}-micro-xrce-dds-agent`, `scripts/setup_phase1.sh`
installed the same name, and `stack-manifest.toml [bridge]` recorded
`ros-jazzy-micro-xrce-dds-agent` as the pin.

The M1–M2 integration spike (the very early-adopter friction the Phase 1 plan budgets for)
**falsified that assumption**: there is **no `ros-jazzy-micro-xrce-dds-agent` package** (nor any
package whose name contains `xrce`) in the ROS 2 Jazzy apt repository. Verified two ways:

- On the host (`/opt/ros/jazzy`, live `ros2.sources`): `apt-cache madison
  ros-jazzy-micro-xrce-dds-agent` and `apt-cache policy …` both return empty; no `micro-xrce`
  package exists.
- In the `osrf/ros:jazzy-desktop` base image (3206 `ros-jazzy-*` packages present): same result.

The apt path was never exercised in M1 because `setup_phase1.sh` had not been re-run on the
host (the agent was simply absent), so the broken install surfaced only when the M2 sim
container build hit `E: Unable to locate package ros-jazzy-micro-xrce-dds-agent`.

## Decision

**Build the Micro XRCE-DDS Agent from source at a pinned eProsima tag — the PX4-canonical
method** — in both the sim container and the host bootstrap, so the two environments produce
the identical `MicroXRCEAgent` binary.

- **Pin** in `stack-manifest.toml [bridge]`: `uxrce_dds_agent_source` (the eProsima repo),
  `uxrce_dds_agent_version = "v3.0.1"` (latest stable), and `uxrce_dds_agent_commit` (the SHA
  the tag dereferences to). The version flows to the Dockerfile via
  `scripts/gen_build_args.py` (`XRCE_AGENT_VERSION`, no literal in the Dockerfile) and to the
  bootstrap via `manifest_get bridge.uxrce_dds_agent_version`.
- **Container** (`docker/sim/Dockerfile`, runtime stage): `git clone --branch v3.0.1`, then a
  `cmake` build with `-DUAGENT_BUILD_EXECUTABLE=ON -DUAGENT_BUILD_TESTS=OFF`; the cmake
  superbuild pulls Fast-DDS/Fast-CDR. The build toolchain is already present because the
  runtime stage derives `FROM px4-build`.
- **Host** (`scripts/setup_phase1.sh::install_xrce_agent`): clone the pinned tag, verify
  `HEAD == uxrce_dds_agent_commit` (catches an upstream-moved tag), `cmake` build, `sudo cmake
  --install`, `sudo ldconfig`. Idempotent: skips when `MicroXRCEAgent` is already on `PATH`.

## Transitive dependency pinning — immutable tags, not moving branches (2026-06-09)

The agent's cmake superbuild fetches four transitive deps (Fast-CDR, Fast-DDS, foonathan_memory,
spdlog). Upstream, the agent v3.0.1 `CMakeLists.txt` points Fast-DDS/Fast-CDR at the **moving
branches** `set(_fastdds_tag 3.x)` / `set(_fastcdr_tag 2.2.x)`. We initially mirrored that —
pinning each dep by branch ref + a tripwire commit in `stack-manifest.toml [bridge]`, with
`build_xrce_agent.sh` failing closed when `git ls-remote <branch>` no longer matched the commit.

That tripwire fired in practice: eProsima advanced `Fast-DDS 3.x` (`dff5a82…` → `9a31251…`) and the
pinned build broke (Hermes High, head `8b85069`). A moving-branch pin is inherently
non-reproducible — the superbuild fetch result depends on *when* you build — so the tripwire only
converts silent drift into a recurring manual re-resolve.

**Decision:** pin all four transitive deps to **immutable tags** matching the agent's declared EXACT
dep versions (`Fast-DDS 3.1` → latest `v3.1.x` = `v3.1.3`; `Fast-CDR 2.2.4` → `v2.2.4`;
foonathan_memory `v0.7-3` and spdlog `v1.9.2` are already tags). `build_xrce_agent.sh` rewrites the
agent's `_fastdds_tag`/`_fastcdr_tag` to the manifest refs **before configuring** (fail-closed if the
upstream `set(_<dep>_tag …)` line is absent), so the superbuild fetches exactly the pinned commit —
reproducibly. The pre-build ls-remote gate and post-build checkout gate are retained as
supply-chain checks (they now catch a *force-pushed/retagged* upstream, not routine branch motion).

A standalone `scripts/check_xrce_pins.py` runs the same ls-remote resolution network-only (no clone /
compile) as a PR-CI job (`xrce-pins` in `python-quality.yml`), so a drifted or tampered pin fails the
PR rather than surfacing only in the nightly reviewer. A fast unit guard
(`tests/unit/test_xrce_pins.py`) asserts every `[bridge]` transitive ref stays an immutable tag.

**Re-pin procedure when bumping `uxrce_dds_agent_version`:** read the new agent tag's
`CMakeLists.txt` for its declared `_fastdds_tag`/`_fastcdr_tag`/versions, pick the matching immutable
dep tags, capture each tag's commit (`git ls-remote <repo> <tag>`), and update all eight `[bridge]`
ref/commit fields. `check_xrce_pins.py` then verifies the capture.

## Consequences

### Positive
- The bridge actually installs — the M2 exit criterion (live `/fmu/*`) is reachable on the host
  and in the container, by the same pinned source build.
- The transitive superbuild fetch is now **reproducible** (immutable tags), and pin drift is caught
  in PR CI, not just by the nightly reviewer.
- Reproducible and supply-chain-consistent with ADR-0005/0006: a versioned, commit-verified
  source pin rather than an unpinned (and non-existent) apt name.

### Negative
- Slower than an apt install: the agent's cmake superbuild adds a few minutes to the sim image
  build and to a host bootstrap. One-time (Docker-layer-cached for the image).

### Neutral
- The agent version is independent of the PX4/`px4_msgs` pin (OQ-3); the agent is
  version-tolerant of the PX4-bundled client. Bumping it is a manifest two-line edit
  (`uxrce_dds_agent_version` + `uxrce_dds_agent_commit`).

## Alternatives considered
- **`snap install micro-xrce-dds-agent`.** Exists, but snap inside the sim container is awkward
  and pins less precisely; rejected.
- **Wait for an apt package.** None is published for Jazzy; not an option for M2.
