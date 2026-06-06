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

## Consequences

### Positive
- The bridge actually installs — the M2 exit criterion (live `/fmu/*`) is reachable on the host
  and in the container, by the same pinned source build.
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
