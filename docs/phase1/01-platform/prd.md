# Platform & Simulation Foundation — Phase 1 Docset 01

> **One-liner:** Stand up the pinned, containerized PX4-SITL + Gazebo-Harmonic + ROS 2 Jazzy substrate — with live `/fmu/*` telemetry over the native uXRCE-DDS bridge and a single `colcon build` that succeeds inside the container — so every other Phase 1 docset builds on a reproducible foundation instead of a setup problem.

**Date:** 2026-06-03
**Status:** Draft (rev 2 — ReviewPRD auto-revise pass applied)
**Owner:** Project owner (solo dev)
**DRI:** jxstanford@wemodulate.energy

## Overview

This docset delivers the pre-hardware platform that all of Phase 1 stands on: the pinned toolchain (Ubuntu 24.04, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, Python 3.12), PX4 SITL flying the vanilla `gz_x500` airframe, the native uXRCE-DDS bridge exposing PX4 as ROS 2 topics, the two-container (`sim`/`dev`) Docker build, a clean `ros2_ws` colcon workspace with vendored `px4_msgs`, and the integrative setup-to-running-mission README spine. It covers milestones M1 (toolchain + vanilla SITL flying) and M2 (ROS 2 Jazzy + uXRCE-DDS bridge), and owns Phase 1 exit-checklist item 9 (containerized build + `colcon build` succeeds) plus the integrative item 10 (README in <20 commands).

This is infrastructure, not a user-facing feature. Its value is entirely downstream: docsets 02 (mission control), 03 (sim environment), 04 (perception), and 05 (logging/replay) all consume this base workspace, container layout, and telemetry surface, and every later hardware phase (2–8) reuses the container definitions on Jetson. The DoD's settled constraints (ADR-0001 distro/OS, ADR-0002 CI, uXRCE-DDS-not-MAVROS, Gazebo Harmonic, vendored `px4_msgs`) are inputs to this PRD, not open questions — they are not relitigated here.

## Problem Statement

> **When** a solo developer (or a collaborator on a modest workstation) starts Phase 1 of the drone project,
> **they struggle with** standing up an early-adopter PX4-on-Jazzy stack (PX4 + Gazebo Harmonic + ROS 2 + uXRCE-DDS + Docker) where any one of a dozen integration points can silently break,
> **which means** that a broken Gazebo plugin, an unpinned message dependency, or a dead agent↔PX4 link gets discovered three milestones later — tangled with mission-logic bugs — instead of caught at the foundation.

Today there is no reproducible substrate: a fresh checkout does not fly a drone, does not publish ROS 2 telemetry, and does not build. The plan itself flags ~1 week of expected M1–M2 integration friction on the deliberately-chosen 24.04/Jazzy combination. Without a pinned, containerized foundation, every collaborator re-derives the setup by hand ("works on my machine"), the agent↔PX4 bridge — the single most common failure point at this stage — has no known-good reference state, and downstream docsets 02–05 have no workspace to land their packages in. This matters now because hardware purchase is gated on Phase 1's exit checklist, and the discipline of the whole project is "shake out in sim what you'd otherwise debug on a vibrating airframe."

## Goals

### Business goals
- Unblock Phase 1 docsets 02–05 and the eventual hardware-purchase decision by delivering a green, reproducible simulation foundation.
- Eliminate environment-skew rework (the "works on my machine" class of bug) across collaborators now, and between dev hosts and Jetson later, by aligning on a single containerized stack.

### User goals
- A developer can go from `git clone` to a flying SITL drone publishing live ROS 2 telemetry by following the README, with no tribal knowledge.
- A node author can subscribe to native PX4 `/fmu/out/*` topics and compile against stable, vendored `px4_msgs` without a MAVLink translation layer and without version drift breaking a green build.
- A collaborator on a modest workstation reproduces the exact environment via `docker compose` without manual host setup.

### Non-goals
- Mission logic, the state machine, and launch entrypoints — owned by 02-mission-control (M3–M4).
- Custom worlds, checkpoints, AprilTag models, and the camera topic — owned by 03-sim-environment (M5).
- Perception nodes and the `CheckpointCapture` message contents — owned by 04-perception (M6); 01 provides only the empty `patrol_interfaces` package shell.
- Bag recording, DGX upload, Foxglove, and the `ingest` service implementation — owned by 05-logging-replay (M7–M8); 01 provides only the container slot.

> Brief non-goals above are orientation only. The contract-level deferral commitments — with rationale and target — are in the Out of Scope table below.

## Out of Scope

> Items explicitly **not** part of this docset. Each has a status, rationale, and target. Listing them here (rather than omitting them) prevents scope creep and tells reviewers exactly what this PRD does NOT authorize.

| Item | Status | Rationale | Target | Added |
|------|--------|-----------|--------|-------|
| Mission logic / state machine / launch entrypoints | Deferred | Belongs to the mission-control docset, not the platform; platform provides the workspace and `/fmu/in` command surface, not the mission that drives it | 02-mission-control (M3–M4) | 2026-06-03 |
| Custom worlds, checkpoints, AprilTag models, camera topic | Deferred | Platform ships only the vanilla `gz_x500` baseline; world assets are a distinct ownership boundary | 03-sim-environment (M5) | 2026-06-03 |
| Perception nodes + `CheckpointCapture` message contents | Deferred | Platform provides the empty `patrol_interfaces` package shell as a landing slot; its contents are owned downstream | 04-perception (M6) | 2026-06-03 |
| Bag recording, DGX upload, Foxglove, `ingest` service implementation | Deferred | Platform provides the `docker/ingest/` container slot only; the recording/replay pipeline is a separate docset | 05-logging-replay (M7–M8) | 2026-06-03 |
| VIO / SLAM | Deferred | Sim provides ground-truth pose; no localizer needed in SITL. Would re-enter when real sensors arrive | Phase 3/4 | 2026-06-03 |
| EKF2 / VIO parameter tuning; MAVROS-vs-DDS re-evaluation | Deferred | The uXRCE-DDS call is settled (ADR-0001); parameter work needs real sensors | Phase 3/4 | 2026-06-03 |
| Isaac Sim | Out of scope | Gazebo Harmonic is sufficient for Phase 1; Isaac is a later-phase fidelity tool | Phase 5/8 | 2026-06-03 |
| Foxglove / QGroundControl containerization | Out of scope | They are desktop apps and stay desktop apps (plan "Containerization") | N/A | 2026-06-03 |
| WSL2 support | Out of scope | GPU/USB/real-time handoff is awkward and won't carry to hardware-in-the-loop; native Linux only (plan "Dev hardware requirements") | N/A | 2026-06-03 |

## Key Hypotheses

- **H1:** We believe pinning every toolchain layer (OS, ROS, PX4, Gazebo, bridge, Python, colcon, Docker) in a single manifest and containerizing the build will eliminate the "works on my machine" failure class, because version skew between hosts is the dominant source of foundation-stage breakage on an early-adopter stack.
  *Signal: a second clean host (or CI runner) reproduces a green `docker compose` build + `colcon build` from the manifest with zero manual host edits.*
- **H2:** We believe the PX4 + Jazzy + Gazebo Harmonic + uXRCE-DDS combination integrates end-to-end with roughly one week of friction (early-adopter, not pioneer), because this combination is reported working in the PX4/ROS 2 community — a claim treated as a **research target the M1–M2 integration spike confirms**, not as a settled fact.
  *Signal: M1 and M2 exit criteria pass within the planned M1–M2 window; the agent↔PX4 link reaches a steady ~50 Hz on `/fmu/out/vehicle_local_position`. If the spike overruns the window, the assumption is falsified and the friction estimate (and OQ-3) is re-opened.*
- **H3:** We believe vendoring and version-pinning `px4_msgs` (rather than pulling it at build time) will prevent message-definition drift from breaking previously-green builds, because PX4 message definitions change between versions and v1.16 message versioning insulates the workspace from the firmware.
  *Signal: re-running `colcon build` after an unrelated upstream PX4 change still succeeds because the vendored, pinned copy is unchanged.*

## Tenets

> Tie-breakers when implementation choices are ambiguous — *unless you know better ones.*

1. **Reproducibility over convenience** — when a faster local shortcut would diverge from the pinned/containerized path, take the pinned path; the README and container are the source of truth.
2. **Catch it at the foundation** — if a failure can be surfaced in M1/M2 sim, surface it here rather than letting it hide until a downstream milestone.
3. **Own the slot, not the contents** — for package shells and container slots that siblings fill, deliver the empty, building landing slot and stop; do not pre-build a sibling's contents.
4. **Pin, don't track** — prefer an explicit pinned version over "latest" everywhere a version is referenced; sliding off latest mid-project costs days.
5. **Native ROS 2, no translation layer** — expose PX4 as native `/fmu/*` topics via uXRCE-DDS; never reach for a MAVLink bridge as a shortcut.

## Functional Requirements

> No REST endpoints or SDK module paths are in scope for this docset — the "surface" is ROS 2 topics, container/build commands, and repo layout. The Path & SDK conventions subsection is therefore omitted (N/A). Topic and command names below follow the DoD's Interfaces section verbatim.

### P1: Critical (must ship)

#### PLAT-1: Vanilla SITL flight in Gazebo Harmonic
The system SHALL provide a pinned toolchain on which `make px4_sitl gz_x500` cleanly launches a drone in Gazebo Harmonic that arms, takes off via QGroundControl, and holds altitude for at least 60 continuous seconds.

**Customer scenario:** the operator validates that the pinned toolchain installs and runs before any architecture is layered on top.

**Pain removed:** a broken Gazebo-plugin or Python-dependency install that would otherwise be discovered three milestones later, tangled with mission-logic bugs, instead of caught at the foundation.

**Acceptance criteria:**
- `make px4_sitl gz_x500` launches a drone in Gazebo Harmonic with no plugin or dependency errors on a clean Ubuntu 24.04 host with the toolchain installed.
- The drone arms and takes off when commanded from QGroundControl (run separately, desktop).
- The drone holds altitude within hover tolerance for at least 60 continuous seconds without manual correction.

**Trace:** UAC-PLAT-1 (Appendix B)

#### PLAT-2: Live PX4 telemetry as native ROS 2 topics over uXRCE-DDS
The system SHALL run the Micro XRCE-DDS Agent bridging PX4 to ROS 2 over UDP localhost such that `ros2 topic list | grep fmu` returns PX4 topics and `/fmu/out/vehicle_local_position` publishes at a steady ~50 Hz (the PX4 SITL default for that topic).

**Customer scenario:** a downstream node author subscribes to native PX4 ROS 2 topics with no MAVLink translation layer in between.

**Pain removed:** the agent↔PX4 connection — the single most common failure point at this stage — is proven in SITL so "working" is a known, reproducible state for every sibling docset, rather than each docset rediscovering it.

**Acceptance criteria:**
- With SITL running and the agent bridged, `ros2 topic list | grep fmu` returns PX4 topics.
- `ros2 topic hz /fmu/out/vehicle_local_position` reports a steady publish rate of ~50 Hz (the PX4 default for this topic) sustained over a 60 s observation window.
- The `/fmu/in/*` command surface is present and addressable (for downstream offboard control by 02).

**Trace:** UAC-PLAT-2 (Appendix B)

#### PLAT-3: Reproducible containerized build (`sim` + `dev`)
The system SHALL provide `sim` and `dev` containers built from a shared Ubuntu 24.04 + ROS 2 Jazzy base, orchestrated by `docker compose`, such that both build successfully from a clean checkout.

**Customer scenario:** a collaborator on a modest workstation reproduces the exact environment via `docker compose` without manual host setup.

**Pain removed:** environment skew between dev hosts (and later between dev hosts and the Jetson) that silently breaks builds with no functional cause.

**Acceptance criteria:**
- `docker compose` builds both the `dev` and `sim` containers from the shared base with no errors.
- The `sim` container contains PX4 SITL + Gazebo Harmonic + the uXRCE-DDS agent + the workspace; the `dev` container shares the base, adds tooling, and mounts source as a volume.
- The container definitions contain no x86-host-specific assumptions in their build steps, and the container runtime (CPU vs NVIDIA Container Runtime) is a swappable parameter — so the same definitions are the starting point for the Jetson image without rewriting the Dockerfiles. (Jetson validation itself is Phase 2, out of scope here.)

**Trace:** UAC-PLAT-3 (Appendix B)

#### PLAT-4: Single `colcon build` succeeds inside the container
The system SHALL provide a clean `ros2_ws` colcon workspace in which a single `colcon build` succeeds with no errors inside the container.

**Customer scenario:** a node author (sibling docset) runs one build command inside the container and gets a green workspace they can land their package into.

**Pain removed:** a foundation that doesn't build cleanly, forcing every downstream author to debug the base workspace before they can start their own work.

**Acceptance criteria:**
- A single `colcon build` against `ros2_ws` inside a built container completes with no errors.
- The build is green on CI Layer B (`colcon build` / `colcon test`) per ADR-0002.
- The empty-but-present `patrol_interfaces` and `patrol_bringup` package shells build as part of the workspace.

**Trace:** UAC-PLAT-4 (Appendix B)

#### PLAT-5: Vendored, version-pinned `px4_msgs`
The system SHALL vendor `px4_msgs` into `ros2_ws/src/external/px4_msgs`, pinned to the chosen PX4 branch (not pulled at build time), building as part of the workspace.

**Customer scenario:** a node author gets stable message definitions that don't drift out from under a known-good build.

**Pain removed:** an unpinned message dependency breaking a previously-green build for no functional reason.

**Acceptance criteria:**
- `px4_msgs` is present under `ros2_ws/src/external/px4_msgs`, vendored (committed), not fetched at build time.
- It is pinned to the chosen PX4 branch/tag and builds as part of the `colcon build`.
- `px4_ros_com` helpers/examples are likewise vendored under `ros2_ws/src/external/`.

**Trace:** UAC-PLAT-5 (Appendix B)

#### PLAT-6: Setup-to-running-mission README (≤20 commands)
The system SHALL provide a top-level README documenting the end-to-end path from a clean machine to a running mission, executable in fewer than 20 commands.

**Customer scenario:** a new collaborator goes from `git clone` to a running simulation by following the README only, with no undocumented steps.

**Pain removed:** tribal-knowledge onboarding and undocumented setup steps that make the environment irreproducible.

**Acceptance criteria:**
- The README documents the full setup-to-running-mission path; following it on a clean machine reaches a running simulation with no steps not in the README.
- The platform bring-up spine fits within the shared ≤20-command budget, leaving room for siblings to append their run steps (budget allocation tracked in OQ-6).
- The README is the integrative bring-up document of record; siblings append, but the spine is owned here.

**Trace:** UAC-PLAT-6 (Appendix B)

#### PLAT-7: Pinned stack manifest (single source of truth)
The system SHALL provide a single manifest that explicitly pins every toolchain layer: OS, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, uXRCE-DDS, Python 3.12, the MCAP storage plugin, colcon, and Docker.

**Customer scenario:** the team avoids days lost to sliding off "latest" mid-project; any layer's version is looked up in one place.

**Pain removed:** version drift across collaborators and CI that produces inconsistent, hard-to-diagnose behavior.

**Acceptance criteria:**
- Every toolchain layer referenced anywhere in the build is pinned to an explicit version in the manifest.
- The manifest is the cited source of truth referenced by the README and container definitions.

**Trace:** UAC-PLAT-7 (Appendix B)

#### PLAT-8: Workspace package shells as landing slots
The system SHALL provide empty-but-present `patrol_bringup` and `patrol_interfaces` package shells in `ros2_ws/src/` that build cleanly, as the landing slots for sibling docsets.

**Customer scenario:** docsets 02 and 04 have a buildable package to land mission launch files and the `CheckpointCapture` message into, with no rebuild-the-world step.

**Pain removed:** siblings having to create and wire up new packages from scratch (and the cross-cutting rebuild that causes) before they can contribute.

**Acceptance criteria:**
- `patrol_bringup` and `patrol_interfaces` package shells exist in `ros2_ws/src/` and build as part of `colcon build`.
- The shells are empty of contents (their contents are owned by 02 and 04 respectively) but are valid ROS 2 packages.

**Trace:** UAC-PLAT-8 (Appendix B)

### P2: Important (should ship)

#### PLAT-9: Headless-capable `sim` container for CI
The system SHALL provide a `sim` container that runs headless (no display) and supplies a working simulation environment for the integration-test tier owned by siblings.

**Customer scenario:** the integration tier (owned by 02/05) spins up SITL in CI without a display attached.

**Acceptance criteria:**
- The `sim` container runs headless and produces a working simulation environment when started without a display.
- The platform build does not block CI; the integration tests that exercise the environment are owned by 02/05, not here.

## Scope Authority

> The FR table above is the **contract** for this PRD. The design document (`docs/phase1/01-platform/design.md` — to be created via /drive) realizes these FRs as components, container/workspace layout, and milestone tasks.
>
> **The design must not introduce surface area beyond this PRD's FR table without a corresponding PRD revision.** If the design proposes a new container service, topic surface, package, or pinned-stack entry not authorized by an FR, the PRD must be updated first.
>
> Conversely, **this PRD must not specify implementation detail beyond the FR shape.** Dockerfile layering, exact compose service decomposition, the specific PX4 tag, threading/transport internals, and rendering-backend choices belong in the design (several are flagged as Open Questions below), not here.
>
> This discipline keeps the design honest and the PRD lean.

## Success Metrics

| Metric | Baseline (current) | Target | How Measured | Timeline |
|--------|-------------------|--------|--------------|----------|
| Clean-host reproducibility (User) | N/A (new — no foundation exists) | A second clean host or CI runner reaches green `docker compose` build + `colcon build` with 0 manual host edits | Run the README path on a fresh host / CI runner; count manual deviations | End of M2 |
| `/fmu/out/vehicle_local_position` publish rate (Technical) | N/A (new) | Steady ~50 Hz sustained over a 60 s SITL run | `ros2 topic hz /fmu/out/vehicle_local_position` | M2 exit |
| README command count, setup-to-running-mission (User) | N/A (new) | < 20 commands end-to-end (platform spine fits within shared budget) | Count executable commands in the README path | M2 exit / item 10 |
| Container + workspace build success (Technical) | N/A (new) | `docker compose` build of `sim`+`dev` and a single `colcon build` both succeed; green on CI Layer B | CI Layer B job result; local `docker compose` + `colcon build` | Every PR (M1–M2) |
| M1–M2 integration-friction window (Business) | N/A (new) | M1 and M2 exit criteria reached within the planned ~1-week M1–M2 friction window | Calendar tracking against milestone exits | M2 exit |

## Technical Considerations

### Integration points
- **PX4-Autopilot upstream** — SITL, the `gz_x500` airframe target, and the bundled `uxrce_dds_client`.
- **ROS 2 Jazzy + Micro XRCE-DDS Agent** — the agent bridges PX4↔ROS 2 over UDP localhost; in SITL the client transport is automatic.
- **Gazebo Harmonic (gz-sim 8)** — Vulkan rendering; the simulator PX4 SITL launches.
- **QGroundControl** — desktop app for M1 manual arm/takeoff verification; not containerized.
- **CI two-layer architecture (ADR-0002)** — Layer B runs `colcon build`/`colcon test`; the SITL tier is a deferred nightly scaffold, never a required per-PR check. The platform build must be green on Layer B.

### Data storage
- No persistent application data is owned by this docset. The repo carries the vendored `px4_msgs`/`px4_ros_com` source, container definitions, the pinned manifest, and the README. Bag/manifest storage is owned by 05.

### Scalability
- Single simulated drone in a simple world is light (a single 4 GB-class discrete GPU is sufficient per the dev-hardware table). No multi-node or multi-drone scaling is in scope.

### Rabbit holes
> Things that look simple but could explode in scope. Flagged early.

- **Headless Vulkan rendering for Gazebo Harmonic in CI** — software/llvmpipe vs hosted-runner GPU is the plan's least-confident area; contain by keeping SITL CI as a nightly scaffold (ADR-0002), not a required per-PR gate (OQ-4).
- **PX4-from-source vs prebuilt SITL artifact in the `sim` image** — building PX4 from source is reproducible but slow; a prebuilt artifact is fast but adds a maintenance surface. Trades image build time against CI runtime budget (OQ-2). Contain by deciding in the design, not by trying both.
- **GPU passthrough to containers on heterogeneous dev hardware** — collaborator GPUs range 4–8 GB; passthrough must stay optional and must not become a hard requirement (OQ-5).
- **The exact PX4 / `px4_msgs` pin** — picking a v1.16.x tag with a matching `px4_msgs` branch is the kind of thing that looks like a one-line decision and turns into the ~1-week integration spike; contain by treating the spike as the explicit mechanism that resolves OQ-3.

### Potential challenges
- The early-adopter PX4-on-Jazzy combination is the highest-risk integration in Phase 1; it is reversible and sim-only (no hardware, no data at stake), so the mitigation is the M1–M2 spike, not a heavier process.
- The agent↔PX4 bridge is the most common failure point; PLAT-2's steady-rate acceptance criterion is the explicit guard.

## Performance Requirements

### Latency / rate targets

| Operation | Target | Gating? |
|-----------|--------|---------|
| `/fmu/out/vehicle_local_position` publish rate | Steady ~50 Hz sustained over 60 s | Yes (M2 exit, PLAT-2) |
| SITL hover stability window | ≥ 60 s continuous altitude hold within hover tolerance | Yes (M1 exit, PLAT-1) |
| `docker compose` build of `sim`+`dev` | Completes successfully on a reference dev host (8-core / 32 GB); wall-clock budget set in the design | No (track; not gating) |
| `colcon build` (clean workspace, in-container) | Completes successfully | Yes (item 9, PLAT-4) |

### Throughput
- Single drone, single simulation process group. No concurrent-user or batch throughput requirements.

### Resource constraints
- Minimum: modern 6-core CPU, 16 GB RAM, 4 GB discrete GPU (Vulkan), 256 GB NVMe. Comfortable: 8+ cores, 32 GB, 8 GB GPU, 1 TB NVMe (per the plan's dev-hardware table). Native Linux only; WSL2 unsupported.

### Optimization approach
- Headless mode for CI to drop the rendering cost; GPU passthrough optional for interactive dev (OQ-4, OQ-5). PX4 source-vs-prebuilt in the image is the build-time lever (OQ-2).

## Observability

> Lightweight for a sim foundation — the "alerts" here are developer-facing verification commands and CI signals, not a production monitoring stack.

### Verification signals

| Signal | What it confirms | Healthy condition |
|--------|------------------|-------------------|
| `ros2 topic list \| grep fmu` | uXRCE-DDS agent is bridged and PX4 topics exist | PX4 topics returned (PLAT-2) |
| `ros2 topic hz /fmu/out/vehicle_local_position` | Telemetry is flowing at the expected rate | Steady ~50 Hz over 60 s (PLAT-2) |
| `docker compose` build exit code | Containers build from the shared base | 0 / success (PLAT-3) |
| `colcon build` exit code (in-container) | Workspace is green | 0 / success (PLAT-4) |
| CI Layer B job status | Platform build is green within the agreed CI architecture | Passing on each PR (ADR-0002) |

### Logging
- Standard tool output (PX4 SITL console, Gazebo, the uXRCE-DDS agent, colcon) is the diagnostic surface. No custom structured logging is introduced by this docset.

## Operational readiness

> "Operations" here is the developer running the stack daily and onboarding collaborators, not a production on-call rotation.

### Runbook

| Scenario | Detection | Response |
|----------|-----------|----------|
| SITL launches but no `/fmu/*` topics | `ros2 topic list \| grep fmu` returns nothing | Confirm the Micro XRCE-DDS Agent is running and `uxrce_dds_client` is up on UDP localhost; restart the agent; this is the known #1 failure point (PLAT-2) |
| `colcon build` fails on `px4_msgs` | colcon error referencing `px4_msgs` | Confirm the vendored pin under `ros2_ws/src/external/px4_msgs` matches the chosen PX4 branch (PLAT-5, OQ-3) |
| `docker compose` build fails | Non-zero build exit | Re-check pinned-manifest versions (PLAT-7); confirm host has the base prerequisites the README lists |
| Gazebo won't render in CI / headless host | Gazebo startup failure with no display | Use the headless `sim` path (PLAT-9); GPU/rendering-backend choice tracked in OQ-4 |

### Deployment checklist (foundation bring-up)
- [ ] Pinned manifest complete and cited by README + containers (PLAT-7)
- [ ] `docker compose` builds `sim` + `dev` from a clean checkout (PLAT-3)
- [ ] `colcon build` green in-container and on CI Layer B (PLAT-4)
- [ ] `make px4_sitl gz_x500` flies + holds altitude 60 s (PLAT-1)
- [ ] `/fmu/*` topics live at ~50 Hz (PLAT-2)
- [ ] README path executes in <20 commands on a clean host (PLAT-6)

### Dependencies and SLAs

| Dependency | Degraded behavior if unavailable | Owner |
|------------|----------------------------------|-------|
| PX4-Autopilot upstream | No SITL / no `gz_x500` target — foundation cannot fly | PX4 (OSS upstream) |
| ROS 2 Jazzy + Micro XRCE-DDS Agent | No bridge — no ROS 2 telemetry | OSS upstream |
| Gazebo Harmonic | No simulator — nothing to fly | OSS upstream |

## Milestones

### M1: Toolchain installed, vanilla SITL flying
- Pinned toolchain installed on Ubuntu 24.04; PX4-Autopilot built from source; `gz_x500` target launching in Gazebo Harmonic.
- Pinned stack manifest drafted (PLAT-7); container scaffolding begun.
- **Validation:** a stakeholder runs `make px4_sitl gz_x500`, arms and takes off the drone from QGroundControl, and watches it hold altitude for 60 continuous seconds (PLAT-1).

### M2: ROS 2 Jazzy + uXRCE-DDS bridge + containerized green build
- ROS 2 Jazzy installed; vendored `px4_msgs`/`px4_ros_com` built in `ros2_ws` (PLAT-5); Micro XRCE-DDS Agent bridging PX4↔ROS 2 (PLAT-2).
- `sim` + `dev` containers building from `docker compose` (PLAT-3, PLAT-9); single `colcon build` green in-container and on CI Layer B (PLAT-4); `patrol_bringup`/`patrol_interfaces` shells present (PLAT-8); README spine complete (PLAT-6); manifest finalized (PLAT-7).
- **Validation:** a stakeholder, with SITL running in the container, sees `ros2 topic list | grep fmu` return PX4 topics and `ros2 topic hz /fmu/out/vehicle_local_position` report ~50 Hz over a 60 s window; then follows the README on a clean host to reach a running simulation in <20 commands.

## Open Questions

| # | Question | Status | Decision target | Rationale (why open / what would resolve it) |
|---|----------|--------|-----------------|----------------------------------------------|
| OQ-1 | Is the bare two-container (`sim`/`dev`) split sufficient, or does the uXRCE-DDS agent warrant its own compose service? | Resolved (ratified 2026-06-03) | M2 design phase | The plan calls two-container "a defensible default, not the only answer" and welcomes pushback; resolved by the design's compose decomposition. |
| OQ-2 | Build PX4-Autopilot from source inside the `sim` image, or layer a prebuilt SITL artifact for faster CI? | Resolved (ratified 2026-06-03) | M2 design phase | Trades image build time / reproducibility against CI runtime budget; flagged low-confidence in the plan's test strategy. |
| OQ-3 | Which exact PX4 v1.16.x tag and matching `px4_msgs` branch to vendor? | Deferred | End of M1–M2 integration spike | Needs the early-adopter spike (the planned ~1-week M1–M2 friction) to settle on a known-good combination; cannot be picked on paper. Also the falsification gate for H2. |
| OQ-4 | Headless rendering backend for CI — software/llvmpipe vs hosted-runner GPU for Gazebo Harmonic's Vulkan rendering? | Resolved (ratified 2026-06-03) | M2 design phase | The plan notes SITL CI orchestration is its least-confident area; rendering on hosted runners is the practical risk. SITL CI stays a nightly scaffold (ADR-0002) until resolved. |
| OQ-5 | How (and whether) to expose the host GPU to Gazebo in the container for interactive dev? | Resolved (ratified 2026-06-03) | M2 design phase | Varies by collaborator hardware (4–8 GB discrete GPUs); must remain optional and not become a hard requirement. |
| OQ-6 | How is the ≤20-command README budget split between the platform bring-up spine and the per-docset run steps siblings append? | Open | M2 exit (integrative, item 10) | Item 10 spans all docsets; platform owns the spine but the total must stay under budget — needs sibling run-step counts to allocate. |
| OQ-7 | Checkpoint mapping schema (`sim/config/checkpoints.yaml`: list of `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`). **Provisional default applied confirmed at combined review (2026-06-03).** Owned by 03; 01 is not an owner but the `gz_x500` baseline + ENU/world-frame convention must stay compatible. | Provisional | Joint /drive review (03 owns; 01 affected via world-frame convention) | Cross-docset contract (a) — owned by 03, consumed by 02+04. Recorded here as a flagged default so the 5 pairs stay coherent; 01 must not contradict the world/ENU frame the positions are expressed in. Needs human confirmation. |
| OQ-8 | `CheckpointCapture` representation — `string image_path` + header/checkpoint_id/pose/metadata by reference, NOT pixels by-value; a separate `sensor_msgs/CompressedImage` topic carries live frames for the bag. **Provisional default applied confirmed at combined review (2026-06-03).** Owned by 04; lands in 01's `patrol_interfaces` shell. | Provisional | Joint /drive review (04 owns; 01 provides the package shell) | Cross-docset contract (b) — owned by 04, consumed by 05. 01 owns only the empty `patrol_interfaces` landing slot, not the message contents, but the slot must accept it. Recorded as a flagged default; needs human confirmation. |

> No owner column by design. OQ-3 is gated on the integration spike (an internal action, not an external blocker). OQ-7/OQ-8 are cross-docset contracts owned by sibling docsets (03/04); they are recorded here as provisional flagged defaults because the human does a combined review of all 5 pairs — 01 does not own them and must not silently invent the contract, but it must stay compatible with them.

## Appendix B: User Acceptance Criteria

> Every P1 FR has a corresponding UAC in Given/When/Then form. UAC IDs match the FR ID.

### UAC-PLAT-1: Vanilla SITL flight in Gazebo Harmonic
**GIVEN** a clean Ubuntu 24.04 host with the pinned toolchain installed
**WHEN** the operator runs `make px4_sitl gz_x500` and arms/takes off from QGroundControl
**THEN** a drone launches in Gazebo Harmonic, takes off, and holds altitude within hover tolerance for at least 60 continuous seconds with no plugin or dependency errors.

### UAC-PLAT-2: Live PX4 telemetry over uXRCE-DDS
**GIVEN** SITL is running with the Micro XRCE-DDS Agent bridged to PX4 over UDP localhost
**WHEN** a node author runs `ros2 topic list | grep fmu` and `ros2 topic hz /fmu/out/vehicle_local_position`
**THEN** PX4 topics are returned and `/fmu/out/vehicle_local_position` reports a steady ~50 Hz publish rate sustained over a 60 s window, and the `/fmu/in/*` command surface is addressable.

### UAC-PLAT-3: Reproducible containerized build
**GIVEN** a clean checkout of the repo on a collaborator's workstation
**WHEN** they run `docker compose` to build the `dev` and `sim` containers
**THEN** both build successfully from the shared Ubuntu 24.04 + ROS 2 Jazzy base with no manual host edits, with `sim` carrying SITL+Gazebo+agent+workspace and `dev` mounting source as a volume, and with no x86-host-specific assumptions baked into the build steps.

### UAC-PLAT-4: Single `colcon build` succeeds inside the container
**GIVEN** a built container with the `ros2_ws` workspace
**WHEN** a node author runs a single `colcon build` inside it
**THEN** the build succeeds with no errors (including the `patrol_interfaces`/`patrol_bringup` shells) and the same build is green on CI Layer B.

### UAC-PLAT-5: Vendored, version-pinned `px4_msgs`
**GIVEN** the workspace
**WHEN** it is built
**THEN** `px4_msgs` is present under `ros2_ws/src/external/px4_msgs`, vendored and pinned to the chosen PX4 branch (not pulled at build time), and builds as part of the workspace alongside vendored `px4_ros_com`.

### UAC-PLAT-6: Setup-to-running-mission README (≤20 commands)
**GIVEN** a collaborator on a clean machine
**WHEN** they follow the README from setup to a running mission
**THEN** the path is fully documented and executable in fewer than 20 commands, with the platform bring-up spine fitting within the shared budget and no step required that is not in the README.

### UAC-PLAT-7: Pinned stack manifest
**GIVEN** the pinned stack manifest
**WHEN** any toolchain layer is referenced (OS, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, uXRCE-DDS, Python 3.12, MCAP plugin, colcon, Docker)
**THEN** its version is explicitly pinned in the manifest and the manifest is the cited source of truth.

### UAC-PLAT-8: Workspace package shells as landing slots
**GIVEN** the `ros2_ws` workspace
**WHEN** it is built
**THEN** the empty-but-present `patrol_bringup` and `patrol_interfaces` package shells exist in `ros2_ws/src/` and build as part of `colcon build`, with their contents left to be owned by 02 and 04.

---

## Review Trail (ReviewPRD auto-pilot)

> Recorded for the combined human review. Tier classified as **Standard** per DoD §8 assessment signals.

**Pass 1 review — dimension scores:** D1 Strong · D2 Strong · D3 Strong · D4 Strong · D5 Strong · D6 N/A (no API/SDK surface) · D7 Strong · D8 Strong · **D9 Adequate→fixed** · D10 Strong · D11 Strong · D12 Strong · **D13 Adequate→fixed**.

**Medium findings resolved in revise pass (rev 1 → rev 2):**
- **D9 (Hypothesis verifiability):** H2's rationale "the community has demonstrated this exact stack working" was an unverified external-system claim asserted as fact. Reframed as a research target the M1–M2 spike confirms, with an explicit falsification condition, and tied to OQ-3.
- **D13 (Requirement smell — vague qualifier in a P1 AC):** PLAT-3 AC3 "structured so the same definitions largely carry to Jetson" used the non-observable comparative "largely carry." Rewritten to the observable condition (no x86-host-specific assumptions in build steps; runtime is a swappable parameter), and cascaded into UAC-PLAT-3.
- **D13 (Requirement smell — loophole in a P1 AC):** PLAT-2 AC "reports a steady rate, typically ~50 Hz" used the loophole "typically." Rewritten to "~50 Hz (the PX4 default for this topic) sustained over a 60 s observation window," and cascaded into UAC-PLAT-2, the Performance Requirements table, the Observability table, and the Success Metrics row for consistency.

**Pass 2 self-review:** clean at the medium floor — no Weak/Missing dimensions and no Critical/High priority actions outstanding. Residual minor smells ("stably"/"hover tolerance" in PLAT-1, "reference dev host" in the non-gating perf row) are DoD-verbatim or bounded by an adjacent numeric and are left as-is (below the medium floor). See deferred-findings list for the two cross-docset contracts (OQ-7/OQ-8) that remain Provisional by design and need human confirmation.
