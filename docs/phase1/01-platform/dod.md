# Definition of Done — Platform & Simulation Foundation

**Phase 1 docset:** 1 of 5 · **Milestones:** M1–M2
**Lifecycle status:** DoD ✅ · PRD ⏳ (/drive) · Design ⏳ (/drive)
**Source:** docs/phase1_simulation_plan.md — M1 ("Toolchain installed, vanilla SITL flying"), M2 ("ROS 2 Jazzy + uXRCE-DDS bridge"), plus "Target stack (pinned)", "Repo structure", "Containerization", "Dev hardware requirements", "What's explicitly NOT in Phase 1", and the "Phase 1 exit checklist".
**Stakeholders:** Project owner (solo dev) — operator/maintainer who runs the stack daily and onboards collaborators; collaborators on modest dev workstations who must reproduce the environment from scratch; downstream — every sibling docset (02–05) and every later phase (2–8) that builds on this base workspace, container layout, and PX4 telemetry surface; reviewers — the person merging PRs against a `main` that must always work-in-sim.
**Depends on:** none (foundation — this is the first docset; everything else stands on it).
**Consumed by:** 02-mission-control (consumes `/fmu/*` topics, `px4_msgs`, the `ros2_ws` build, the `sim` container, the mission launch entrypoint), 03-sim-environment (extends the `sim` world/airframe baseline), 04-perception (builds nodes in the same workspace), 05-logging-replay (records the same topic surface, extends the `ingest` container); Phase 2+ reuses the container definitions on Jetson with the NVIDIA Container Runtime swapped in.

## 1. Intent

Stand up the full pre-hardware platform — pinned toolchain, PX4 SITL in Gazebo Harmonic, ROS 2 Jazzy with the native uXRCE-DDS bridge, and the two-container (`sim`/`dev`) build — so that a simulated PX4 drone publishes live ROS 2 telemetry and a single `colcon build` succeeds inside a reproducible container. This is the substrate every other Phase 1 docset and every later hardware phase is built on; if the foundation is shaky, every downstream problem is tangled with a setup problem.

## 2. Scope

**In scope:**
- PX4-Autopilot SITL launching Gazebo Harmonic (`gz_x500` airframe), armable and altitude-holding (M1).
- ROS 2 Jazzy install on Ubuntu 24.04 + vendored, version-pinned `px4_msgs` built in `ros2_ws`, and a running Micro XRCE-DDS Agent bridging PX4↔ROS 2 over UDP localhost (M2).
- Live PX4 telemetry visible as native ROS 2 topics (`/fmu/out/*`) at a steady publish rate.
- The `sim` container (PX4 SITL + Gazebo Harmonic + uXRCE-DDS agent + workspace) and the `dev` container (same base + tooling, source mounted as a volume), both built and orchestrated from `docker compose`.
- A clean `ros2_ws` baseline (the empty-but-present `patrol_interfaces` / `patrol_bringup` package shells exist so siblings have somewhere to land) where a single `colcon build` succeeds inside the container.
- The end-to-end setup-to-running-mission README path (≤20 commands), owned here as the integrative environment-bring-up document.
- The pinned version manifest for the whole stack (OS, ROS distro, PX4, Gazebo, bridge, Python, colcon, Docker).

**Out of scope (explicit deferrals — item · rationale · target):**
- Mission logic / state machine · belongs to the mission-control docset, not the platform · 02-mission-control (M3–M4).
- Custom worlds, checkpoints, AprilTag models · the platform ships only the vanilla `gz_x500` baseline · 03-sim-environment (M5).
- Perception nodes and the `CheckpointCapture` message · platform provides the workspace, not the contents · 04-perception (M6).
- Bag recording, DGX upload, Foxglove, the `ingest` service implementation · platform provides the container slot only · 05-logging-replay (M7–M8).
- VIO / SLAM · sim provides ground-truth pose; no localizer needed in SITL · Phase 3/4.
- EKF2 / VIO parameter tuning, MAVROS-vs-DDS re-evaluation · the uXRCE-DDS call is settled; parameter work needs real sensors · Phase 3/4.
- Isaac Sim · Gazebo Harmonic is sufficient for Phase 1 · Phase 5/8.
- Foxglove / QGroundControl containerization · they are desktop apps and stay desktop apps · out of roadmap.
- WSL2 support · GPU/USB/real-time handoff is awkward and won't carry to hardware-in-the-loop · out of roadmap (native Linux only).

## 3. Capabilities (must-do — seeds the PRD's functional requirements)

1. **(P1) Vanilla SITL flight.** `make px4_sitl gz_x500` cleanly launches a drone in Gazebo Harmonic; it arms, takes off via QGroundControl, and hovers stably for 60 seconds.
   - *Customer scenario:* the operator validates that the pinned toolchain installs and runs before any architecture is layered on.
   - *Pain removed:* a broken Gazebo-plugin / Python-dep install discovered three milestones later, tangled with mission-logic bugs, instead of caught at the foundation.
2. **(P1) Live ROS 2 telemetry over uXRCE-DDS.** With SITL running, the Micro XRCE-DDS Agent bridges PX4 to ROS 2; `ros2 topic list | grep fmu` returns PX4 topics and `/fmu/out/vehicle_local_position` publishes at a steady rate (~50 Hz).
   - *Customer scenario:* a downstream node author subscribes to native PX4 ROS 2 topics with no MAVLink translation layer.
   - *Pain removed:* the agent↔PX4 connection (the single most common failure point at this stage) is proven in SITL so "working" is a known, reproducible state for every sibling docset.
3. **(P1) Reproducible containerized build.** `docker compose` builds the `sim` and `dev` containers from the shared Ubuntu 24.04 + ROS 2 Jazzy base, and a single `colcon build` succeeds inside.
   - *Customer scenario:* a collaborator on a modest workstation reproduces the exact environment without "works on my machine" drift.
   - *Pain removed:* environment skew between dev hosts (and later between dev and Jetson) that silently breaks builds.
4. **(P1) Vendored, version-pinned `px4_msgs`.** `px4_msgs` is vendored into `ros2_ws/src/external/` and pinned to the chosen PX4 branch, building as part of the workspace.
   - *Customer scenario:* a node author gets stable message definitions that don't drift out from under a known-good build.
   - *Pain removed:* an unpinned message dependency breaking a previously-green build for no functional reason.
5. **(P1) Setup-to-running-mission README (≤20 commands).** The README documents the end-to-end path from clean machine to a running mission in under 20 commands.
   - *Customer scenario:* a new collaborator goes from `git clone` to a running simulation by following the README only.
   - *Pain removed:* tribal-knowledge onboarding and undocumented setup steps.
6. **(P1) Pinned stack manifest.** A single source of truth pins every layer (OS, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, uXRCE-DDS, Python 3.12, MCAP plugin, colcon, Docker).
   - *Customer scenario:* the team avoids days lost to sliding off "latest" mid-project.
   - *Pain removed:* version drift across collaborators and CI.
7. **(P2) Headless-capable `sim` container for CI.** The `sim` container runs headless so the integration tier (owned by 02/05) can spin it up in CI without a display. *(P2: the platform must not block CI, but the CI integration tests themselves are owned by sibling docsets.)*

## 4. Acceptance criteria / Definition of Done (falsifiable — seeds the PRD's UACs)

Sourced from M1 Exit, M2 Exit, the Containerization section, and exit-checklist items 9 and 10.

- [ ] **AC-1** *(M1 Exit)* GIVEN a clean Ubuntu 24.04 host with the toolchain installed, WHEN `make px4_sitl gz_x500` is run, THEN a drone launches in Gazebo Harmonic, arms and takes off via QGroundControl, and hovers stably for 60 seconds.
- [ ] **AC-2** *(M2 Exit)* GIVEN SITL is running, WHEN `ros2 topic list | grep fmu` is run, THEN PX4 topics are returned.
- [ ] **AC-3** *(M2 Exit)* GIVEN SITL is running with the uXRCE-DDS agent bridged, WHEN `ros2 topic hz /fmu/out/vehicle_local_position` is run, THEN it reports a steady rate (typically ~50 Hz).
- [ ] **AC-4** *(exit-checklist item 9)* GIVEN a checkout of the repo, WHEN `docker compose` builds the `dev` and `sim` containers, THEN both build successfully from the shared Ubuntu 24.04 + ROS 2 Jazzy base.
- [ ] **AC-5** *(exit-checklist item 9)* GIVEN a built container, WHEN a single `colcon build` is run inside it against `ros2_ws`, THEN the build succeeds with no errors.
- [ ] **AC-6** GIVEN the workspace, WHEN it is built, THEN `px4_msgs` is present under `ros2_ws/src/external/`, vendored and pinned to the chosen PX4 branch (not pulled at build time).
- [ ] **AC-7** *(exit-checklist item 10)* GIVEN a collaborator on a clean machine, WHEN they follow the README from setup to a running mission, THEN the path is fully documented and executable in fewer than 20 commands.
- [ ] **AC-8** GIVEN the pinned stack manifest, WHEN any toolchain layer is referenced, THEN its version is explicitly pinned (OS, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, uXRCE-DDS, Python 3.12, colcon, Docker).
- [ ] **AC-9** GIVEN the integration test tier owned by siblings (exit-checklist item 4), WHEN it needs to spin up SITL in CI, THEN the `sim` container runs headless and provides a working simulation environment. *(Platform provides the environment; the test that exercises it is owned by 02/05.)*

## 5. Interfaces

**Owns (contracts this docset defines that others depend on):**
- The PX4↔ROS 2 telemetry surface: native `/fmu/out/*` topics (notably `/fmu/out/vehicle_local_position`) and the `/fmu/in/*` command surface, delivered via the uXRCE-DDS agent — consumed by 02 (offboard control), 05 (recorded topics).
- The vendored package `ros2_ws/src/external/px4_msgs` (and `px4_ros_com` helpers/examples), version-pinned — the message vocabulary every ROS 2 node compiles against.
- The `ros2_ws` colcon workspace layout and build entrypoint (`colcon build`) — the shared build all sibling packages land in.
- The `sim` container (`docker/sim/`) and `dev` container (`docker/dev/`), and the `docker compose` orchestration that brings them up — the runtime every sibling and CI integration test uses.
- The pinned stack manifest (versions of OS/ROS/PX4/Gazebo/bridge/Python/colcon/Docker).
- The top-level README setup-to-running-mission path (integrative; siblings append their own run steps but the bring-up spine is owned here).
- The empty package shells `patrol_bringup` and `patrol_interfaces` exist in the workspace as the landing slots (their *contents* are owned by 02 and 04 respectively).

**Consumes (from other docsets / PX4):**
- PX4-Autopilot upstream (SITL, the `gz_x500` airframe target, the bundled `uxrce_dds_client`).
- ROS 2 Jazzy and the Micro XRCE-DDS Agent upstream.
- Gazebo Harmonic (gz-sim 8) upstream.
- QGroundControl (desktop, for M1 manual arm/takeoff verification — not containerized).
- The CI two-layer architecture from ADR-0002 (Layer B `colcon build`/`colcon test`; the SITL tier scaffold) — platform must build green within it.

## 6. Settled constraints (do NOT relitigate — cite the source)

- **Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12.** Decided in ADR-0001 and the plan's "Target stack (pinned)"; Humble/22.04 was deliberately reversed. Accept ~1 week of M1–M2 integration friction as the cost.
- **PX4 v1.16.x or latest stable**, with message versioning. (Plan "Target stack".)
- **uXRCE-DDS native, not MAVROS.** Native ROS 2 topics, no translation layer. (Plan "Target stack" / "How to engage"; ADR-0001 neutral consequences.)
- **Gazebo Harmonic (gz-sim 8), not Gazebo Classic.** Classic is deprecated. (Plan "Target stack".)
- **`px4_msgs` vendored and version-pinned**, not pulled at build time. (Plan "Repo structure".)
- **Two-container split (`sim`, `dev`) on a shared Ubuntu 24.04 + ROS 2 Jazzy base.** A defensible default per the plan's "Containerization"; alignment with the future Jetson (JetPack 7.2) environment is the rationale.
- **Foxglove and QGroundControl are NOT containerized** — desktop apps stay desktop apps. (Plan "Containerization".)
- **Native Linux only; WSL2 not supported.** (Plan "Dev hardware requirements".)
- **A DGX is not required for Phase 1.** (Plan "Dev hardware requirements".)
- **CI is two-layer per ADR-0002** — fast pure-Python Layer A and `colcon`-based Layer B; SITL is a deferred nightly scaffold, never a required per-PR check. The platform build must be green on Layer B.

## 7. Open decisions (handed to /drive — each: question · decision target · why open)

- **Container layout depth** · is the bare two-container (`sim`/`dev`) split sufficient, or does the uXRCE-DDS agent warrant its own service in compose? · the plan calls two-container "a defensible default, not the only answer" and explicitly welcomes pushback.
- **PX4 source vs prebuilt in the `sim` image** · build PX4-Autopilot from source inside the image, or layer a prebuilt SITL artifact for faster CI? · trades image build time / reproducibility against CI runtime budget (flagged as a low-confidence area in the plan's "Test strategy").
- **Exact PX4 / `px4_msgs` pin** · which specific v1.16.x tag and matching `px4_msgs` branch to vendor · needs the early-adopter integration spike (the plan's ~1-week M1–M2 friction) to settle on a known-good combination.
- **Headless rendering backend for CI** · software/llvmpipe vs hosted-runner GPU for Gazebo Harmonic's Vulkan rendering in the `sim` container · the plan notes SITL CI orchestration is its least-confident area; rendering on hosted runners is the practical risk.
- **GPU passthrough story for the `sim`/`dev` containers on dev hosts** · how (and whether) to expose the host GPU to Gazebo in the container for interactive dev · varies by collaborator hardware (4–8 GB discrete GPUs per the dev-hardware table); must not become a hard requirement.
- **README command-budget allocation** · how the ≤20-command budget is split between platform bring-up and the per-docset run steps siblings append · integrative item 10 spans all docsets; platform owns the spine but the total must stay under budget.

## 8. Assessment signals (so prd-engine right-sizes the PRD)

| Dimension | Value | One-line justification |
|---|---|---|
| Nature | infrastructure | Toolchain, containers, and the bridge that everything else stands on; no user-facing feature. |
| Complexity | complex | Multi-component integration (PX4 + Gazebo + ROS 2 + uXRCE-DDS + Docker) on an early-adopter stack with ~1 week of expected friction. |
| Urgency | standard | Foundational and gating, but deliberate — hardware purchase waits on it; no emergency. |
| Risk | medium | Early-adopter PX4-on-Jazzy combination is the highest-risk integration in Phase 1, but it is reversible and sim-only (no hardware, no data at stake). |
| Reversibility | mostly-reversible | Pins and container definitions can be re-cut; the distro/OS call (ADR-0001) is the one costly-to-reverse decision and it is already settled. |
| Scope | platform-wide | Every sibling docset and every later phase consumes this base workspace, container layout, and telemetry surface. |
| Audience | developer | Solo dev + collaborators reproducing the environment; internal tooling, not external. |

**Suggested PRD tier:** Standard (Complexity = complex × Risk = medium → Standard per the prd-engine Complexity×Risk matrix; the infrastructure nature pulls in Performance/Observability and Operational-Readiness conditional sections rather than a tier bump).

## 9. Traceability

- **Milestones:**
  - M1 — Toolchain installed, vanilla SITL flying (`make px4_sitl gz_x500` arms/takes off via QGC, hovers 60s) — docs/phase1_simulation_plan.md#m1--toolchain-installed-vanilla-sitl-flying
  - M2 — ROS 2 Jazzy + uXRCE-DDS bridge (`ros2 topic` shows `/fmu/*` at steady rate) — docs/phase1_simulation_plan.md#m2--ros-2-jazzy--uxrce-dds-bridge
- **Exit-checklist items owned:** 9 (containerized dev/sim build + single `colcon build` succeeds — primary owner); 10 (README setup-to-running-mission in <20 commands — integrative, platform is primary owner). Provides the SITL environment that item 4 (integration test, owned by 02) and items 5–8 (owned by 05) run inside.
- **Packages / dirs:** `ros2_ws/` (workspace + `colcon build`), `ros2_ws/src/external/px4_msgs`, `ros2_ws/src/external/px4_ros_com`, `ros2_ws/src/patrol_bringup` (shell), `ros2_ws/src/patrol_interfaces` (shell), `docker/sim/`, `docker/dev/`, top-level `README.md`, `docs/decisions/0001-distro-and-os.md`, `docs/decisions/0002-ci-architecture.md`.
- **Lifecycle:** dod.md (this) → prd.md (via /drive) → design.md (via /drive).
