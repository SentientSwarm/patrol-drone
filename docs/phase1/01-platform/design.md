# Platform & Simulation Foundation — System Design Working Document

**Status:** Approved (combined review 2026-06-03; bootstrapped to Linear)
**Version:** 0.3.0
**Date:** 2026-06-03
**Projects:** Autonomous Drone Patrol — Phase 1 Docset 01 (Platform & Simulation Foundation)
**Authors:** jxstanford@wemodulate.energy (solo dev / DRI)

> **Requirements source (sole):** `docs/phase1/01-platform/prd.md` (rev 2). This design realizes that PRD's FR table (PLAT-1 … PLAT-9) as components, container/workspace layout, and milestone tasks. It specifies *how*; it does not introduce surface area beyond the PRD's FR table (per the PRD's Scope Authority note). Where a `how` decision is genuinely open, it is recorded in §2, not silently invented.
>
> **Upstream (DoD):** `docs/phase1/01-platform/dod.md` — milestones M1–M2, the Capabilities (P1/P2) that seed the FRs, the falsifiable acceptance criteria (AC-1…AC-9), the owned/consumed interface contracts (§5), and the settled constraints (§6) this design does not relitigate.

---

## 1. Introduction

This design covers the pre-hardware **platform substrate** for Phase 1 of the autonomous-drone project: a pinned, containerized PX4-SITL + Gazebo-Harmonic + ROS 2 Jazzy stack that flies a vanilla `gz_x500` drone, exposes PX4 as native `/fmu/*` ROS 2 topics over the uXRCE-DDS bridge, builds cleanly with a single `colcon build` inside a `sim`/`dev` container pair, and is documented end-to-end in a ≤20-command README. It is the foundation every other Phase 1 docset (02–05) and every later hardware phase (2–8) stands on. It is infrastructure, not a user-facing feature — its entire value is downstream reproducibility.

The design is deliberately **right-sized for simulation Phase 1**. There is no application database, no REST/SDK surface, no multi-tenant auth, and no production runtime. The "components" are containers, a colcon workspace, vendored message packages, a process-level bridge, a pinned-version manifest, and a README — plus the two empty package shells (`patrol_bringup`, `patrol_interfaces`) that siblings land their contents into. The "layers" are the host/toolchain layer, the container layer, the simulation/flight-stack layer, the ROS 2 middleware/bridge layer, and the workspace/build layer. The "systemic interfaces" are developer-facing verification commands and the existing two-layer CI (ADR-0002), not a monitoring stack.

Two cross-docset contracts touch this docset even though it does not own either: the checkpoint-mapping schema (owned by 03-sim-environment) and the `CheckpointCapture` message representation (owned by 04-perception; this docset provides only the empty `patrol_interfaces` landing slot the message lands in). On the first: this docset does **not** author or own any coordinate-frame convention — it ships only the vanilla `gz_x500` SITL baseline, and the ENU (Gazebo world) / NED (PX4 offboard) frames are inherited from upstream PX4/Gazebo defaults, not defined here. The frame in which 03's checkpoint positions are expressed, and the NED↔world conversion boundary, are owned by 03 and 02 respectively (02's DoD: "PX4 offboard uses NED relative to the EKF origin … conversion happens at one explicit boundary"). 01's only obligation is the negative one: the `gz_x500` baseline it ships must not contradict whatever frame those positions are ultimately expressed in. Both contracts are carried here as provisional, flagged decisions confirmed at the combined review (§2, OQ-7/OQ-8), matching the PRD's open-questions table.

### Source Projects (Linear)

| # | Project | Est. | Wave |
|---|---------|------|------|
| 1 | [Patrol Drone 01 Platform](https://linear.app/wemodulate/project/patrol-drone-01-platform-742e10556ec3) — Platform & Simulation Foundation (Swarm team; project id `b8c9ed1c-51dc-4ead-a63a-04d8de0cd352`) | ~2 ew (M1–M2, ~1 week each + integration friction) | 1 |

### Related Projects

| Project | Relevance |
|---------|-----------|
| 02-mission-control | Consumes `/fmu/*`, `px4_msgs`, `ros2_ws` build, `sim` container; lands `patrol_mission` + fills `patrol_bringup` shell |
| 03-sim-environment | Extends the `sim` world/airframe baseline; owns checkpoint-mapping YAML (OQ-7) |
| 04-perception | Builds nodes in the same workspace; owns `CheckpointCapture`, fills `patrol_interfaces` shell (OQ-8) |
| 05-logging-replay | Records the `/fmu/*` + `/patrol/*` topic surface; extends the `docker/ingest/` container slot |
| CI Workflows (ADR-0002) | The two-layer CI this platform build must be green on; Layer B (`ros-ci.yml`) already exists |

---

## 2. Open Questions & Assumptions

All items below mirror the PRD's Open Questions table (OQ-1 … OQ-8) — the design is the resolution venue for the `how`-shaped ones (OQ-1, OQ-2, OQ-4, OQ-5, OQ-6) and carries OQ-3 (integration-spike-gated) and OQ-7/OQ-8 (cross-docset contracts owned elsewhere) as deferrals. The combined human review (2026-06-03) ratified the design resolutions and confirmed the two provisional cross-docset defaults; OQ-3 and OQ-6 remain genuinely deferred (a probe/measurement settles each, not a paper decision). Each design resolution is recorded with rationale.

| # | Item | Source | Status / Resolution | Decision target |
|---|------|--------|---------------------|-----------------|
| OQ-1 | Two-container (`sim`/`dev`) split sufficient, or does the uXRCE-DDS agent warrant its own compose service? | PRD OQ-1 | **Resolved (design; ratified 2026-06-03):** keep the two-container split; run the agent as a **process inside the `sim` container** (its own compose-managed lifecycle is unnecessary in SITL where the client transport is automatic). The agent is one process in the `sim` container's entrypoint, not a third image. Rationale: minimal surface, matches the DoD's settled two-container constraint, and the agent has no independent scaling or failure-isolation need in Phase 1. Revisit if a sibling needs the agent without SITL. | — (resolved) |
| OQ-2 | Build PX4 from source in the `sim` image, or layer a prebuilt SITL artifact for faster CI? | PRD OQ-2 | **Resolved (design; ratified 2026-06-03):** **build PX4 from source** inside the `sim` image at a pinned tag, in a multi-stage Dockerfile (build stage → slim runtime stage) so the source toolchain does not bloat the runtime layer. Rationale: reproducibility is the project tenet ("Reproducibility over convenience"); a pinned source build is the source of truth, and the multi-stage split + a Docker-layer-cached build stage keeps rebuild cost bounded. The image is built once and cached; per-PR CI Layer B does **not** rebuild PX4 (it builds only the colcon workspace), so CI runtime is unaffected. | — (resolved) |
| OQ-3 | Which exact PX4 v1.16.x tag and matching `px4_msgs` branch to vendor? | PRD OQ-3 | **Deferred:** cannot be picked on paper; settled by the M1–M2 integration spike. The manifest (§4.2.9, C9) carries a `px4_version` / `px4_msgs_ref` pair as the single edit point; the spike fills it with a known-good combination. Also the falsification gate for the PRD's H2. Tracked in MZ (§6.5). | End of M1–M2 integration spike |
| OQ-4 | Headless rendering backend for CI — software/llvmpipe vs hosted-runner GPU for Gazebo Harmonic's Vulkan? | PRD OQ-4 | **Resolved (design; ratified 2026-06-03) for Phase 1 scope:** the `sim` container supports a **headless software-rendering path** (llvmpipe / `LIBGL_ALWAYS_SOFTWARE`-style env, no display) as the default CI-capable mode (PLAT-9). The hosted-runner-GPU question is moot for *required* CI because SITL stays a **nightly scaffold** (ADR-0002, `sitl-nightly.yml`), never a per-PR gate. The platform only owns "the `sim` container *can* run headless"; which runner the nightly SITL tier eventually uses is owned by the CI / sibling integration tier. | — (resolved for platform scope; nightly-runner choice deferred to 02/05) |
| OQ-5 | How (and whether) to expose the host GPU to Gazebo in the container for interactive dev? | PRD OQ-5 | **Resolved (design; ratified 2026-06-03):** GPU passthrough is an **optional, swappable compose profile** (`docker compose --profile gpu`) that adds the NVIDIA Container Runtime device reservation; the default profile is CPU/software-rendering. Rationale: collaborator GPUs range 4–8 GB and some have none usable in-container; passthrough must never be a hard requirement. The runtime is the same swappable parameter the Jetson image will flip (PLAT-3 AC3). | — (resolved) |
| OQ-6 | How is the ≤20-command README budget split between platform spine and per-docset run steps? | PRD OQ-6 | **Deferred (integrative):** the platform owns the bring-up *spine* and budgets it explicitly (§4.2.10, C10 targets ≤12 commands for clone→flying→`/fmu/*` live, leaving ≥8 for siblings); the final allocation needs sibling run-step counts and is settled at the M2 exit / item-10 integration. Tracked in MZ (§6.5). | M2 exit (item 10, integrative) |
| OQ-7 | Checkpoint-mapping schema (`sim/config/checkpoints.yaml`: `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`). | PRD OQ-7 | **Provisional default applied, confirmed at combined review (2026-06-03).** Owned by **03**; 01 is not an owner and **authors no coordinate-frame convention** — it ships only the vanilla `gz_x500` SITL baseline. The ENU (Gazebo world) / NED (PX4 offboard) frames are inherited from upstream PX4/Gazebo defaults; the frame the positions are expressed in is owned by 03, and the NED↔world conversion boundary is owned by 02 (per 02's DoD). 01's only obligation is the negative one: the `gz_x500` baseline must not contradict the frame 03's positions are expressed in. Recorded so the 5 docset pairs stay coherent. | Joint `/drive` review (03 owns) — confirmed 2026-06-03 |
| OQ-8 | `CheckpointCapture` representation — `std_msgs/Header header`, `string checkpoint_id`, `geometry_msgs/PoseStamped pose`, `string image_path` (NOT pixels by-value), `diagnostic_msgs/KeyValue[] metadata`; a separate `sensor_msgs/CompressedImage` topic carries live frames for the bag. | PRD OQ-8 | **Provisional default applied, confirmed at combined review (2026-06-03).** Owned by **04**; lands in 01's `patrol_interfaces` shell (C8). 01 owns only the empty landing slot — the slot must *accept* this message but defines none of its fields. Recorded so the 5 docset pairs stay coherent. | Joint `/drive` review (04 owns) — confirmed 2026-06-03 |

**Assumptions** (explicit, design-level):

- **A1:** PX4 v1.16's bundled `uxrce_dds_client` auto-starts in SITL with the correct UDP-localhost transport (PRD Technical Considerations; verified in §3.2 against upstream docs). If the spike (OQ-3) finds a tag where this is not automatic, the `sim` entrypoint adds an explicit `uxrce_dds_client start` — a one-line entrypoint change, not a design change.
- **A2:** `patrol_interfaces` is an `ament_cmake` (C++/IDL) package so it can host ROS message definitions; `patrol_bringup` is an `ament_python` package (launch/config host). This matches the repo's existing `ros2_ws/README.md` package plan (§3.2).
- **A3:** The empty-but-present shells must build under `colcon build` on the current empty workspace; the CI Layer B job (`ros-ci.yml`) already no-ops green on the empty tree and self-activates as packages land (ADR-0002), so the shells are the first packages that activate it.
- **A4:** The reference dev host for non-gating wall-clock budgets is the PRD "comfortable" profile (8-core / 32 GB / 8 GB GPU). Minimum-spec (6-core / 16 GB / 4 GB GPU) is supported but not the budgeting baseline.

---

## 3. Existing Foundation

This is a greenfield simulation foundation, but it lands in an **existing repo skeleton** (committed 2026-06-03) with placeholders, two ADRs, and a fully-built two-layer CI. The "existing foundation" is therefore the repo scaffold + the settled architectural decisions, not a running application.

### 3.1 Repo-Skeleton Architecture (the layers this design populates)

```
patrol-drone/                          LAYER
├── docker/                            ┌─ Container layer (this docset owns sim/, dev/; ingest/ slot → 05)
│   ├── sim/    (.gitkeep)             │   PX4 SITL + Gazebo + agent + workspace
│   ├── dev/    (.gitkeep)             │   shared base + tooling, source mounted
│   └── ingest/ (.gitkeep)            └─  05 fills this
├── docker-compose.yml  (new)          ┌─ Orchestration (this docset owns; --profile gpu optional)
├── ros2_ws/                           ┌─ Workspace/build layer (this docset owns the baseline)
│   └── src/   (.gitkeep)              │   colcon workspace; patrol_* shells + external/
├── sim/                               ┌─ Sim-asset layer (03 owns worlds/models; 01 ships vanilla gz_x500 only)
│   ├── worlds/ models/ px4_sitl_overrides/
├── .github/workflows/                 ┌─ CI layer (ADR-0002 — already built)
│   ├── python-quality.yml (Layer A)   │   fast pure-Python gates
│   ├── ros-ci.yml         (Layer B)   │   colcon build + colcon test  ← platform build green here
│   └── sitl-nightly.yml   (scaffold)  └─  SITL nightly, never per-PR
├── docs/decisions/0001…, 0002…        ← settled constraints (cited, not relitigated)
├── stack-manifest.* (new)             ← pinned-version single source of truth (C9)
└── README.md                          ← integrative bring-up spine (this docset owns, C10)
```

| Layer | Owns (this docset) | Current state |
|-------|--------------------|---------------|
| **Host / toolchain** | The pinned-version manifest; the install path the README documents | None — README + manifest are new |
| **Container** | `docker/sim/`, `docker/dev/`, `docker compose` orchestration | `.gitkeep` placeholders only |
| **Simulation / flight stack** | PX4 SITL + Gazebo Harmonic (`gz_x500`) inside `sim` | None — new |
| **ROS 2 middleware / bridge** | uXRCE-DDS agent process; `/fmu/*` topic surface | None — new |
| **Workspace / build** | `ros2_ws` layout, `colcon build` entrypoint, vendored `external/`, `patrol_*` shells | `ros2_ws/src/.gitkeep` only |
| **CI** | (consumes) platform build must be green on Layer B | **Already built** (`ros-ci.yml`) |

### 3.2 Verified Preconditions

External-system and existing-repo claims this design depends on, verified at research time against the actual repo and upstream sources.

| Claim | Verification | Result | Citation |
|-------|--------------|--------|----------|
| The repo skeleton already provides empty `docker/sim/`, `docker/dev/`, `docker/ingest/` slots and an empty `ros2_ws/src/` | `find . -type f -not -path './.venv*'` against the repo | `docker/sim/.gitkeep`, `docker/dev/.gitkeep`, `docker/ingest/.gitkeep`, `ros2_ws/src/.gitkeep` all present | `docker/sim/.gitkeep`; `docker/dev/.gitkeep`; `ros2_ws/src/.gitkeep` |
| The CI already has a Layer B `colcon build`/`colcon test` job the platform build must be green on | `ls .github/workflows/` + read ADR-0002 | `ros-ci.yml` exists (`action-ros-ci` colcon build+test, Jazzy container); guarded to no-op green on the empty skeleton, self-activates as packages land | `.github/workflows/ros-ci.yml`; `docs/decisions/0002-ci-architecture.md:26-29,46-48` |
| The distro/OS/ROS pins (Ubuntu 24.04 + Jazzy + Python 3.12; JetPack 7.2 for later) are settled, not open | Read ADR-0001 | "Adopt Option B: Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2 … starting from Phase 1" | `docs/decisions/0001-distro-and-os.md:23` |
| The intended package split is `patrol_interfaces` = ament_cmake (hosts messages), `patrol_bringup` = ament_python (launch/config) | Read repo workspace README | "`patrol_interfaces` \| C++ (ament_cmake) \| Custom messages…"; "`patrol_bringup` \| Python (ament_python) \| Launch files, configs, params" | `ros2_ws/README.md` (Planned packages table) |
| `px4_msgs` is intended to be vendored + version-pinned under `src/external/`, not pulled at build time | Read repo workspace README + plan | "`px4_msgs` — … pinned to the PX4 firmware version we build against. Branch correspondence matters; do not unpin." | `ros2_ws/README.md` (External dependencies); `docs/phase1_simulation_plan.md:170` |
| PX4 v1.16+ bundles `uxrce_dds_client`; in SITL the client transport (UDP localhost) is automatic | Read plan "Target stack" + M2 narrative (upstream PX4 uXRCE-DDS docs) | "uXRCE-DDS \| bundled with PX4 v1.14+"; "In SITL it's automatic; on real hardware it's a parameter." | `docs/phase1_simulation_plan.md:111,203` |
| `make px4_sitl gz_x500` is the canonical PX4-SITL Gazebo-Harmonic launch target | Read plan M1 narrative (upstream PX4 build docs) | `make px4_sitl gz_x500` documented as the M1 launch command | `docs/phase1_simulation_plan.md:190` |

### 3.3 Architectural Decision (inherited, not made here)

**Decision:** Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12; PX4 v1.16.x; Gazebo Harmonic; uXRCE-DDS native (not MAVROS); `px4_msgs` vendored+pinned; two-container `sim`/`dev` split; two-layer CI.
**Rationale:** Settled in ADR-0001, ADR-0002, and the plan's "Target stack" / "Containerization". This design **does not relitigate** them (PRD §Tenets, DoD §6).
**Implication:** Every component below is pinned to these choices; the manifest (§4.2.9, C9) is where the exact versions live.

### 3.4 Responsibility Matrix for the Platform Foundation

| Feature aspect | Layer / Component | Why |
|----------------|-------------------|-----|
| Flight dynamics + airframe (`gz_x500`) | Simulation/flight stack — C1 | PX4 SITL + Gazebo Harmonic own physics; 01 ships the vanilla baseline only |
| PX4↔ROS 2 bridging | ROS 2 middleware/bridge — C4 | The agent is the single bridge process; `/fmu/*` is the owned contract |
| Reproducible runtime | Container — C1/C2/C3 | Pinned images + compose eliminate host skew |
| Green build surface | Workspace/build — C5/C6/C7/C8 | `colcon build` over vendored + shell packages is the landing slot |
| Version single-source-of-truth | Host/toolchain — C9 | One manifest pins every layer; OQ-3 has a single edit point |
| Onboarding / reproducibility narrative | Host/toolchain — C10 | README spine is the integrative bring-up document |

### 3.5 Codebase Snapshot

| Repository | Branch | Commit | Date | Relevant Paths |
|-----------|--------|--------|------|----------------|
| `patrol-drone` | `main` | `8d03170` | 2026-06-03 | `docker/{sim,dev,ingest}/`, `ros2_ws/`, `sim/`, `.github/workflows/`, `docs/decisions/`, `README.md` |

---

## 4. Detailed Design

### 4.1 UC Traceability Matrix

Every UAC in the PRD's Appendix B (UAC-PLAT-1 … UAC-PLAT-8) and the P2 FR (PLAT-9) maps to at least one design component. Reverse (PRD-trace): every component below traces to a named PLAT FR — no component introduces surface beyond the FR table.

| Design Component | Covers FRs / UACs | Milestone |
|------------------|-------------------|-----------|
| **C1 — `sim` container** | PLAT-1, PLAT-3, PLAT-9 (UAC-PLAT-1, -3) | M1→M2 |
| **C2 — `dev` container** | PLAT-3 (UAC-PLAT-3) | M2 |
| **C3 — `docker compose` orchestration** | PLAT-3, PLAT-5, PLAT-9 (UAC-PLAT-3) | M2 |
| **C4 — uXRCE-DDS agent process + `/fmu/*` topic surface** | PLAT-2 (UAC-PLAT-2) | M2 |
| **C5 — `ros2_ws` colcon workspace** | PLAT-4, PLAT-5, PLAT-8 (UAC-PLAT-4, -5, -8) | M2 |
| **C6 — vendored `px4_msgs` / `px4_ros_com`** | PLAT-5 (UAC-PLAT-5) | M2 |
| **C7 — `patrol_bringup` package shell** | PLAT-8 (UAC-PLAT-8) | M2 |
| **C8 — `patrol_interfaces` package shell** | PLAT-8 (UAC-PLAT-8); landing slot for OQ-8 | M2 |
| **C9 — pinned stack manifest** | PLAT-7 (UAC-PLAT-7) | M1→M2 |
| **C10 — setup-to-running-mission README spine** | PLAT-6 (UAC-PLAT-6) | M2 |

Reverse-trace check — every PLAT FR has at least one component:

| FR | Covered by | FR | Covered by |
|----|-----------|----|-----------|
| PLAT-1 | C1 | PLAT-6 | C10 |
| PLAT-2 | C4 | PLAT-7 | C9 |
| PLAT-3 | C1, C2, C3 | PLAT-8 | C5, C7, C8 |
| PLAT-4 | C5 | PLAT-9 | C1, C3 |
| PLAT-5 | C3, C5, C6 | | |

### 4.2 Component Architecture

The building blocks of the platform — their boundaries, responsibilities, and relationships. No application services, no database, no API surface; the components are containers, a colcon workspace, vendored message source, a bridge process, a manifest, and a README.

#### 4.2.0 Component Inventory

| Component | Type | Boundary | Responsibility | Dependencies |
|-----------|------|----------|----------------|--------------|
| **C1 `sim` container** | container | Owns PX4 SITL + Gazebo Harmonic + uXRCE-DDS agent + the built workspace; does **not** own world assets (03) or mission logic (02) | Fly `gz_x500`; run the agent; expose `/fmu/*`; run headless for CI | C9 (pins), C6 (vendored msgs), C5 (workspace) |
| **C2 `dev` container** | container | Owns the shared base + dev tooling + source-as-volume mount; does **not** bake PX4 SITL or source code in | Day-to-day interactive build/edit environment | C9, C5 (mounted) |
| **C3 `docker compose` orchestration** | infrastructure/config | Owns the compose file, service decomposition, and the optional `gpu` profile; does **not** own a third agent service (OQ-1) | Bring up `sim`/`dev`; swap CPU↔GPU runtime parameter | C1, C2, C9 |
| **C4 uXRCE-DDS agent + `/fmu/*` surface** | process / external-bundled | Owns the agent process lifecycle inside C1 and the `/fmu/out/*` + `/fmu/in/*` topic contract; does **not** own message *contents* (px4_msgs) | Bridge PX4↔ROS 2 over UDP-localhost at steady ~50 Hz | C1 (host), C6 (types) |
| **C5 `ros2_ws` colcon workspace** | module/workspace | Owns the workspace layout, the `colcon build` entrypoint, and `src/external/` + `src/patrol_*` layout; does **not** own sibling package contents | One green `colcon build`; the shared landing workspace | C6, C7, C8 |
| **C6 vendored `px4_msgs` / `px4_ros_com`** | library (vendored) | Owns the committed, version-pinned copy under `src/external/`; does **not** track upstream at build time | Stable message vocabulary every node compiles against | C9 (`px4_msgs_ref` pin) |
| **C7 `patrol_bringup` shell** | module (package shell) | Owns the empty-but-present `ament_python` package; does **not** own launch/config contents (02) | A buildable landing slot for 02's launch files | C5 |
| **C8 `patrol_interfaces` shell** | module (package shell) | Owns the empty-but-present `ament_cmake` package; does **not** define `CheckpointCapture` fields (04) | A buildable landing slot that accepts 04's message (OQ-8) | C5 |
| **C9 pinned stack manifest** | infrastructure/config | Owns the single pinned-version source of truth + the `px4_version`/`px4_msgs_ref` OQ-3 edit point; does **not** pin sibling-owned versions | One place every layer's version is looked up | — |
| **C10 README bring-up spine** | documentation | Owns the integrative setup→running spine (≤12-command platform budget); does **not** own sibling run-steps | clone→flying→`/fmu/*` live, no tribal knowledge | C1, C3, C4, C5, C9 |

#### 4.2.0a Component Dependency Diagram

```
                         ┌──────────────────────────┐
                         │  C9 pinned stack manifest │  (single source of truth;
                         │  px4_version/px4_msgs_ref │   OQ-3 edit point)
                         └─────────────┬─────────────┘
              ┌────────────────────────┼────────────────────────┐
              ▼                        ▼                         ▼
     ┌─────────────────┐      ┌─────────────────┐       ┌─────────────────┐
     │ C1 sim container│      │ C2 dev container│       │ C6 vendored     │
     │ PX4 SITL+Gazebo │      │ base+tooling,   │       │ px4_msgs /      │
     │ +agent+workspace│      │ src as volume   │       │ px4_ros_com     │
     └───┬────────┬────┘      └────────┬────────┘       └────────┬────────┘
         │        │                    │                         │
         │        │  ┌─────────────────┴───────┐                 │ vendored into
         │        │  │ C3 docker compose        │                ▼
         │        │  │ orchestration            │       ┌─────────────────┐
         │        │  │ (--profile gpu optional) │       │ C5 ros2_ws       │
         │        │  └──────────────────────────┘       │ colcon workspace │
         │        │                                     │  src/external/ ◄─┘
         │        ▼                                     │  src/patrol_*    │
         │  ┌──────────────────────────┐                └───┬──────────┬───┘
         │  │ C4 uXRCE-DDS agent        │                    │          │
         │  │ /fmu/out/* (~50 Hz)       │            ┌───────▼──┐  ┌────▼────────┐
         │  │ /fmu/in/*  (addressable)  │            │ C7       │  │ C8          │
         │  └───────────┬──────────────┘            │ patrol_  │  │ patrol_     │
         │              │ consumed by 02/05         │ bringup  │  │ interfaces  │
         │              ▼                           │ (shell)  │  │ (shell;     │
         │       [ 02 mission / 05 logging ]        └──────────┘  │  OQ-8 slot) │
         │                                                        └─────────────┘
         └──────────────────────────────────────────────────────────────────►
                  C10 README bring-up spine narrates C1→C3→C4→C5 path
```

Every C1–C10 inventory row appears as a node above; every node traces back to an inventory row. Inventory triangle (inventory ↔ diagram ↔ consumer-facing manifestation in §4.2.12) is consistent.

#### 4.2.1 C1 — `sim` container

**Type:** container
**Boundary:** Owns PX4 SITL (built from source at the pinned tag), Gazebo Harmonic, the uXRCE-DDS agent process, and a built copy of `ros2_ws`. Does **not** own custom worlds/models (03) or mission logic (02).
**Location:** `docker/sim/Dockerfile`, `docker/sim/entrypoint.sh`
**Dependencies:** C9 (version pins), C6 (vendored messages built into the workspace), C5 (the workspace)

Multi-stage Dockerfile (OQ-2 resolution: build-from-source, slim runtime):

```dockerfile
# ── Stage 1: px4-build ───────────────────────────────────────────────
# Pinned base (C9: ros_distro=jazzy → osrf/ros:jazzy-desktop on Ubuntu 24.04)
FROM ${ROS_BASE_IMAGE} AS px4-build
ARG PX4_VERSION            # C9: px4_version (OQ-3 edit point)
RUN git clone --recurse-submodules --branch ${PX4_VERSION} \
        https://github.com/PX4/PX4-Autopilot.git /opt/PX4-Autopilot \
 && bash /opt/PX4-Autopilot/Tools/setup/ubuntu.sh --no-nuttx \
 && make -C /opt/PX4-Autopilot px4_sitl gz_x500   # compiles SITL + gz target

# ── Stage 2: runtime ─────────────────────────────────────────────────
FROM ${ROS_BASE_IMAGE} AS runtime
ARG GZ_VERSION             # C9: gazebo=harmonic
# Gazebo Harmonic + Micro XRCE-DDS Agent + headless software-render deps
RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-${GZ_VERSION} \
        ros-jazzy-micro-xrce-dds-agent \
        mesa-utils libgl1-mesa-dri \              # llvmpipe software render
 && rm -rf /var/lib/apt/lists/*
# Copy only the built SITL artifacts from the build stage (no source toolchain)
COPY --from=px4-build /opt/PX4-Autopilot/build /opt/PX4-Autopilot/build
COPY --from=px4-build /opt/PX4-Autopilot/Tools /opt/PX4-Autopilot/Tools
COPY ros2_ws /opt/ros2_ws
RUN . /opt/ros/jazzy/setup.sh && colcon build --base-paths /opt/ros2_ws
COPY docker/sim/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
```

Entrypoint contract (`docker/sim/entrypoint.sh`):

```bash
#!/usr/bin/env bash
set -euo pipefail
# Headless software-rendering default (OQ-4); GPU profile overrides via env (OQ-5)
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"

source /opt/ros/jazzy/setup.bash
source /opt/ros2_ws/install/setup.bash

# 1. Micro XRCE-DDS Agent on UDP-localhost (the bridge — C4)
MicroXRCEAgent udp4 -p 8888 &

# 2. PX4 SITL + Gazebo Harmonic gz_x500 (headless unless display attached)
HEADLESS="${HEADLESS:-1}" make -C /opt/PX4-Autopilot px4_sitl gz_x500
# In SITL the bundled uxrce_dds_client auto-starts (A1); the entrypoint adds an
# explicit `uxrce_dds_client start` only if the OQ-3 spike finds it isn't automatic.
```

**Performance note:** PX4 source build happens once at image-build time (Docker-layer-cached); per-PR CI does not rebuild it. Headless llvmpipe drops the render cost so the container runs without a display in CI (PLAT-9). Single drone / simple world stays within the minimum-spec 4 GB-GPU / 16 GB-RAM envelope.

*Traces to: PLAT-1, PLAT-3, PLAT-9 (UAC-PLAT-1, UAC-PLAT-3).*

#### 4.2.2 C2 — `dev` container

**Type:** container
**Boundary:** Owns the shared base image + interactive dev tooling (editor server, debugger, Python/colcon tooling) and mounts the host source tree as a volume. Does **not** bake PX4 SITL or the source code into the image (so edits are live).
**Location:** `docker/dev/Dockerfile`
**Dependencies:** C9 (same pinned base as C1), C5 (mounted, not baked)

```dockerfile
FROM ${ROS_BASE_IMAGE} AS dev      # identical base layer as C1 (C9)
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions python3-vcstool \
        gdb vim less ros-jazzy-rmw-cyclonedds-cpp \
 && rm -rf /var/lib/apt/lists/*
# Source is NOT copied — it is bind-mounted at runtime (see C3 compose volumes)
WORKDIR /workspace/ros2_ws
CMD ["bash"]
```

**Conventions:** shares the exact base layer hash with C1 (one base, two derivatives — the DoD's two-container constraint). Source is a bind-mount so a node author edits on the host and rebuilds in-container with no image rebuild.

*Traces to: PLAT-3 (UAC-PLAT-3).*

#### 4.2.3 C3 — `docker compose` orchestration

**Type:** infrastructure / config
**Boundary:** Owns `docker-compose.yml`, the `sim`/`dev` service decomposition, and the optional `gpu` profile. The uXRCE-DDS agent is **not** a third service (OQ-1 resolution — it is a process inside `sim`). Does **not** own the `ingest` service (05 fills `docker/ingest/`).
**Location:** `docker-compose.yml` (repo root)
**Dependencies:** C1, C2, C9

```yaml
# docker-compose.yml
services:
  sim:
    build:
      context: .
      dockerfile: docker/sim/Dockerfile
      args:
        ROS_BASE_IMAGE: ${ROS_BASE_IMAGE}   # from C9 manifest / .env
        PX4_VERSION:    ${PX4_VERSION}      # C9: px4_version (OQ-3)
        GZ_VERSION:     ${GZ_VERSION}       # C9: gazebo=harmonic
    network_mode: host                       # UDP-localhost agent<->PX4 (A1)
    environment:
      LIBGL_ALWAYS_SOFTWARE: "1"             # headless default (OQ-4)
      HEADLESS: "1"

  dev:
    build:
      context: .
      dockerfile: docker/dev/Dockerfile
      args:
        ROS_BASE_IMAGE: ${ROS_BASE_IMAGE}
    volumes:
      - ./ros2_ws:/workspace/ros2_ws        # source mounted, not baked (C2)
    network_mode: host

  # Optional GPU passthrough — never required (OQ-5). `docker compose --profile gpu up`
  sim-gpu:
    extends: { service: sim }
    profiles: ["gpu"]
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: ["gpu"]          # NVIDIA Container Runtime; Jetson swaps here (PLAT-3 AC3)
    environment:
      LIBGL_ALWAYS_SOFTWARE: "0"
```

**New in this design:** the `gpu` profile is the swappable runtime parameter — default CPU/software-render, optional NVIDIA reservation. The same `devices`/runtime knob is what the Phase 2 Jetson image flips (no Dockerfile rewrite — PLAT-3 AC3).

*Traces to: PLAT-3, PLAT-5, PLAT-9 (UAC-PLAT-3).*

#### 4.2.4 C4 — uXRCE-DDS agent process + `/fmu/*` topic surface

**Type:** process (Micro XRCE-DDS Agent, bundled) + owned topic contract
**Boundary:** Owns the agent process lifecycle (started in C1's entrypoint) and the `/fmu/out/*` + `/fmu/in/*` topic contract that 02/05 consume. Does **not** own the message *contents* (those are C6's `px4_msgs` types) or the mission that drives `/fmu/in/*` (02).
**Location:** `docker/sim/entrypoint.sh` (process start); contract documented in C10 README + this section.
**Dependencies:** C1 (host process), C6 (the `px4_msgs` types the topics carry)

Owned topic contract (the surface 02/05 build against):

| Topic | Type (`px4_msgs/…`) | Direction | Rate | Consumer |
|-------|---------------------|-----------|------|----------|
| `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` | PX4→ROS 2 | steady ~50 Hz (PX4 SITL default) over 60 s | 02 (offboard), 05 (record) |
| `/fmu/out/vehicle_status` | `VehicleStatus` | PX4→ROS 2 | event/periodic | 02, 05 |
| `/fmu/out/battery_status` | `BatteryStatus` | PX4→ROS 2 | periodic | 02 (low-battery abort), 05 |
| `/fmu/out/*` (full set) | various `px4_msgs/*` | PX4→ROS 2 | per topic | 05 (broad record) |
| `/fmu/in/*` (e.g. `vehicle_command`, `offboard_control_mode`, `trajectory_setpoint`) | various `px4_msgs/*` | ROS 2→PX4 | command-driven | 02 (offboard control) |

Transport: UDP-localhost (`udp4 -p 8888`); the PX4-side `uxrce_dds_client` auto-starts in SITL (A1). The platform's only acceptance obligation (PLAT-2) is that `ros2 topic list | grep fmu` returns the topics and `/fmu/out/vehicle_local_position` holds a steady ~50 Hz over a 60 s window, with `/fmu/in/*` present and addressable.

**Failure mode (the #1 stage failure):** if `ros2 topic list | grep fmu` returns nothing, the agent or `uxrce_dds_client` is down — detection and recovery are in §4.4.5.

*Traces to: PLAT-2 (UAC-PLAT-2).*

#### 4.2.5 C5 — `ros2_ws` colcon workspace

**Type:** module / workspace
**Boundary:** Owns the workspace layout, the `colcon build` entrypoint, and the `src/external/` + `src/patrol_*` layout. Does **not** own the contents of sibling packages.
**Location:** `ros2_ws/` (build entrypoint `colcon build` run from `ros2_ws/`)
**Dependencies:** C6 (vendored external), C7, C8 (shells)

Workspace layout after this docset:

```
ros2_ws/
├── src/
│   ├── external/
│   │   ├── px4_msgs/       # C6 — vendored, pinned (committed)
│   │   └── px4_ros_com/    # C6 — vendored helpers/examples
│   ├── patrol_bringup/     # C7 — ament_python shell (empty, builds)
│   └── patrol_interfaces/  # C8 — ament_cmake shell (empty, builds; OQ-8 slot)
├── build/  install/  log/  # ephemeral colcon outputs (gitignored)
```

Build contract: a single `colcon build` from `ros2_ws/` completes with exit code 0 — green both in-container (C1/C2) and on CI Layer B (`ros-ci.yml`, ADR-0002). The empty-but-present `patrol_*` shells build as part of the workspace. Clean rebuild recovery (INF-P1): `rm -rf build/ install/ log/ && colcon build`.

*Traces to: PLAT-4, PLAT-5, PLAT-8 (UAC-PLAT-4, -5, -8).*

#### 4.2.6 C6 — vendored `px4_msgs` / `px4_ros_com`

**Type:** library (vendored, committed)
**Boundary:** Owns the committed copy under `src/external/`, pinned to `px4_msgs_ref` (the matching branch/tag for the chosen PX4 firmware, OQ-3). Does **not** pull at build time — the copy is source-controlled and unchanged by upstream PX4 movement (PRD H3).
**Location:** `ros2_ws/src/external/px4_msgs/`, `ros2_ws/src/external/px4_ros_com/`
**Dependencies:** C9 (`px4_msgs_ref` pin)

| Property | Value |
|----------|-------|
| Vendoring | committed into the repo (not a `.repos`/`vcs` fetch at build time) |
| Pin | `px4_msgs_ref` from C9, matching the PX4 firmware tag (`px4_version`) |
| Builds as | part of the single `colcon build` (C5) |
| Determinism check (INF-P2 / H3 signal) | re-running `colcon build` after an unrelated upstream PX4 change still succeeds because the vendored copy is unchanged |

*Traces to: PLAT-5 (UAC-PLAT-5).*

#### 4.2.7 C7 — `patrol_bringup` package shell

**Type:** module (package shell)
**Boundary:** Owns the empty-but-present `ament_python` package — a valid, buildable ROS 2 package with no contents. Does **not** own launch files / configs / params (02 fills those).
**Location:** `ros2_ws/src/patrol_bringup/`
**Dependencies:** C5

Minimal valid `ament_python` shell:

```
patrol_bringup/
├── package.xml                  # <build_type>ament_python</build_type>
├── setup.py                     # entry_points empty; package metadata only
├── setup.cfg
├── resource/patrol_bringup      # ament resource marker
└── patrol_bringup/__init__.py   # empty module
```

`package.xml` declares `<buildtool_depend>ament_python` and no functional deps; it builds green as part of C5's `colcon build`. 02 lands `mission_basic.launch.py` / `mission_patrol.launch.py` here later — no rebuild-the-world step.

*Traces to: PLAT-8 (UAC-PLAT-8).*

#### 4.2.8 C8 — `patrol_interfaces` package shell

**Type:** module (package shell)
**Boundary:** Owns the empty-but-present `ament_cmake` (IDL-capable) package. Does **not** define the `CheckpointCapture` message fields — that is 04's contract (OQ-8). The slot must **accept** the OQ-8 shape (`std_msgs/Header header`, `string checkpoint_id`, `geometry_msgs/PoseStamped pose`, `string image_path`, `diagnostic_msgs/KeyValue[] metadata`) without 01 authoring any field.
**Location:** `ros2_ws/src/patrol_interfaces/`
**Dependencies:** C5

Minimal valid `ament_cmake` IDL-host shell:

```
patrol_interfaces/
├── package.xml          # <build_type>ament_cmake</build_type>;
│                        #   member_of_group rosidl_interface_packages
├── CMakeLists.txt       # find_package(rosidl_default_generators);
│                        #   rosidl_generate_interfaces(... )  ← empty msg list now
└── msg/                 # empty — 04 lands CheckpointCapture.msg here
```

The `CMakeLists.txt` is wired for `rosidl_generate_interfaces` so that when 04 drops `msg/CheckpointCapture.msg` in, the package generates types with no structural change. With an empty `msg/` list it still builds green (the package is a valid, buildable interface package). The shell declares the dependencies the OQ-8 message will need (`std_msgs`, `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`) so 04's drop-in is a one-file add.

*Traces to: PLAT-8 (UAC-PLAT-8); landing slot for OQ-8.*

#### 4.2.9 C9 — pinned stack manifest

**Type:** infrastructure / config
**Boundary:** Owns the single pinned-version source of truth and the `px4_version`/`px4_msgs_ref` pair that is the OQ-3 spike's single edit point. Does **not** pin sibling-owned versions (camera/world/bag tooling).
**Location:** `stack-manifest.{toml|yaml}` (repo root) + `.env` for compose ARG plumbing
**Dependencies:** none (it is the root of the dependency graph)

```toml
# stack-manifest.toml — single source of truth (PLAT-7). README + Dockerfiles
# + docker-compose.yml cite this; nothing pins a version anywhere else.
[os]
ubuntu        = "24.04"            # Noble Numbat (ADR-0001)

[middleware]
ros_distro    = "jazzy"           # ROS 2 Jazzy Jalisco (ADR-0001)
python        = "3.12"            # Ubuntu 24.04 default

[flight_stack]
px4_version   = "v1.16.x-PINNED"  # ← OQ-3 single edit point (filled by M1–M2 spike)
px4_msgs_ref  = "release/1.16"    # ← OQ-3 matching branch (filled by spike)

[simulator]
gazebo        = "harmonic"        # gz-sim 8 (ADR-0001 / plan)

[bridge]
uxrce_dds     = "bundled-px4-1.16"  # Micro XRCE-DDS Agent + bundled client

[bags]
mcap_plugin   = "rosbag2-storage-mcap"  # storage plugin (pin; 05 records)

[build]
colcon        = "pinned"          # colcon-common-extensions pin
docker        = "pinned"          # Docker + Compose pin

[container]
ros_base_image = "osrf/ros:jazzy-desktop"  # shared C1/C2 base
```

Every toolchain layer referenced anywhere in the build resolves to one row here. The README (C10) and the container definitions (C1/C2/C3) cite this manifest and pull values via `.env`/ARG — no version literal is duplicated. The OQ-3 resolution is a two-line edit (`px4_version`, `px4_msgs_ref`); nothing else moves.

*Traces to: PLAT-7 (UAC-PLAT-7).*

#### 4.2.10 C10 — setup-to-running-mission README spine

**Type:** documentation
**Boundary:** Owns the integrative setup→running bring-up spine — the platform's clone→flying→`/fmu/*`-live path, budgeted at ≤12 commands so siblings can append their run-steps within the shared ≤20-command budget (OQ-6). Does **not** own sibling run-steps (02–05 append).
**Location:** `README.md` (repo root)
**Dependencies:** C1, C3, C4, C5, C9

Platform bring-up spine (the budgeted ≤12-command path; exact commands finalized at M2/manifest pin):

```bash
# ── Platform bring-up spine (≤12 commands; siblings append within the ≤20 budget) ──
git clone --recurse-submodules <repo-url> patrol-drone        # 1
cd patrol-drone                                               # 2 (compound cd ok in README copy)
cp stack-manifest.toml .env.example .env                      # 3  (manifest → compose env)
docker compose build sim dev                                  # 4  (C1+C2 from shared base, PLAT-3)
docker compose run --rm dev colcon build                      # 5  (single green build, PLAT-4)
docker compose up -d sim                                      # 6  (PX4 SITL + Gazebo + agent, PLAT-1)
docker compose exec sim ros2 topic list | grep fmu            # 7  (bridge up, PLAT-2)
docker compose exec sim ros2 topic hz \
    /fmu/out/vehicle_local_position                           # 8  (~50 Hz over 60 s, PLAT-2)
# (QGroundControl, desktop, arms/takes off — M1 manual verification, PLAT-1)
# Siblings (02–05) append `ros2 launch patrol_bringup mission_patrol.launch.py` etc.
```

**Conventions:** the spine is the integrative document of record; the manifest (C9) is the cited version source. The final budget split between this spine and sibling run-steps is settled at M2 exit / item-10 integration (OQ-6).

*Traces to: PLAT-6 (UAC-PLAT-6).*

#### 4.2.11 Inventory Triangle Consistency

| Inventory row (§4.2.0) | Diagram node (§4.2.0a) | Consumer-facing manifestation (§4.2.12) |
|------------------------|------------------------|------------------------------------------|
| C1 `sim` container | `C1 sim container` node | `docker compose up sim` / `docker/sim/` |
| C2 `dev` container | `C2 dev container` node | `docker compose run dev` / `docker/dev/` |
| C3 compose | `C3 docker compose` node | `docker-compose.yml` / `--profile gpu` |
| C4 agent + `/fmu/*` | `C4 uXRCE-DDS agent` node | `/fmu/out/*`, `/fmu/in/*` topics (02/05) |
| C5 workspace | `C5 ros2_ws` node | `colcon build` entrypoint |
| C6 vendored msgs | `C6 vendored` node | `ros2_ws/src/external/*` |
| C7 `patrol_bringup` | `C7 patrol_bringup` node | `ros2_ws/src/patrol_bringup/` |
| C8 `patrol_interfaces` | `C8 patrol_interfaces` node | `ros2_ws/src/patrol_interfaces/` (OQ-8 slot) |
| C9 manifest | `C9 manifest` node | `stack-manifest.toml` + `.env` |
| C10 README spine | `C10 README` node | `README.md` ≤12-command spine |

All three artifacts enumerate the identical C1–C10 set — no drift.

#### 4.2.12 Consumer-facing Manifestation

The "consumer surface" of this infrastructure docset is the set of files/commands/topics siblings and CI touch: the `docker compose` commands (C1/C2/C3), the `colcon build` entrypoint (C5), the `/fmu/out/*` + `/fmu/in/*` topic contract (C4), the vendored message vocabulary (C6), the two empty package shells as landing slots (C7/C8), the pinned manifest lookup (C9), and the README spine (C10). There is no SDK, REST surface, or public class library — by docset nature (infrastructure / local single-host sim).

### 4.3 Layer View

#### 4.3.1 Layer Mapping

| Layer | Components | Key Responsibilities |
|-------|-----------|----------------------|
| **Host / toolchain** | C9, C10 | Pin every version in one manifest; document the clone→flying spine |
| **Container** | C1, C2, C3 | Reproducible `sim`/`dev` from a shared base; optional GPU profile |
| **Simulation / flight stack** | C1 | Fly vanilla `gz_x500` in Gazebo Harmonic via PX4 SITL |
| **ROS 2 middleware / bridge** | C4 | Bridge PX4↔ROS 2; own the `/fmu/*` topic contract at ~50 Hz |
| **Workspace / build** | C5, C6, C7, C8 | One green `colcon build`; vendored messages + landing-slot shells |
| **CI** (consumed) | — (ADR-0002 Layer B `ros-ci.yml`) | Platform build must be green on Layer B per-PR; SITL nightly only |

Layer definitions match the repo skeleton (§3.1), not invented.

#### 4.3.2 Container layer — Design Notes

**Conventions:** one shared base image (C1 and C2 derive from the same `ros_base_image` row in C9). The agent is a process inside `sim`, not a service (OQ-1).
**New in this design:** the multi-stage `sim` Dockerfile (build-from-source PX4 → slim runtime, OQ-2); the optional `gpu` compose profile (OQ-5); headless software-render default (OQ-4).
**Integration points:** the container runtime (CPU vs NVIDIA Container Runtime) is the single swappable parameter the Phase 2 Jetson image flips (PLAT-3 AC3) — no Dockerfile rewrite.

#### 4.3.3 Workspace / build layer — Design Notes

**Conventions:** `src/external/` holds vendored third-party source; `src/patrol_*` holds first-party packages; `patrol_interfaces` is `ament_cmake` (IDL), `patrol_bringup` is `ament_python` (A2).
**New in this design:** the two empty-but-present shells (C7/C8) are the first packages that activate the guarded CI Layer B job (A3).
**Integration points:** the single `colcon build` is the entrypoint every sibling lands into; the workspace is green on Layer B (`ros-ci.yml`).

### 4.4 Systemic / Platform Interfaces

#### 4.4.1 Interface Integration Summary

| Interface | Current State (§3) | Design Changes | Priority |
|-----------|--------------------|----------------|----------|
| Observability (dev-facing verification) | None — greenfield | Verification commands (`ros2 topic list/hz`, `docker compose`/`colcon build` exit codes) + CI Layer B job status are the only "monitoring" | P1 |
| CI integration | `ros-ci.yml` Layer B already built (ADR-0002), no-ops green on empty tree | Platform lands the first packages that activate the guarded job; no CI change | P1 |
| Security | N/A — local single-user sim | UDP-localhost-only agent; `.env` secret hygiene; no auth surface, no principals | N/A (correct-by-scope) |
| Configuration | None | `stack-manifest.toml` + `.env` (C9) are the single config surface | P1 |
| Performance / Capacity | None | Headless software-render default; optional GPU profile; single-drone envelope | P2 |

#### 4.4.2 Observability (dev-facing verification)

**Current state:** none (greenfield).
**Design changes:** the "alerts" are developer-facing verification commands and CI signals, not a production monitoring stack. Standard tool output (PX4 SITL console, Gazebo, the agent, colcon) is the diagnostic surface; no custom structured logging is introduced.

| Signal | Confirms | Healthy condition |
|--------|----------|-------------------|
| `ros2 topic list \| grep fmu` | agent bridged, PX4 topics exist | PX4 topics returned (PLAT-2) |
| `ros2 topic hz /fmu/out/vehicle_local_position` | telemetry flowing | steady ~50 Hz over 60 s (PLAT-2) |
| `docker compose build` exit code | containers build from shared base | 0 / success (PLAT-3) |
| `colcon build` exit code (in-container) | workspace green | 0 / success (PLAT-4) |
| CI Layer B job status | platform build green in CI architecture | passing each PR (ADR-0002) |

**Failure mode:** if a verification command fails, the developer reads the standard tool output; there is no second observability tier (correct-by-scope for Phase 1 sim).

#### 4.4.3 CI Integration

**Current state:** `ros-ci.yml` (Layer B `colcon build`/`colcon test`, Jazzy container) is already built and guarded to no-op green on the empty skeleton (ADR-0002).
**Design changes:** the platform's `patrol_*` shells + vendored `external/` are the first packages that self-activate the guarded job (A3). No workflow file is modified by this docset. SITL stays a nightly scaffold (`sitl-nightly.yml`), never a required per-PR check (OQ-4).

**Failure mode:** if Layer B goes red, the green-build hard floor (PLAT-4, exit item 9) is breached and the PR cannot merge.

#### 4.4.4 Security

**Current state:** N/A.
**Design changes:** none required. The agent binds UDP-localhost only; `.env` carries no secrets beyond pinned-version ARGs; there is no auth surface, no network-exposed service, and no principals. `[OOS: local single-user SITL — no auth/data-isolation surface in Phase 1]`.

**Failure mode:** N/A by scope.

#### 4.4.5 Cross-cutting Failure Modes

| Category | Failure mode | Detection | Degraded behavior | Recovery |
|----------|--------------|-----------|-------------------|----------|
| Network dependency (the #1 stage failure) | uXRCE-DDS bridge down: SITL up but no `/fmu/*` topics | `ros2 topic list \| grep fmu` returns nothing | No ROS 2 telemetry; downstream nodes see no data | Confirm `MicroXRCEAgent` running + `uxrce_dds_client` up on UDP-localhost; restart the agent (re-run entrypoint) — the known #1 failure point (PLAT-2) |
| Persistent state | `colcon` build/install tree corrupted or stale (mixed partial build) | `colcon build` error referencing stale artifacts | Workspace won't build green | Clean rebuild (INF-P1): `rm -rf build/ install/ log/ && colcon build` |
| Persistent state | Vendored `px4_msgs` pin mismatches PX4 firmware tag | `colcon` error referencing `px4_msgs` | `colcon build` fails on message generation | Confirm `src/external/px4_msgs` pin matches `px4_version`/`px4_msgs_ref` (C9, OQ-3, PLAT-5) |
| Persistent state | Disk full on bag/build volume | write returns ENOSPC | build / sim run fails | Operator clears build outputs / bags; NVMe headroom per dev-hardware table |
| Network dependency | `docker compose` build fails (base pull or apt) | non-zero build exit | No `sim`/`dev` image | Re-check pinned manifest versions (C9, PLAT-7); confirm host has base prerequisites the README lists |
| Plugin / extension | Gazebo won't render headless (missing software-render deps) | Gazebo startup failure with no display | No simulator render | Use the headless `sim` path (PLAT-9); confirm `mesa-utils`/`libgl1-mesa-dri` present; GPU/runner choice tracked in OQ-4 |
| Identity provider | (all sub-modes) | — | — | `[OOS: local single-user SITL — no identity provider in scope]` |
| Mesh / cross-cluster | (all sub-modes) | — | — | `[OOS: single-host sim — no mesh / cross-cluster in Phase 1]` |

### 4.5 Key Interaction Sequences

#### Sequence 1: Clean checkout → flying SITL + live `/fmu/*` (happy path — PLAT-1+2+3+4)

```
Developer            docker compose         sim container              ROS 2 / agent (C4)
   |                      |                      |                            |
   ├─ git clone ─────────►│                      |                            |
   ├─ compose build ─────►│                      |                            |
   │                      ├─ build C1 (PX4 src,  |                            |
   │                      │   multi-stage) ─────►│ (image cached)             |
   │                      ├─ build C2 (dev) ────►│                            |
   ├─ run dev colcon build►│─────────────────────►│ colcon build (C5)         |
   │   ◄── exit 0 (green build, PLAT-4) ─────────┤                            |
   ├─ compose up sim ────►│─────────────────────►│ entrypoint:                |
   │                      │                      ├─ MicroXRCEAgent udp4 ──────►│
   │                      │                      ├─ make px4_sitl gz_x500      |
   │                      │                      │   (uxrce_dds_client auto)──►│ bridge up
   ├─ ros2 topic list | grep fmu ───────────────────────────────────────────►│
   │   ◄── /fmu/out/*, /fmu/in/* returned (PLAT-2) ──────────────────────────┤
   ├─ ros2 topic hz /fmu/out/vehicle_local_position ────────────────────────►│
   │   ◄── steady ~50 Hz over 60 s (PLAT-2) ─────────────────────────────────┤
   ├─ (QGroundControl, desktop) arm + takeoff ──► sim: holds altitude 60 s (PLAT-1)
```

#### Sequence 2: uXRCE-DDS bridge-down recovery (error path — §4.4.5 #1)

```
Developer                     sim container                 agent (C4)
   |                              |                             |
   ├─ ros2 topic list | grep fmu ─────────────────────────────►│
   │   ◄── (nothing) ─────────────────────────────────────────┤  ✗ bridge down
   ├─ docker compose exec sim \                                 |
   │     pgrep MicroXRCEAgent ──►│ (no process)                 |
   ├─ docker compose restart sim ►│ entrypoint re-runs:         |
   │                              ├─ MicroXRCEAgent udp4 ──────►│  ✓ agent back
   │                              ├─ uxrce_dds_client (auto/    |
   │                              │   explicit per A1) ────────►│  ✓ client back
   ├─ ros2 topic list | grep fmu ─────────────────────────────►│
   │   ◄── /fmu/* returned ───────────────────────────────────┤  ✓ recovered
```

#### Sequence 3: CI Layer B on a PR that lands a shell (PLAT-4 AC2, PLAT-8)

```
PR author            GitHub Actions          ros-ci.yml (Layer B, ADR-0002)
   |                      |                          |
   ├─ push PR (adds C7/C8 shells + C6 vendored) ────►│ guarded job self-activates (A3)
   │                      ├─ action-ros-ci ─────────►│ colcon build (Jazzy container)
   │                      │                          ├─ build px4_msgs (C6)
   │                      │                          ├─ build patrol_bringup (C7)
   │                      │                          ├─ build patrol_interfaces (C8)
   │                      │   ◄── exit 0 ────────────┤ colcon test (no-op for shells)
   │   ◄── Layer B green (PLAT-4 AC2) ───────────────┤
```

### 4.6 Data Model Changes (Consolidated)

No application database or schema. Persisted artifacts are source-controlled files (container definitions, vendored message source, pinned manifest, README) plus ephemeral build outputs.

| Artifact | Change | Detail |
|----------|--------|--------|
| `docker/sim/`, `docker/dev/` | **Replace `.gitkeep`** | Multi-stage `sim` Dockerfile + entrypoint (C1); `dev` Dockerfile (C2) |
| `docker-compose.yml` | **New** | `sim`/`dev` services + optional `gpu` profile (C3) |
| `ros2_ws/src/external/{px4_msgs,px4_ros_com}` | **New (vendored, committed)** | Pinned to `px4_msgs_ref` (C6) |
| `ros2_ws/src/{patrol_bringup,patrol_interfaces}` | **New (shells)** | Empty-but-present, buildable (C7/C8) |
| `stack-manifest.toml` + `.env` | **New** | Pinned-version single source of truth (C9) |
| `README.md` | **Placeholder → spine** | ≤12-command platform bring-up spine (C10) |
| `build/`, `install/`, `log/` | **Ephemeral (gitignored)** | colcon outputs; not persisted |

### 4.7 UX Mocks

Not applicable — no GUI. The "interface" is the command line and file tree; the closest analog is the README spine (§4.2.10) and verification-command outputs (§4.4.2). QGroundControl (the only M1-path GUI) is an external desktop app, explicitly not owned/containerized here.

---

## 5. Design Questions FAQ

### Q1: Main components and interactions

Ten components across five layers (§4.2). The `sim` container (C1) flies `gz_x500` via PX4 SITL + Gazebo Harmonic and hosts the uXRCE-DDS agent (C4), which bridges PX4 to ROS 2 and owns the `/fmu/*` topic contract. The `dev` container (C2) shares C1's base and mounts source. `docker compose` (C3) orchestrates both with an optional GPU profile. The `ros2_ws` workspace (C5) builds vendored `px4_msgs`/`px4_ros_com` (C6) and the two empty `patrol_*` shells (C7/C8) in a single green `colcon build`. The manifest (C9) pins every version; the README (C10) narrates the clone→flying→`/fmu/*` spine. Interactions are in §4.5.

### Q2: Core API contracts and data models

The owned contract is the `/fmu/*` ROS 2 topic surface (C4, §4.2.4): `/fmu/out/vehicle_local_position` (`px4_msgs/VehicleLocalPosition`) steady ~50 Hz over 60 s, the `/fmu/out/*` set, and an addressable `/fmu/in/*` command surface. There is no REST/SDK/DB. The `CheckpointCapture` message is **not** defined here (OQ-8, owned by 04); only its landing slot C8 must accept the shape `std_msgs/Header header`, `string checkpoint_id`, `geometry_msgs/PoseStamped pose`, `string image_path`, `diagnostic_msgs/KeyValue[] metadata`.

### Q3: Deployment and infrastructure dependencies

Infra is local dev hosts (native Linux, Ubuntu 24.04; minimum 6-core/16 GB/4 GB-GPU, comfortable 8-core/32 GB/8 GB-GPU) plus the existing two-layer CI (ADR-0002). All versions pin in the manifest (C9); compose pulls them via `.env`/ARG. The container runtime (CPU vs NVIDIA) is the single swappable parameter (OQ-5) that the Phase 2 Jetson image flips. No production runtime, no cloud infra.

### Q4: External components and interfaces

PX4-Autopilot upstream (SITL, `gz_x500`, bundled `uxrce_dds_client`); ROS 2 Jazzy + Micro XRCE-DDS Agent; Gazebo Harmonic (gz-sim 8); QGroundControl (desktop, M1 manual verification, not containerized); the CI two-layer architecture (ADR-0002). Each is a settled-constraint input (§3.3), not relitigated.

### Q5: Testing strategy

Build/integration gates (the platform's own tier): `docker compose build` (PLAT-3), `colcon build` green in-container + on CI Layer B (PLAT-4). E2E demo gates: `make px4_sitl gz_x500` hover 60 s (PLAT-1), `/fmu/*` at ~50 Hz over 60 s (PLAT-2), README clean-host reproduction in <20 commands (PLAT-6). SITL stays a nightly scaffold (`sitl-nightly.yml`), never per-PR (OQ-4); the integration tests that *exercise* the `sim` environment are owned by 02/05 (PLAT-9). Inferred: INF-P1 (clean-rebuild recovery), INF-P2 (pinned-manifest determinism on upstream PX4 change — H3 signal).

### Q6: Security implications

Minimal by nature — UDP-localhost-only agent, `.env` hygiene, no principals, no auth surface (§4.4.4). `[OOS]` for identity/mesh by scope.

### Q7: Technical risks and open questions

| Risk | Status | Tie to §2 |
|------|--------|-----------|
| PX4-on-Jazzy early-adopter integration friction (highest-risk Phase 1 integration) | Mitigated by M1–M2 spike; falsification gate for H2 | OQ-3 (Deferred) |
| Exact PX4 tag / `px4_msgs` branch not pickable on paper | Deferred to spike; single manifest edit point | OQ-3 |
| ≤20-command README budget allocation across docsets | Deferred (integrative); platform spine budgeted ≤12 | OQ-6 |
| Headless Vulkan render in CI | Resolved (software-render default; SITL nightly) | OQ-4 |
| GPU passthrough on heterogeneous hardware | Resolved (optional `--profile gpu`, never required) | OQ-5 |
| Checkpoint-frame compatibility (cross-docset) | Provisional, confirmed 2026-06-03; 01 owns no frame, must not contradict | OQ-7 |
| `CheckpointCapture` landing-slot acceptance | Provisional, confirmed 2026-06-03; slot accepts, defines nothing | OQ-8 |

All Open/Provisional/Deferred statuses match a §2 OQ row; OQ-3 and OQ-6 remain genuinely deferred (probe/measurement), the rest are resolved.

---

## 6. Implementation Plan

Three milestones — **M1, M2** (mirroring the DoD's M1/M2, walking-skeleton) plus **MZ** (terminal catch-all). Per-docset-local numbering (the PRD/DoD master-plan M3–M8 references are traceability links, not Linear milestones). Each milestone's Definition of Done includes a documentation true-up (§6.4); MZ holds the comprehensive final documentation + test consolidation (§6.5).

### 6.0 Linear Project

**Project:** [Patrol Drone 01 Platform](https://linear.app/wemodulate/project/patrol-drone-01-platform-742e10556ec3)
**Team:** Swarm
**Project id:** `b8c9ed1c-51dc-4ead-a63a-04d8de0cd352`
**Created from:** Section 6 of this document (bootstrapped to Linear 2026-06-03; tasks below map 1:1 to the created issues).

### 6.1 Milestone Overview

| # | Milestone | Type | Shippable Demo | Scope | Dependencies | Exit Criteria | Linear |
|---|-----------|------|----------------|-------|-------------|---------------|--------|
| M1 | Toolchain installed, vanilla SITL flying | skeleton | Stakeholder runs `make px4_sitl gz_x500`, arms/takes off from QGroundControl, watches a 60 s hover | Pinned toolchain that flies `gz_x500` + hovers 60 s (PLAT-1); manifest drafted (C9); `sim` build stage scaffolded (C1); first README fragment (C10). No ROS/containers-runtime yet (deliberate — validate the install before layering architecture) | None | `make px4_sitl gz_x500` cleanly launches; drone arms/takes off via QGC; holds altitude 60 s (PLAT-1). Manifest draft present; `sim` build stage compiles | Bootstrapped |
| M2 | ROS 2 Jazzy + uXRCE-DDS bridge + containerized green build | layer 1: ROS 2 bridge, reproducible build & onboarding | Stakeholder sees live `/fmu/*` at ~50 Hz over 60 s in the container, then reproduces the README path on a clean host in <20 commands | Vendored `px4_msgs`/`px4_ros_com` (C6); agent bridging + `/fmu/*` (C4); `sim`+`dev` from compose incl. headless + optional gpu (C1/C2/C3, PLAT-9); single green `colcon build` in-container + on CI Layer B (C5, PLAT-4); `patrol_*` shells (C7/C8); README spine (C10); finalized manifest (C9) | M1 | `ros2 topic list \| grep fmu` returns topics + `ros2 topic hz /fmu/out/vehicle_local_position` ~50 Hz over 60 s (PLAT-2); `docker compose` builds `sim`+`dev` (PLAT-3); single `colcon build` green in-container + on CI Layer B (PLAT-4); shells build (PLAT-8); README path <20 commands on clean host (PLAT-6) | Bootstrapped |
| MZ | Consolidation & deferred backlog | terminal | No new platform capability — MZ reviewed and cleared, or items explicitly punted to Phase 2 | Absorbs non-blocking work surfaced during M1–M2: OQ-3 pin, OQ-6 budget finalization, e2e/integration test expansion, final documentation + test consolidation | M2 | MZ reviewed and cleared, or items explicitly punted to Phase 2 | Bootstrapped |

### 6.2 Milestone Details

#### M1: Toolchain installed, vanilla SITL flying

**Type:** skeleton
**Goal:** the thinnest end-to-end slice — a pinned toolchain that flies `gz_x500` and holds altitude 60 s via QGroundControl (PLAT-1). No ROS/containers-runtime yet (deliberate — validate the install before layering architecture).
**Shippable demo:** stakeholder runs `make px4_sitl gz_x500`, arms/takes off from QGroundControl, and watches a 60-second hover.
**Dependencies:** none (foundation).
**Exit criteria:** `make px4_sitl gz_x500` cleanly launches with no plugin/dependency errors on a clean Ubuntu 24.04 host; drone arms/takes off via QGC; holds altitude 60 s (PLAT-1). Manifest draft present (C9); the `sim` multi-stage build stage compiles (C1).

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| ROS 2 install, the bridge, and `/fmu/*` | design §4.2.4 (C4 is an M2 component) | M2 |
| `sim`/`dev` container runtime orchestration | design §4.2.3 (C3 is M2) | M2 |
| Vendored `px4_msgs` + workspace build | design §4.2.5/§4.2.6 (C5/C6 are M2) | M2 |
| Exact PX4 tag / `px4_msgs` branch pin | design §2 OQ-3 ("cannot be picked on paper") | MZ (spike) |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T1.1 | Install toolchain + build PX4 from source (pinned tag); fly `gz_x500` (PLAT-1) | `PX4-Autopilot` checkout (external); host toolchain | C1 | Simulation/flight stack | L | — | PLAT-1 |
| T1.2 | Draft the pinned stack manifest (OS/ROS/PX4/Gazebo/bridge/Python/colcon/Docker) | `stack-manifest.toml` (new) | C9 | Host/toolchain | M | T1.1 | — |
| T1.3 | Scaffold the sim container build stage (multi-stage Dockerfile; build-from-source per OQ-2) | `docker/sim/Dockerfile` (new) | C1 | Container | L | T1.1, T1.2 | — |
| T1.4 | First README fragment (M1 bring-up path) | `README.md` (modify) | C10 | Host/toolchain | S | T1.1 | — |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| E2E (demo gate) | Vanilla SITL flight (PLAT-1) | `make px4_sitl gz_x500` launches; arm/takeoff via QGC; 60 s hover |
| Build | `sim` build stage compiles (C1) | Multi-stage build stage produces SITL artifacts at the pinned tag |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| `README.md` (M1 fragment) | developers | Clean-host install + `make px4_sitl gz_x500` path |
| `stack-manifest.toml` (draft) | developers / CI | Pinned versions (PX4 tag provisional until OQ-3 spike) |

#### M2: ROS 2 Jazzy + uXRCE-DDS bridge + containerized green build

**Type:** layer 1: ROS 2 bridge, reproducible build & onboarding
**Goal:** thicken the skeleton across every remaining layer — vendored messages (C6), agent bridging + `/fmu/*` (C4), `sim`+`dev` from compose incl. headless (C1/C2/C3, PLAT-9), single green `colcon build` in-container + on CI Layer B (C5, PLAT-4), `patrol_*` shells (C7/C8), README spine (C10), finalized manifest (C9).
**Shippable demo:** stakeholder sees live `/fmu/*` at ~50 Hz over 60 s in the container, then reproduces the README path on a clean host in <20 commands.
**Dependencies:** M1.
**Exit criteria:** `ros2 topic list | grep fmu` returns topics; `ros2 topic hz /fmu/out/vehicle_local_position` ~50 Hz over 60 s (PLAT-2); `docker compose` builds `sim`+`dev` (PLAT-3); single `colcon build` green in-container + on CI Layer B (PLAT-4); `patrol_*` shells build (PLAT-8); README path <20 commands on a clean host (PLAT-6); manifest finalized (PLAT-7).

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| Mission launch files / state machine in `patrol_bringup` | design §4.2.7 (C7 is an empty shell; contents owned by 02) | 02-mission-control (M3–M4) |
| `CheckpointCapture` message fields in `patrol_interfaces` | design §4.2.8 (C8 accepts but defines nothing; OQ-8 owned by 04) | 04-perception (M6) |
| Custom worlds / AprilTags / camera topic | PRD Out of Scope (vanilla `gz_x500` baseline only) | 03-sim-environment (M5) |
| Bag recording / `ingest` service | design §3.1 (`docker/ingest/` slot only) | 05-logging-replay (M7–M8) |
| Exact PX4 tag / `px4_msgs` branch pin | design §2 OQ-3 (settled by the spike) | MZ (spike) |
| Final ≤20-command budget allocation | design §2 OQ-6 (needs sibling run-step counts) | MZ / M2 exit (integrative item 10) |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T2.1 | Vendor `px4_msgs` + `px4_ros_com` (pinned to the PX4 tag) [C6] | `ros2_ws/src/external/px4_msgs/` (new, vendored); `ros2_ws/src/external/px4_ros_com/` (new, vendored) | C6 | Workspace/build | M | T1.2 | — |
| T2.2 | Create `patrol_interfaces` + `patrol_bringup` package shells [C7/C8, PLAT-8] | `ros2_ws/src/patrol_interfaces/` (new); `ros2_ws/src/patrol_bringup/` (new) | C7, C8 | Workspace/build | M | — | PLAT-8 |
| T2.3 | sim container: PX4 SITL + Gazebo Harmonic + uXRCE-DDS agent, headless software-render llvmpipe [C1/C3, OQ-1/OQ-4] | `docker/sim/Dockerfile` (modify); `docker/sim/entrypoint.sh` (new) | C1, C3 | Container | L | T1.3 | — |
| T2.4 | dev container + docker compose orchestration; optional `--profile gpu` [C2, OQ-5] | `docker/dev/Dockerfile` (new); `docker-compose.yml` (new) | C2, C3 | Container | L | T2.3 | — |
| T2.5 | uXRCE-DDS agent bridging → live `/fmu/*` at ~50 Hz [C4, PLAT-2] | `docker/sim/entrypoint.sh` (modify) | C4 | ROS 2 middleware/bridge | M | T2.1, T2.3 | PLAT-2 |
| T2.6 | Single green `colcon build` in-container + on CI Layer B [C5, PLAT-4, ADR-0002] | `ros2_ws/` (build entrypoint); CI Layer B `ros-ci.yml` (consumed, no change) | C5 | Workspace/build | M | T2.1, T2.2 | PLAT-4 |
| T2.7 | Finalize the pinned stack manifest [C9] | `stack-manifest.toml` (modify); `.env` (new) | C9 | Host/toolchain | S | T2.1–T2.6 | PLAT-7 |
| T2.8 | README bring-up spine (clone→flying→`/fmu` live, ≤20 commands) [C10] | `README.md` (modify) | C10 | Host/toolchain | M | T2.3–T2.7 | PLAT-6 |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Build | `docker compose` build + single `colcon build` (PLAT-3, PLAT-4) | `sim`+`dev` build from shared base; `colcon build` exit 0 incl. shells (C5/C7/C8) |
| Integration | Agent bridging (PLAT-2) | `ros2 topic list \| grep fmu` returns topics; `/fmu/out/vehicle_local_position` ~50 Hz over 60 s; `/fmu/in/*` addressable |
| CI | Layer B green (PLAT-4 AC2, ADR-0002) | `ros-ci.yml` self-activates on landed shells + vendored msgs; `colcon build`/`colcon test` green |
| E2E (demo gate) | Clean-host README reproduction (PLAT-6) | Fresh host reaches running sim in <20 commands, 0 manual deviations |
| Determinism (INF-P2) | Pinned-manifest stability (H3) | Re-run `colcon build` after unrelated upstream PX4 change → still green |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| `README.md` (spine) | developers / collaborators | clone→flying→`/fmu/*` live ≤12-command platform spine (C10) |
| `stack-manifest.toml` (final) | developers / CI | All layers pinned; `px4_version`/`px4_msgs_ref` from the spike |
| Runbook (in README) | operators (solo dev) | Bridge-down / build-fail / headless-render recovery (§4.4.5) |

### 6.3 Layered Delivery Sequence

**Skeleton + layering rationale:**

1. **M1 (skeleton — toolchain installed, vanilla SITL flying)** is the thinnest end-to-end slice and a shippable demo on its own (a flying SITL drone). The path crosses the host/toolchain + simulation/flight-stack layers at minimum thickness (no ROS, no container runtime yet — deliberate, validate the install first).
2. **M2 (layer 1: ROS 2 bridge, reproducible build & onboarding)** thickens the skeleton across every remaining layer — bridge (C4), container runtime (C1/C2/C3), workspace + vendored messages + shells (C5/C6/C7/C8), manifest (C9), README spine (C10). After M2 the demo also shows live `/fmu/*` at ~50 Hz and a clean-host README reproduction.

**What gets demoable, when:**
- After M1: a flying `gz_x500` SITL drone holding altitude 60 s.
- After M2: M1 demo + live `/fmu/*` at ~50 Hz over 60 s + clean-host README reproduction in <20 commands.
- After MZ: no new capability — consolidation; deferred backlog cleared or punted to Phase 2.

**Scope-shedding plan (within M2):** README polish (T2.8) → `dev` ergonomics (T2.4) → the `gpu` profile, in that order. **Hard floors that cannot be shed:** green `colcon build` (PLAT-4, exit item 9) and the live `/fmu/*` bridge (PLAT-2). M1 alone remains a shippable demo if the schedule slips.

**Parallel work opportunities:** vendoring + shells (T2.1/T2.2) parallelize with the `sim` entrypoint (T2.3) once the M1 spike fixes the PX4 tag. OQ-7/OQ-8 resolve out-of-band at the joint `/drive` review and do not block platform tasks (for OQ-7, 01 owns no frame convention — it only ships the vanilla `gz_x500` baseline that must not contradict 03's chosen frame; for OQ-8, 01 only provides the empty `patrol_interfaces` slot).

### 6.4 Definition of Done

A milestone is complete when:
- [ ] All tasks are implemented and code-reviewed
- [ ] All specified tests pass (build, integration, E2E, determinism)
- [ ] **Shippable demo runs end-to-end** (M1: flying SITL drone; M2: M1 + live `/fmu/*` + clean-host README reproduction)
- [ ] **Documentation true-up:** PRD / Design / DoD / README reconciled against what was actually built that milestone
- [ ] No P1 bugs remain
- [ ] Systemic interfaces (verification commands, CI Layer B) are integrated per §4.4

### 6.5 MZ — Terminal Consolidation Milestone

MZ adds no new platform capability; it absorbs work surfaced during M1–M2 that isn't blocking, and is the project's "done" gate ("project done" = MZ reviewed and cleared, or items explicitly punted to Phase 2). Seeded backlog:

| MZ item | Source | Rationale |
|---------|--------|-----------|
| Pin exact PX4 v1.16.x tag + matching `px4_msgs` branch (OQ-3 spike) | §2 OQ-3; COMBINED-REVIEW #10 | Settled by the M1–M2 integration spike (falsification gate for H2); retrofit the manifest (C9) `px4_version`/`px4_msgs_ref` + docs |
| Finalize the ≤20-command README budget across docsets (OQ-6) | §2 OQ-6; COMBINED-REVIEW | Needs sibling (02–05) run-step counts; platform spine budgeted ≤12 — allocate the remainder |
| e2e/integration test-suite expansion | DoD §4 AC-9; Linear MZ convention | Broaden the SITL/integration tier the `sim` container enables (owned by 02/05) without blocking platform |
| Documentation + test consolidation (final true-up) | Linear MZ convention | Comprehensive final PRD/Design/DoD/README + test reconciliation |

**Exit:** MZ reviewed and cleared, or items explicitly punted to Phase 2.

---

## 7. Changelog

### v0.3.0 — 2026-06-03

**Full-depth regeneration from PRD** (replaces the v0.2.0 body); no scope or decision change. All 10 components (C1–C10), all 9 FRs (PLAT-1…PLAT-9), the §2 Open-Questions resolutions (OQ-1/2/4/5 resolved; OQ-3/6 deferred; OQ-7/8 provisional-confirmed), and the §6 milestone plan (M1, M2, MZ) with the exact bootstrapped Linear task list are identical in substance to v0.2.0 — only the level of detail in the file changed.

**Sections regenerated to full depth (every block realized inline):**
- §4.2.0 inventory, §4.2.0a dependency diagram, §4.2.1–§4.2.10 per-component specs (C1–C10) with Dockerfiles, entrypoint, compose YAML, manifest TOML, topic-contract table, package-shell layouts, README spine.
- §4.2.11 inventory-triangle consistency table; §4.2.12 consumer-facing manifestation.
- §4.3 layer mapping + per-layer design notes; §4.4 systemic interfaces incl. §4.4.5 cross-cutting failure-mode table (bridge-down #1, clean-rebuild, pin-mismatch, headless-render, identity/mesh `[OOS]`).
- §4.5 three ASCII interaction sequences (happy path; bridge-down recovery; CI Layer B on a shell PR); §4.6 consolidated file-change table.
- §5 Q1–Q7 FAQ; §6.1 milestone overview table, §6.2 M1/M2 task tables (enumerating T1.1–T1.4 and T2.1–T2.8 exactly as bootstrapped to Linear), §6.5 MZ table.
- Appendix B Given/When/Then UACs.

**Codebase drift:** None — snapshot unchanged (`patrol-drone@8d03170`); the seven §3.2 Verified Preconditions re-checked and still hold.

**Key decisions:** None changed. Header bumped 0.2.0 → 0.3.0; Requirements-source and Upstream lines preserved.

### v0.2.0 — 2026-06-03

**Topics:**
- ReviewDesign (auto-pilot) D2 PRD-trace finding (Medium): the design asserted that docset 01 "establishes / owns" a world/ENU **frame convention** that OQ-7's checkpoint positions must stay compatible with. The PRD authorizes no frame-convention scope (PRD has zero ENU/NED/frame-convention language; PRD OQ-7 only says the `gz_x500` baseline "must stay compatible" with the frame the positions are expressed in), and the cross-docset README + sibling DoDs place frame ownership elsewhere (03 expresses positions in the world frame; 02 owns the NED↔world conversion boundary). The ownership over-claim was the unauthorized scope.

**Codebase drift:** None — snapshot unchanged (`patrol-drone@8d03170`); the seven §3.2 Verified Preconditions re-checked and still hold verbatim.

**Sections modified:**
- §1 Introduction ¶3 — 01 authors **no** coordinate-frame convention; ENU/NED inherited from upstream PX4/Gazebo defaults; frame + conversion-boundary ownership attributed to 03/02; 01's obligation restated as the negative "must not contradict" form.
- §2 Open Questions OQ-7 — reworded to match (drops "the world/ENU frame convention this docset establishes").
- §6.3 Parallel-work note — OQ-7 phrasing aligned.
- Header — version 0.1.0 → 0.2.0; status notes the review-revise pass.

**Key decisions:**
- Docset 01 owns only the vanilla `gz_x500` SITL baseline; it does not own, author, or establish any coordinate-frame convention. OQ-7 frame ownership stays with 03 (position frame) and 02 (NED↔world conversion). No FR-table, component, or milestone-task change — the edit is scope-language only; all 9 FRs and 10 components are unchanged.

### v0.1.0 — 2026-06-03

**Initial version** — Created via CreateDesign workflow (auto-pilot Stage 3), grounded solely in `docs/phase1/01-platform/prd.md` rev 2. Resolved design-level open questions OQ-1 (agent as in-container process), OQ-2 (PX4 from source, multi-stage), OQ-4 (headless software-render default; SITL nightly), OQ-5 (optional `gpu` compose profile). Deferred OQ-3 (PX4 tag — integration spike), OQ-6 (≤20-command budget allocation — integrative). Carried OQ-7/OQ-8 as provisional cross-docset contracts pending human confirmation. Verified preconditions against the existing repo skeleton, both ADRs, and the existing CI Layer B workflow.

---

## Appendix A: Workstream Overviews

Single-workstream docset (the platform foundation). No multi-workstream decomposition needed; the milestone breakdown (§6) is the delivery structure.

**Priority:** P1 | **Wave:** 1 | **Estimate:** ~2 ew (M1–M2, ~1 week each + integration friction)

The platform foundation is one cohesive workstream — pinned toolchain, container pair, bridge, workspace, manifest, README — delivered as a walking skeleton (M1 flies; M2 thickens to the full reproducible substrate; MZ consolidates). Key issues are the bootstrapped Linear tasks T1.1–T1.4 (M1) and T2.1–T2.8 (M2) plus the MZ backlog (§6.5).

---

## Appendix B: User Acceptance Criteria

The PRD's UACs (UAC-PLAT-1 … UAC-PLAT-8), reproduced for traceability — every one appears in the §4.1 matrix.

**UAC-PLAT-1: Vanilla SITL flight in Gazebo Harmonic**
GIVEN a clean Ubuntu 24.04 host with the pinned toolchain installed
WHEN the operator runs `make px4_sitl gz_x500` and arms/takes off from QGroundControl
THEN a drone launches in Gazebo Harmonic, takes off, and holds altitude within hover tolerance for at least 60 continuous seconds with no plugin or dependency errors.
*(Design coverage: C1, §4.2.1; Sequence 1, §4.5.)*

**UAC-PLAT-2: Live PX4 telemetry over uXRCE-DDS**
GIVEN SITL is running with the Micro XRCE-DDS Agent bridged to PX4 over UDP localhost
WHEN a node author runs `ros2 topic list | grep fmu` and `ros2 topic hz /fmu/out/vehicle_local_position`
THEN PX4 topics are returned and `/fmu/out/vehicle_local_position` reports a steady ~50 Hz publish rate sustained over a 60 s window, and the `/fmu/in/*` command surface is addressable.
*(Design coverage: C4, §4.2.4; Sequences 1+2, §4.5.)*

**UAC-PLAT-3: Reproducible containerized build**
GIVEN a clean checkout of the repo on a collaborator's workstation
WHEN they run `docker compose` to build the `dev` and `sim` containers
THEN both build successfully from the shared Ubuntu 24.04 + ROS 2 Jazzy base with no manual host edits, with `sim` carrying SITL+Gazebo+agent+workspace and `dev` mounting source as a volume, and with no x86-host-specific assumptions baked into the build steps.
*(Design coverage: C1/C2/C3, §4.2.1–§4.2.3.)*

**UAC-PLAT-4: Single `colcon build` succeeds inside the container**
GIVEN a built container with the `ros2_ws` workspace
WHEN a node author runs a single `colcon build` inside it
THEN the build succeeds with no errors (including the `patrol_interfaces`/`patrol_bringup` shells) and the same build is green on CI Layer B.
*(Design coverage: C5, §4.2.5; Sequence 3, §4.5.)*

**UAC-PLAT-5: Vendored, version-pinned `px4_msgs`**
GIVEN the workspace
WHEN it is built
THEN `px4_msgs` is present under `ros2_ws/src/external/px4_msgs`, vendored and pinned to the chosen PX4 branch (not pulled at build time), and builds as part of the workspace alongside vendored `px4_ros_com`.
*(Design coverage: C6, §4.2.6.)*

**UAC-PLAT-6: Setup-to-running-mission README (≤20 commands)**
GIVEN a collaborator on a clean machine
WHEN they follow the README from setup to a running mission
THEN the path is fully documented and executable in fewer than 20 commands, with the platform bring-up spine fitting within the shared budget and no step required that is not in the README.
*(Design coverage: C10, §4.2.10; OQ-6 budget allocation.)*

**UAC-PLAT-7: Pinned stack manifest**
GIVEN the pinned stack manifest
WHEN any toolchain layer is referenced (OS, ROS 2 Jazzy, PX4 v1.16.x, Gazebo Harmonic, uXRCE-DDS, Python 3.12, MCAP plugin, colcon, Docker)
THEN its version is explicitly pinned in the manifest and the manifest is the cited source of truth.
*(Design coverage: C9, §4.2.9.)*

**UAC-PLAT-8: Workspace package shells as landing slots**
GIVEN the `ros2_ws` workspace
WHEN it is built
THEN the empty-but-present `patrol_bringup` and `patrol_interfaces` package shells exist in `ros2_ws/src/` and build as part of `colcon build`, with their contents left to be owned by 02 and 04.
*(Design coverage: C7/C8, §4.2.7–§4.2.8.)*

### Inferred Requirements [INFERRED]

**INF-P1: Clean rebuild recovery** *(ref: UAC-PLAT-4)*
GIVEN a corrupted or stale colcon build/install tree
WHEN a developer runs `rm -rf build/ install/ log/ && colcon build`
THEN the workspace rebuilds green from a clean state.
*(Design coverage: §4.2.5, §4.4.5.)*

**INF-P2: Pinned-manifest determinism on upstream PX4 change** *(ref: UAC-PLAT-5; PRD H3 signal)*
GIVEN the vendored, pinned `px4_msgs` under `src/external/`
WHEN an unrelated upstream PX4 change lands and `colcon build` is re-run
THEN the build still succeeds because the vendored copy is unchanged.
*(Design coverage: C6 + C9, §4.2.6/§4.2.9.)*
