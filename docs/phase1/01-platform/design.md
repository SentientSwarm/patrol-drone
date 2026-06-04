# Platform & Simulation Foundation — System Design Working Document

**Status:** Draft (rev — ReviewDesign auto-revise pass applied)
**Version:** 0.2.0
**Date:** 2026-06-03
**Projects:** Autonomous Drone Patrol — Phase 1 Docset 01 (Platform & Simulation Foundation)
**Authors:** jxstanford@wemodulate.energy (solo dev / DRI)

> **Requirements source (sole):** `docs/phase1/01-platform/prd.md` (rev 2). This design realizes that PRD's FR table (PLAT-1 … PLAT-9) as components, container/workspace layout, and milestone tasks. It specifies *how*; it does not introduce surface area beyond the PRD's FR table (per the PRD's Scope Authority note). Where a `how` decision is genuinely open, it is recorded in §2, not silently invented.

---

## 1. Introduction

This design covers the pre-hardware **platform substrate** for Phase 1 of the autonomous-drone project: a pinned, containerized PX4-SITL + Gazebo-Harmonic + ROS 2 Jazzy stack that flies a vanilla `gz_x500` drone, exposes PX4 as native `/fmu/*` ROS 2 topics over the uXRCE-DDS bridge, builds cleanly with a single `colcon build` inside a `sim`/`dev` container pair, and is documented end-to-end in a ≤20-command README. It is the foundation every other Phase 1 docset (02–05) and every later hardware phase (2–8) stands on. It is infrastructure, not a user-facing feature — its entire value is downstream reproducibility.

The design is deliberately **right-sized for simulation Phase 1**. There is no application database, no REST/SDK surface, no multi-tenant auth, and no production runtime. The "components" are containers, a colcon workspace, vendored message packages, a process-level bridge, a pinned-version manifest, and a README — plus the two empty package shells (`patrol_bringup`, `patrol_interfaces`) that siblings land their contents into. The "layers" are the host/toolchain layer, the container layer, the simulation/flight-stack layer, the ROS 2 middleware/bridge layer, and the workspace/build layer. The "systemic interfaces" are developer-facing verification commands and the existing two-layer CI (ADR-0002), not a monitoring stack.

Two cross-docset contracts touch this docset even though it does not own either: the checkpoint-mapping schema (owned by 03-sim-environment) and the `CheckpointCapture` message representation (owned by 04-perception; this docset provides only the empty `patrol_interfaces` landing slot the message lands in). On the first: this docset does **not** author or own any coordinate-frame convention — it ships only the vanilla `gz_x500` SITL baseline, and the ENU (Gazebo world) / NED (PX4 offboard) frames are inherited from upstream PX4/Gazebo defaults, not defined here. The frame in which 03's checkpoint positions are expressed, and the NED↔world conversion boundary, are owned by 03 and 02 respectively (02's DoD: "PX4 offboard uses NED relative to the EKF origin … conversion happens at one explicit boundary"). 01's only obligation is the negative one: the `gz_x500` baseline it ships must not contradict whatever frame those positions are ultimately expressed in. Both contracts are carried here as provisional, flagged decisions pending human confirmation (§2, OQ-7/OQ-8), matching the PRD's open-questions table.

### Source Projects (Linear)

| # | Project | Est. | Wave |
|---|---------|------|------|
| 1 | Phase 1 Docset 01 — Platform & Simulation Foundation (no Linear project yet; bootstrap is a separate `/drive` step) | ~2 ew (M1–M2, ~1 week each + integration friction) | 1 |

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

All items below mirror the PRD's Open Questions table (OQ-1 … OQ-8) — the design is the resolution venue for the `how`-shaped ones (OQ-1, OQ-2, OQ-4, OQ-5, OQ-6) and carries OQ-3 (integration-spike-gated) and OQ-7/OQ-8 (cross-docset contracts owned elsewhere) as deferrals. Each design resolution is recorded with rationale.

| # | Item | Source | Status / Resolution | Decision target |
|---|------|--------|---------------------|-----------------|
| OQ-1 | Two-container (`sim`/`dev`) split sufficient, or does the uXRCE-DDS agent warrant its own compose service? | PRD OQ-1 | **Resolved (design):** keep the two-container split; run the agent as a **process inside the `sim` container** (its own compose-managed lifecycle is unnecessary in SITL where the client transport is automatic). The agent is one process in the `sim` container's entrypoint, not a third image. Rationale: minimal surface, matches the DoD's settled two-container constraint, and the agent has no independent scaling or failure-isolation need in Phase 1. Revisit if a sibling needs the agent without SITL. | — (resolved) |
| OQ-2 | Build PX4 from source in the `sim` image, or layer a prebuilt SITL artifact for faster CI? | PRD OQ-2 | **Resolved (design):** **build PX4 from source** inside the `sim` image at a pinned tag, in a multi-stage Dockerfile (build stage → slim runtime stage) so the source toolchain does not bloat the runtime layer. Rationale: reproducibility is the project tenet ("Reproducibility over convenience"); a pinned source build is the source of truth, and the multi-stage split + a layer-cached build stage keeps rebuild cost bounded. The image is built once and cached; per-PR CI Layer B does **not** rebuild PX4 (it builds only the colcon workspace), so CI runtime is unaffected. | — (resolved) |
| OQ-3 | Which exact PX4 v1.16.x tag and matching `px4_msgs` branch to vendor? | PRD OQ-3 | **Deferred:** cannot be picked on paper; settled by the M1–M2 integration spike. The manifest (§4.2.9) carries a `px4_version` / `px4_msgs_ref` pair as the single edit point; the spike fills it with a known-good combination. Also the falsification gate for the PRD's H2. | End of M1–M2 integration spike |
| OQ-4 | Headless rendering backend for CI — software/llvmpipe vs hosted-runner GPU for Gazebo Harmonic's Vulkan? | PRD OQ-4 | **Resolved (design) for Phase 1 scope:** the `sim` container supports a **headless software-rendering path** (llvmpipe / `LIBGL_ALWAYS_SOFTWARE`-style env, no display) as the default CI-capable mode (PLAT-9). The hosted-runner-GPU question is moot for *required* CI because SITL stays a **nightly scaffold** (ADR-0002, `sitl-nightly.yml`), never a per-PR gate. The platform only owns "the `sim` container *can* run headless"; which runner the nightly SITL tier eventually uses is owned by the CI / sibling integration tier. | — (resolved for platform scope; nightly-runner choice deferred to 02/05) |
| OQ-5 | How (and whether) to expose the host GPU to Gazebo in the container for interactive dev? | PRD OQ-5 | **Resolved (design):** GPU passthrough is an **optional, swappable compose profile** (`docker compose --profile gpu`) that adds the NVIDIA Container Runtime device reservation; the default profile is CPU/software-rendering. Rationale: collaborator GPUs range 4–8 GB and some have none usable in-container; passthrough must never be a hard requirement. The runtime is the same swappable parameter the Jetson image will flip (PLAT-3 AC3). | — (resolved) |
| OQ-6 | How is the ≤20-command README budget split between platform spine and per-docset run steps? | PRD OQ-6 | **Deferred (integrative):** the platform owns the bring-up *spine* and budgets it explicitly (§4.2.10 targets ≤12 commands for clone→flying→`/fmu/*` live, leaving ≥8 for siblings); the final allocation needs sibling run-step counts and is settled at the M2 exit / item-10 integration. | M2 exit (item 10, integrative) |
| OQ-7 | Checkpoint-mapping schema (`sim/config/checkpoints.yaml`: `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`). | PRD OQ-7 | **Provisional default applied, pending user confirmation.** Owned by **03**; 01 is not an owner and **authors no coordinate-frame convention** — it ships only the vanilla `gz_x500` SITL baseline. The ENU (Gazebo world) / NED (PX4 offboard) frames are inherited from upstream PX4/Gazebo defaults; the frame the positions are expressed in is owned by 03, and the NED↔world conversion boundary is owned by 02 (per 02's DoD). 01's only obligation is the negative one: the `gz_x500` baseline must not contradict the frame 03's positions are expressed in. Recorded so the 5 docset pairs stay coherent. **Needs human confirmation.** | Joint `/drive` review (03 owns) |
| OQ-8 | `CheckpointCapture` representation — `string image_path` + header/checkpoint_id/pose/metadata by reference (NOT pixels by-value); a separate `sensor_msgs/CompressedImage` topic carries live frames for the bag. | PRD OQ-8 | **Provisional default applied, pending user confirmation.** Owned by **04**; lands in 01's `patrol_interfaces` shell. 01 owns only the empty landing slot (§4.2.8) — the slot must *accept* this message but defines none of its fields. Recorded so the 5 docset pairs stay coherent. **Needs human confirmation.** | Joint `/drive` review (04 owns) |

**Assumptions** (explicit, design-level):
- **A1:** PX4 v1.16's bundled `uxrce_dds_client` auto-starts in SITL with the correct UDP-localhost transport (PRD Technical Considerations; verified in §3.2 against upstream docs). If the spike (OQ-3) finds a tag where this is not automatic, the `sim` entrypoint adds an explicit `uxrce_dds_client start` — a one-line entrypoint change, not a design change.
- **A2:** `patrol_interfaces` is an `ament_cmake` (C++/IDL) package so it can host ROS message definitions; `patrol_bringup` is an `ament_python` package (launch/config host). This matches the repo's existing `ros2_ws/README.md` package plan (§3.2).
- **A3:** The empty-but-present shells must build under `colcon build` on the current empty workspace; the CI Layer B job (`ros-ci.yml`) already no-ops green on the empty tree and self-activates as packages land (ADR-0002), so the shells are the first packages that activate it.

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
├── ros2_ws/                           ┌─ Workspace/build layer (this docset owns the baseline)
│   └── src/   (.gitkeep)              │   colcon workspace; patrol_* shells + external/
├── sim/                               ┌─ Sim-asset layer (03 owns worlds/models; 01 ships vanilla gz_x500 only)
│   ├── worlds/ models/ px4_sitl_overrides/
├── .github/workflows/                 ┌─ CI layer (ADR-0002 — already built)
│   ├── python-quality.yml (Layer A)   │   fast pure-Python gates
│   ├── ros-ci.yml         (Layer B)   │   colcon build + colcon test  ← platform build green here
│   └── sitl-nightly.yml   (scaffold)  └─  SITL nightly, never per-PR
├── docs/decisions/0001…, 0002…        ← settled constraints (cited, not relitigated)
└── README.md                          ← integrative bring-up spine (this docset owns)
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
**Implication:** Every component below is pinned to these choices; the manifest (§4.2.9) is where the exact versions live.

### 3.4 Codebase Snapshot

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

### 4.2 Component Architecture

*(Full §4.2.0 component inventory, §4.2.0a dependency diagram, and per-component specs C1–C10 unchanged from v0.1.0 — the canonical text is in the on-disk file `docs/phase1/01-platform/design.md`. Inventory triangle (§4.2.11) confirmed consistent: §4.2.0 inventory ↔ §4.2.0a diagram ↔ consumer-facing manifestation all enumerate C1–C10. C4 owns the `/fmu/*` topic contract — `/fmu/out/vehicle_local_position` (`px4_msgs/VehicleLocalPosition`) steady ~50 Hz over 60 s, the `/fmu/out/*` set, addressable `/fmu/in/*` — consumed by 02/05. C8 `patrol_interfaces` is the empty `ament_cmake` landing slot that must accept OQ-8's `CheckpointCapture` shape without 01 defining any field. C9 manifest is the single pinned-version source of truth with `px4_version`/`px4_msgs_ref` as the OQ-3 spike's single edit point.)*

### 4.3 Layer View

Layers: Host/toolchain (C9, C10) · Container (C1, C2, C3) · Simulation/flight stack (C1) · ROS 2 middleware/bridge (C4) · Workspace/build (C5, C6, C7, C8) · CI (consumed — ADR-0002 Layer B `ros-ci.yml`). Layer definitions match the repo skeleton (§3.1), not invented. *(Full §4.3 unchanged from v0.1.0; canonical text on disk.)*

### 4.4 Systemic / Platform Interfaces

Observability is dev-facing verification commands (`ros2 topic list/hz`, `docker compose`/`colcon build` exit codes, CI Layer B job status); no custom structured logging. CI integration consumes the existing `ros-ci.yml` (no CI change — the platform lands the first packages that activate the guarded job). Security is correctly N/A for a local single-user sim (UDP-localhost-only agent, `.env` secret hygiene, no auth surface). §4.4.5 cross-cutting failure modes table includes the #1 uXRCE-DDS bridge-down recovery; identity-provider and mesh sub-modes marked explicitly OOS for Phase 1. *(Full §4.4 unchanged from v0.1.0; canonical text on disk.)*

### 4.5 Key Interaction Sequences

Three sequences: (1) clean checkout → flying SITL + live `/fmu/*` (happy path, PLAT-1+2+3+4); (2) uXRCE-DDS bridge-down recovery (error path); (3) CI Layer B on a PR that lands a shell (PLAT-4 AC2, PLAT-8). *(Full ASCII sequences unchanged from v0.1.0; canonical text on disk.)*

### 4.6 Data Model Changes (Consolidated)

No application database or schema. Persisted artifacts are source-controlled files (container definitions, vendored message source, pinned manifest, README) plus ephemeral build outputs. New/modified files: `docker/{sim,dev}/` (replace `.gitkeep`), `docker-compose.yml` (new), `ros2_ws/src/external/{px4_msgs,px4_ros_com}` (vendored), `ros2_ws/src/{patrol_bringup,patrol_interfaces}` (shells), stack manifest (new), `README.md` (placeholder → spine). *(Full table unchanged from v0.1.0; canonical text on disk.)*

### 4.7 UX Mocks

Not applicable — no GUI. The "interface" is the command line and file tree; the closest analog is the README spine (§4.2.10) and verification-command outputs (§4.4.2). QGroundControl (the only M1-path GUI) is an external desktop app, explicitly not owned/containerized here.

---

## 5. Design Questions FAQ

Q1 (components/interactions), Q2 (owned contract = the `/fmu/*` ROS 2 topic surface; no REST/SDK/DB; `CheckpointCapture` not defined here, only its landing slot C8 must accept it), Q3 (infra = local dev hosts + existing CI; manifest pins), Q4 (external deps: PX4/ROS 2/Gazebo/QGC/CI), Q5 (testing = build/integration + E2E demo gates; SITL stays nightly), Q6 (security minimal by nature — UDP-localhost-only, `.env` hygiene, no principals), Q7 (top risks each tie to a §2 OQ; cross-docset contracts OQ-7/OQ-8 are Provisional, need human confirmation, owned by 03/04). All Q7 Open/Provisional/Deferred statuses match a §2 OQ row. *(Full Q1–Q7 prose unchanged from v0.1.0; canonical text on disk.)*

---

## 6. Implementation Plan

Two milestones mirroring the DoD's M1/M2, walking-skeleton structure:

- **M1 (skeleton — toolchain installed, vanilla SITL flying):** thinnest end-to-end slice — pinned toolchain that flies `gz_x500` and hovers 60 s via QGC (PLAT-1). No ROS/containers yet (deliberate — validate the install before layering architecture). Demo gate: stakeholder runs `make px4_sitl gz_x500`, arms/takes off, watches 60 s hover. Tasks T1.1–T1.4 (toolchain+PX4 source build; draft manifest; scaffold `sim` build stage; first README fragment).
- **M2 (layer 1: ROS 2 bridge, reproducible build & onboarding):** thickens the skeleton across every remaining layer — vendored `px4_msgs`/`px4_ros_com` (C6), agent bridging + `/fmu/*` (C4), `sim`+`dev` from compose incl. headless (C1/C2/C3, PLAT-9), single green `colcon build` in-container + on CI Layer B (C5, PLAT-4), `patrol_*` shells (C7/C8), README spine (C10), finalized manifest (C9). Demo gate: live `/fmu/*` at ~50 Hz over 60 s + clean-host README reproduction in <20 commands. Tasks T2.1–T2.8 with Files-Touched populated for L/XL tasks, per-milestone Out-of-Scope tables with Source citations, testing + documentation tables.

§6.3 Layered Delivery: M1 is a shippable demo on its own (flying SITL drone). Scope-shedding order within M2: README polish (T2.8) → `dev` ergonomics (T2.4) → `gpu` profile. **Hard floors that cannot be shed:** green `colcon build` (PLAT-4, exit item 9) and the live `/fmu/*` bridge (PLAT-2). Parallel: vendoring+shells (T2.1/T2.2) parallelize with `sim` entrypoint (T2.3) once the M1 spike fixes the PX4 tag; OQ-7/OQ-8 resolve out-of-band at the joint `/drive` review and do not block platform tasks (for OQ-7, 01 owns no frame convention — it only ships the vanilla `gz_x500` baseline that must not contradict 03's chosen frame; for OQ-8, 01 only provides the empty `patrol_interfaces` slot). §6.4 Definition of Done includes the shippable-demo gate per milestone. *(Full §6.1/§6.2 tables unchanged from v0.1.0 except §6.3 OQ-7 phrasing; canonical text on disk.)*

---

## 7. Changelog

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

**Cascade Hygiene Checklist — revision v0.2.0:** all rows (§2, §3, §3.2, §4.1, §4.2.0, §4.2.0a, §4.2.{N}, §4.3, §4.4.1, §4.4.{N}, §4.4.5, §4.5, §4.6, §4.7, §5 Q1–Q7, §6.1, §6.2 FR-labels, §6.2 OOS, §6.2 Files-Touched, §6.3, §6.4, §7, AppB, INF) marked `[updated]` or `[no-change]` with one-line rationale — zero `[ ]` blockers. Edits confined to §1/§2-OQ7/§6.3/§7/header; everything else verified clean. §6.2 milestone-task FR labels re-audited: every task references PLAT-1…9, all present in the PRD FR table.

### v0.1.0 — 2026-06-03

**Initial version** — Created via CreateDesign workflow (auto-pilot Stage 3), grounded solely in `docs/phase1/01-platform/prd.md` rev 2. Resolved design-level open questions OQ-1 (agent as in-container process), OQ-2 (PX4 from source, multi-stage), OQ-4 (headless software-render default; SITL nightly), OQ-5 (optional `gpu` compose profile). Deferred OQ-3 (PX4 tag — integration spike), OQ-6 (≤20-command budget allocation — integrative). Carried OQ-7/OQ-8 as provisional cross-docset contracts pending human confirmation. Verified preconditions against the existing repo skeleton, both ADRs, and the existing CI Layer B workflow.

---

## Appendix A: Workstream Overviews

Single-workstream docset (the platform foundation). No multi-workstream decomposition needed; the milestone breakdown (§6) is the delivery structure.

---

## Appendix B: User Acceptance Criteria

The PRD's UACs (UAC-PLAT-1 … UAC-PLAT-8), reproduced for traceability — every one appears in the §4.1 matrix. Plus two inferred requirements: **INF-P1** (clean rebuild recovery via `rm -rf build/ install/ log/ && colcon build`, design coverage §4.4.5) and **INF-P2** (pinned-manifest determinism on upstream PX4 change — the PRD's H3 signal as an acceptance check, design coverage C6+C9). *(Full Given/When/Then text unchanged from v0.1.0; canonical text on disk at `docs/phase1/01-platform/design.md`.)*

---

> **Note on this rendering:** the authoritative, fully-expanded design (all §4.2 per-component specs, §4.3 layer notes, §4.4 interface tables, §4.5 ASCII sequences, §6.1/§6.2 milestone tables, and Appendix B Given/When/Then text verbatim) is the on-disk file `/Users/jxstanford/devel/SentientSwarm/patrol-drone/docs/phase1/01-platform/design.md` at v0.2.0. This structured rendering reproduces the header, §1–§3, §4.1 matrix, §7 changelog, and Appendix A in full, and summarizes the sections the v0.2.0 revision did not touch to avoid divergence; those sections are byte-for-byte the v0.1.0 content that passed self-review clean at the medium floor.
