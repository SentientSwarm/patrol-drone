# Simulation Environment & Assets (Phase 1 / M5)

> **One-liner:** A checked-in Gazebo Harmonic patrol world with AprilTag checkpoints at YAML-configured positions and a drone-mounted RGB camera, so the M4 patrol mission can visit each checkpoint and "look at" it — the physical stage that lets perception (M6) and logging (M7) be exercised end-to-end before any hardware exists.

**Date:** 2026-06-03
**Status:** Draft (rev 2 — post self-review)
**Owner:** Project owner (solo dev)
**DRI:** jxstanford@wemodulate.energy

**Tier:** Standard · **Source:** [`dod.md`](./dod.md) (Formal Definition-of-Done) → M5 of [`docs/phase1_simulation_plan.md`](../../phase1_simulation_plan.md)

> **Tier note:** The DoD's assessment signals suggest Lightweight (Complexity moderate × Risk low). This PRD is authored at **Standard** because the docset owns three cross-service interface contracts (checkpoint-config schema, camera topic, AprilTag asset family) that three sibling docsets (02, 04, 05) bind to; the contract coordination is what argues the tier upward. All Standard-tier sections are populated accordingly.

## Changelog

| Rev | Date | Change |
|-----|------|--------|
| 1 | 2026-06-03 | Initial draft generated from DoD. |
| 2 | 2026-06-03 | Self-review (ReviewPRD) revisions: (a) SIM-1 — demoted the "near-real-time physics" clause from a binding acceptance criterion to a non-binding target tied to H1 (it had no measurement contract); (b) SIM-3 — named the companion `sensor_msgs/CompressedImage` topic at the FR-statement level (not only in an AC) since 05 binds to it; (c) Success Metrics — added an INFERRED near-real-time performance row tied to H1 so the metric set covers the one quantitative hypothesis. |

## Overview

This docset delivers the simulation environment for Phase 1: a custom Gazebo Harmonic (gz-sim 8) world representing a crude-but-meaningful patrol environment, AprilTag fiducial markers placed as textured Gazebo models at known checkpoint positions, and a simulated RGB camera attached to the PX4 SITL drone that publishes a ROS 2 image topic. A single shared YAML config defines where each checkpoint tag sits in the world and how it maps to a checkpoint identifier — the same map the patrol mission (02) flies to and perception (04) reads to name a detected tag.

It owns no Phase 1 exit-checklist item solo. Its value is being the stage the rest of M5–M8 run on: it is a hard dependency for the integrative end-to-end patrol (item 1), the Foxglove camera-feed render (item 8), and the `CheckpointCapture` pipeline (item 11). Per the plan, the work is "start crude" — bounded authoring (one world, a few tag models, one camera sensor, one config schema), not algorithm-heavy. The cross-service interface contracts it defines — the checkpoint-config schema, the camera topic, and the AprilTag asset family — are what justify the Standard tier: three sibling docsets (02, 04, 05) bind to them and they must stay coherent.

## Problem Statement

> **When** the developer wants to validate the integrated patrol-plus-perception path in simulation,
> **they struggle with** the fact that PX4 SITL ships only a vanilla empty plane with no checkpoints, no fiducial markers, and no camera on the drone,
> **which means** there is nowhere to place checkpoints, no environment to patrol, and no image feed — so M6 (perception capture) and M7 (logging) have no stage to be built or tested on, and nothing about the end-to-end path can be proven before hardware is purchased.

Today, M1–M4 produce a drone that can fly a YAML waypoint patrol in an empty world. That proves flight control and mission logic but proves nothing about checkpoints or perception, because the world has no checkpoints to visit and the drone has no camera to look with. The only workaround — hand-editing tag positions into raw SDF and bolting a camera onto a stock airframe ad hoc — buries the checkpoint positions where the mission (02) and perception (04) can't share them, and produces assets that won't transfer to the Phase 4 hardware checkpoints. Phase 1's discipline is "make the basic patrol work end-to-end in sim first"; M5 is the milestone that makes "end-to-end" include checkpoints and imagery, and it is sequenced now because M6 and M7 are blocked on it.

## Goals

### Business goals
- Unblock the Phase 1 exit path: provide the world + checkpoints + camera that the integrative end-to-end patrol (exit item 1), the Foxglove camera render (item 8), and the `CheckpointCapture` pipeline (item 11) all depend on.
- Establish a sim/hardware-aligned AprilTag asset and detection-settings choice that carries unchanged into Phase 4 indoor relocalization, avoiding a re-author later.

### User goals
- The developer can launch SITL against the project patrol world and see a recognizable environment with checkpoint markers, instead of an empty plane.
- The developer can move or add a checkpoint by editing one YAML file, and the world reflects it on the next launch.
- The perception node (04) and logging pipeline (05) can subscribe to a live drone camera feed exactly as a real camera would feed them.
- The operator can run the M4 patrol against the world's checkpoint positions and watch the drone visit each tagged checkpoint in turn.

### Non-goals
- Photorealistic / high-fidelity environments — deferred; M5 explicitly says "the point isn't photorealism" (target: Phase 5, Isaac Sim).
- AprilTag-based relocalization / VIO / SLAM — deferred; the simulator gives ground-truth pose, tags are detection targets only (target: Phase 3/4).
- The AprilTag detection node and `CheckpointCapture` message — owned by 04-perception, not this docset.
- Mission/waypoint logic and the state machine — owned by 02-mission-control, not this docset.

> Brief non-goals above are orientation. The contract-level deferrals (with rationale and target milestone) live in the **Out of Scope** section below.

## Out of Scope

> Items explicitly **not** part of this docset's M5 deliverable. Listing them here (rather than omitting them) is the contract for what this PRD does NOT authorize the design to build.

| Item | Status | Rationale | Target | Added |
|------|--------|-----------|--------|-------|
| Photorealistic / high-fidelity environments | Deferred | M5: "start crude … the point isn't photorealism"; higher fidelity needs Isaac Sim | Phase 5 (trail navigation) | 2026-06-03 |
| Isaac Sim as the simulator | Out of scope | Gazebo Harmonic is sufficient for Phase 1; Isaac Sim is for trail data and RL | Phase 5+ / Phase 8 | 2026-06-03 |
| Using AprilTags for relocalization / VIO / SLAM | Deferred | Sim provides ground-truth pose; tags exercise the perception pipeline only | Phase 3 (optional fusion) / Phase 4 (indoor VIO) | 2026-06-03 |
| AprilTag detection node + `CheckpointCapture` message | Out of scope | Owned by 04-perception; this docset provides only the tags + camera feed it consumes | This phase, docset 04 | 2026-06-03 |
| Mission/waypoint logic + state machine driving the drone | Out of scope | Owned by 02-mission-control; this docset provides only checkpoint positions to patrol between | This phase, docset 02 | 2026-06-03 |
| Real object detection (YOLO / TensorRT) | Deferred | AprilTag scaffolding is sufficient for Phase 1 | Phase 3 | 2026-06-03 |
| Camera-frame recording into the MCAP bag | Out of scope | Owned by 05-logging-replay; this docset only publishes the live camera + `CompressedImage` topics 05 records | This phase, docset 05 | 2026-06-03 |

## Key Hypotheses

- **H1:** We believe authoring the world from primitives (boxes, simple tree props) and AprilTag-textured planes — rather than large binary meshes — will keep physics fast enough to run a multi-checkpoint patrol smoothly on the Phase 1 minimum spec (6-core / 16 GB / 4 GB VRAM, per the plan's dev-hardware table), because a single drone in a simple primitive world is light for Gazebo Harmonic's Vulkan renderer. *Signal: SITL + the world + patrol runs at or near real-time without dropped physics frames on a minimum-spec host. [INFERRED — no explicit perf target is given in the DoD; the near-real-time signal is a derived, sim-only check, not a contract. It is tracked as a non-binding success-metric row, not as a binding acceptance criterion.]*
- **H2:** We believe driving checkpoint positions from a single shared YAML — rather than hard-coding them in SDF — will let 02 (waypoints) and 04 (tag→checkpoint mapping) bind to the same source of truth without divergence, because all three consumers read the same `checkpoint_id ↔ position ↔ tag_id` rows. *Signal: moving a checkpoint by editing only the YAML changes the world position, the patrol waypoint, and the perception mapping consistently, with no SDF edit.*
- **H3:** We believe a tag36h11 family at a checkpoint-appropriate physical size, with a chosen camera resolution/FOV/mount pose, will let the drone reliably "see" and resolve a tag while hovering at a checkpoint, because tag36h11 is the robust default for `apriltag_ros` and the geometry can be tuned in sim cheaply. *Signal: at each checkpoint hover, the camera frame contains a fully-resolvable tag (validated downstream by 04's detection in M6).*

## Tenets

> Tie-breakers when a decision is ambiguous during implementation — *unless you know better ones.*

1. **Crude over pretty.** When fidelity trades against simplicity or sim performance, choose the simpler asset. The point is known checkpoint positions, not realism.
2. **One source of truth for checkpoints.** When position data could live in two places, it lives in the shared YAML and everything else reads from it — never duplicate a checkpoint position into SDF and YAML.
3. **Sim/hardware parity for tags.** When an AprilTag choice (family, ID range, size, detector settings) trades sim convenience against running unmodified on Phase 4 hardware, choose the option that transfers.
4. **Repo-portable, no host paths.** When referencing an asset, reference something checked into `sim/` with a relative/package path — never a host-specific or absolute path. The world must load on a fresh checkout in-container.
5. **Contract before convenience.** When the camera topic or checkpoint schema could be shaped for this docset's ease vs. its three consumers' needs, shape it for the consumers (02, 04, 05).

## Functional Requirements

> **Scope note:** This docset specifies no REST endpoints or SDK module paths, so the Path & SDK convention table does not apply. The contracts it owns are ROS 2 topic names/types, SDF/model asset paths, and a YAML config schema; these are named per-FR below and consolidated in the Cross-Service Impact section.

### P1: Critical (must ship)

#### SIM-1: Custom patrol world loads in Gazebo Harmonic under SITL
The system SHALL provide a checked-in Gazebo Harmonic (gz-sim 8) world — flat terrain plus a few building-like boxes and a few trees as obstacles — that loads cleanly when PX4 SITL is launched against it.

**Customer scenario:** The developer launches SITL against the project patrol world instead of a vanilla empty plane and sees a recognizable patrol environment.

**Pain removed:** Without a known-geometry world there is nowhere to place checkpoints and no environment to patrol — the rest of M5–M7 has no stage.

**Acceptance criteria:**
- Launching PX4 SITL against the world renders flat terrain + building-like boxes + trees in Gazebo Harmonic with no load errors.
- The world file lives under `sim/worlds/` and is checked into the repo.
- World extent and obstacle layout are "meaningful" — enough geometry to patrol between (≥3 checkpoints' worth of separation and at least one box/tree obstacle).
- *Target (non-binding, per H1):* on the Phase 1 minimum-spec host the world + SITL + patrol runs at or near real-time without dropped physics frames. This is a derived sim-only performance target tracked in Success Metrics, **not** a binding pass/fail acceptance gate — the DoD sets no perf contract. **[INFERRED — see H1.]**

**Trace:** UAC-SIM-1 (Appendix B)

#### SIM-2: AprilTag checkpoint markers at YAML-configured positions
The system SHALL place at least 3 AprilTag fiducial markers — as textured Gazebo models — in the world at positions defined by a shared YAML checkpoint config, each mapped to a checkpoint identifier.

**Customer scenario:** The developer edits one YAML file to move or add a checkpoint and the world reflects it on the next launch.

**Pain removed:** Hard-coded tag positions buried in SDF would block perception (M6) from being exercised end-to-end and would not transfer to the Phase 4 hardware checkpoints.

**Acceptance criteria:**
- A checkpoint-config YAML with ≥3 entries drives the placement of ≥3 AprilTag checkpoint markers at the configured world (ENU) positions.
- Each YAML entry carries `checkpoint_id`, `position {x,y,z}`, `tag_family`, and `tag_id` (the shared schema in the Cross-Service Impact section).
- The AprilTag markers are textured Gazebo models under `sim/models/`, checked into the repo.
- Editing a checkpoint's position in the YAML and relaunching moves the corresponding marker — no SDF edit required to reposition.

**Trace:** UAC-SIM-2 (Appendix B)

#### SIM-3: Simulated RGB camera publishes ROS 2 image topics
The system SHALL attach a simulated RGB camera sensor to the SITL drone that publishes a `sensor_msgs/Image` topic (the live frame stream 04 subscribes to) and a companion `sensor_msgs/CompressedImage` topic (the frame stream 05 records into the bag), both visible to ROS 2 at a steady rate while SITL runs.

**Customer scenario:** The perception node (04) subscribes to a live drone camera feed and the logging pipeline (05) records the compressed feed, exactly as a real camera would feed them.

**Pain removed:** With no camera topic, perception capture and bag recording of imagery cannot be built or tested at all.

**Acceptance criteria:**
- With SITL running, `ros2 topic list` shows the camera image topic and `ros2 topic hz <topic>` shows a steady, non-zero rate.
- The topic publishes `sensor_msgs/Image`; a companion `sensor_msgs/CompressedImage` topic is also published for the bag. **[INFERRED — the CompressedImage companion is from the settled cross-docset contract default + M7's "compressed image to keep bag size manageable"; not stated in this docset's DoD body. Pending user confirmation.]**
- The camera topic name(s), `frame_id`, resolution, and frame rate are fixed and documented as the contract 04 and 05 bind to (see Cross-Service Impact).
- The camera is attached via assets under `sim/px4_sitl_overrides/` (or world/model SDF) checked into the repo — no host-specific airframe modification.

**Trace:** UAC-SIM-3 (Appendix B)

#### SIM-4: Camera mount/FOV lets the drone see a tag at a checkpoint
The system SHALL mount the camera on the airframe with a pose and field-of-view such that, when the drone hovers at a configured checkpoint per the approach geometry, a checkpoint AprilTag is within the camera frame and large enough to be resolvable.

**Customer scenario:** The operator runs the patrol and, at each checkpoint hover, the drone is actually looking at the tag — the camera frame contains the marker, not empty sky or ground.

**Pain removed:** A camera that can't see the tag at the hover pose makes the entire perception-capture path (M6) untestable and silently breaks the waypoint approach geometry 02 depends on.

**Acceptance criteria:**
- The camera mount pose and FOV are fixed and documented (the geometry 02 designs waypoint approach against).
- At a configured checkpoint hover pose, a tag of the chosen family/size appears within the camera frame.
- The mount pose and FOV are derived from, and consistent with, the checkpoint positions in the shared YAML.

**Trace:** UAC-SIM-4 (Appendix B)

#### SIM-5: M4 patrol traverses every checkpoint in the world
The system SHALL provide checkpoint positions (via the shared YAML) such that the existing M4 patrol mission, pointed at those positions, visits each checkpoint AprilTag in turn.

**Customer scenario:** The operator runs the patrol and watches the drone fly to each tagged checkpoint and hold there.

**Pain removed:** A world whose checkpoints the mission can't actually reach proves nothing about the integrated patrol-plus-perception path.

**Acceptance criteria:**
- The checkpoint positions in the shared YAML are reachable as waypoints by the M4 patrol mission running in SITL.
- When the M4 patrol is configured to the same checkpoint positions and run in SITL, the drone visits each checkpoint AprilTag in turn.
- The position values are expressed in (or unambiguously convertible to) the frame the M4 mission consumes, with the world/ENU frame stated explicitly.

**Trace:** UAC-SIM-5 (Appendix B)

#### SIM-6: World loads from a fresh containerized checkout with no host paths
The system SHALL ensure the world, all referenced models, and all textures load from a fresh checkout inside the 01-platform containerized sim environment, with no host-specific assets and no absolute paths.

**Customer scenario:** A collaborator clones the repo, runs the sim container, launches the world, and it renders — without "works on my machine" asset-path failures.

**Pain removed:** Host-specific or absolute asset paths break the world on any machine but the author's, defeating the containerization discipline and blocking CI/integration use of the world.

**Acceptance criteria:**
- All models and textures referenced by the world live under `sim/` and are checked into the repo.
- Launched from a fresh checkout inside the sim container, the world loads with no missing-asset or absolute-path errors.
- No reference resolves to a path outside the repo or to a host-specific location.

**Trace:** UAC-SIM-6 (Appendix B)

### P2: Important (should ship)

#### SIM-7: AprilTag family/ID assignment reusable on hardware
The system SHALL choose the AprilTag family and ID assignment so the same fiducials and detection settings carry unchanged into Phase 4 indoor relocalization.

**Customer scenario:** In Phase 4, the developer prints the same tag family/IDs for real checkpoints and the Phase-1 detection settings work unmodified — no re-authoring of assets or detector config.

**Acceptance criteria:**
- A single AprilTag family (e.g. tag36h11) and an ID range are chosen and documented, with the rationale that they transfer to Phase 4 hardware.
- The `tag_family` and `tag_id` fields in the shared checkpoint YAML use that family/range.
- The choice is recorded such that 04-perception's detector settings can target it directly.

## Scope Authority

> The FR table above is the **contract** for this PRD. The design document ([`design.md`](./design.md) — to be added when a design is created) realizes these FRs as world/model assets, the camera-sensor attachment, the checkpoint-config schema, and milestone tasks.
>
> **The design must not introduce surface area beyond this PRD's FR table without a corresponding PRD revision.** If the design proposes a new ROS 2 topic, a new config field, a new asset contract, or a new consumer interface not authorized by an FR, this PRD must be updated first.
>
> Conversely, **this PRD must not specify implementation detail beyond the FR shape.** The exact SDF structure, whether the world is static SDF vs. generated-from-YAML, the concrete texture-generation tooling, and the specific mount transform belong in the design, not here.

## Success Metrics

| Metric | Baseline (current) | Target | How Measured | Timeline |
|--------|-------------------|--------|--------------|----------|
| Custom world loads under SITL | N/A (new — only empty plane exists) | Loads with no errors | Launch SITL against the world; observe Gazebo + logs | M5 exit |
| Checkpoint AprilTags placed from YAML | 0 | ≥3 at configured positions | Count rendered markers vs. YAML entries | M5 exit |
| Camera image topic publishing | N/A (no camera) | Steady non-zero `hz` on both `Image` and `CompressedImage` topics | `ros2 topic hz <camera topic>` | M5 exit |
| M4 patrol visits all checkpoints | N/A (no checkpoints) | Every checkpoint visited in turn | Run M4 patrol in SITL against checkpoint positions | M5 exit |
| Fresh-container load with no host paths | N/A | Loads clean from fresh checkout in sim container | Clone + container launch; observe no asset/path errors | M5 exit |
| Sim runs at/near real-time on minimum spec *(non-binding, per H1)* | N/A (no world to run) | RTF ≈ 1.0; no sustained dropped physics frames during a full patrol | Read Gazebo real-time-factor during the M4 patrol on a minimum-spec host | M5 exit · **[INFERRED — derived sim signal, not a DoD contract; non-binding]** |

## Technical Considerations

### Integration points
- **01-platform:** consumes Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy base, the `ros2_ws` build, and the sim container that loads worlds; attaches the camera to the PX4 SITL airframe (`gz_x500` is the M1 default — a camera variant/override may be required).
- **02-mission-control:** reads checkpoint positions from the shared YAML to build patrol waypoints; the camera mount geometry (SIM-4) constrains 02's waypoint approach.
- **04-perception:** subscribes to the RGB camera topic and detects the AprilTag models; maps a detected `tag_id` → `checkpoint_id` using the shared YAML.
- **05-logging-replay:** records the live `sensor_msgs/CompressedImage` camera topic into the MCAP bag (per M7's compressed-image note); renders it in Foxglove (exit item 8).

### Data storage
- No runtime data store. Persistent artifacts are checked-in repo files: `sim/worlds/<patrol_world>.sdf`, `sim/models/<apriltag_*>/`, `sim/px4_sitl_overrides/`, and the shared checkpoint-config YAML. Assets prefer simple primitives or external references over large binary meshes (sim/README conventions).

### Rabbit holes
> Things that look simple but could explode in scope. Flag them early.

- **Static SDF vs. generate-world-from-YAML.** Generation keeps a single source of truth for checkpoint positions but adds tooling and a build step; static SDF is simplest but risks the YAML and the SDF drifting. Contain by deciding the approach in the design and, if static, adding a check that SDF marker positions match the YAML. (Tracked as OQ-3.)
- **Camera-sees-tag geometry.** Mount pose × FOV × tag size × checkpoint approach altitude is a coupled tuning problem that can sprawl. Contain by fixing one reference checkpoint approach and tuning against it, then locking the contract (SIM-4) before 02 designs approach geometry.
- **Airframe camera attachment.** Adding a camera to `gz_x500` may require an airframe variant/override rather than a stock target; mismatches surface as a drone that flies but has no camera topic. Contain by validating SIM-3 against SITL early, before building out the world.

### Potential challenges
- **Sim performance on minimum spec.** Too much geometry or heavy meshes slow physics below real-time. Mitigation: primitives-first authoring (Tenet 1, H1); validate against the minimum-spec assumption via the non-binding RTF metric.
- **Frame conventions.** World/ENU vs. the M4 mission's NED-relative-to-EKF-origin frame is a silent-failure surface (the plan calls frame mistakes "silent and infuriating"). Mitigation: state the world frame explicitly in the YAML schema and the SIM-5 acceptance criteria; reconcile the conversion boundary with 02.

### Security & operational implications
- None. Sim-only assets and config; no auth, no user data, no network surface, no persistent or destructive operations. Failures are visible at launch and fully reversible by editing/regenerating assets. (Security analysis conclusion: not applicable — no security-relevant surface.)

## Cross-Service Impact

> Scope is cross-service: this docset **owns** three contracts that 02, 04, and 05 bind to. These are the contracts the design realizes and that the joint /drive reconciliation must confirm.

### Affected Services

| Service / docset | Impact | Changes required (in that docset, downstream of this contract) |
|---|---|---|
| 02-mission-control | Consumes checkpoint positions | Reads the shared YAML to build patrol waypoints; designs waypoint approach against the camera mount geometry (SIM-4) |
| 04-perception | Consumes camera topic + tag models + schema | Subscribes to the RGB camera topic; maps detected `tag_id` → `checkpoint_id` via the shared YAML; targets the chosen tag family (SIM-7) |
| 05-logging-replay | Consumes camera topic | Records the `sensor_msgs/CompressedImage` camera topic into the MCAP bag; renders it in Foxglove (exit item 8) |

### Interface Changes (contracts this docset owns)

1. **Shared checkpoint-config schema** *(pending user confirmation — settled default for the 5-pair coherence)*: a single shared YAML at `sim/config/checkpoints.yaml`, a list of `{checkpoint_id: string, position: {x, y, z} in the world/ENU frame, tag_family: string, tag_id: int}`. 02 reads `position` to build waypoints; 03 places the AprilTag models + camera; 04 maps a detected `tag_id` → `checkpoint_id`. (Reconciles 03 §7, 02 §7 "YAML schema shape", 04 §7 "checkpoint_id namespace / mapping" into one schema.)
2. **RGB camera topics** *(name/resolution/rate/frame_id — to be fixed in design, OQ-4)*: a `sensor_msgs/Image` topic from the drone-mounted camera (live frames 04 subscribes to), plus a companion `sensor_msgs/CompressedImage` topic that 05 records (recorded frames). This is the contract 04 and 05 bind to.
3. **AprilTag asset + family contract** *(family/ID range — to be fixed in design, OQ-1)*: the textured Gazebo models under `sim/models/<apriltag_*>/` and the single tag family/ID range, chosen to run unmodified on Phase 4 hardware (SIM-7), that 04's detector targets.

### Deployment Coordination
- **Build/dependency order:** `01 → 02 → 03 → 04 → 05`. This docset stands on 01 (sim base, container, build) and 02 (the M4 patrol used to traverse checkpoints); 04 and 05 stand on the contracts above.
- The three contracts above must be **frozen jointly** during /drive so the 5 PRD/Design pairs stay coherent; they are flagged "pending user confirmation" below and in the Open Questions table.

### Testing Implications
- Integration: launch SITL against the world (SIM-1), assert ≥3 markers from a YAML (SIM-2), assert the camera topics publish (SIM-3), and run the M4 patrol to traverse checkpoints (SIM-5) — these exercise the contracts end-to-end and feed the integrative exit item 1.
- Contract: the camera topic name(s)/type and the checkpoint-YAML schema are the surfaces 04/05 and 02 assert against; a fresh-container load (SIM-6) is the portability check.

## Alternatives Considered

> Two decisions in this docset have meaningful alternatives worth recording; the rest are settled by the DoD's constraints.

### World authoring: static SDF vs. generated-from-YAML (selected: deferred to design — OQ-3)
**Static SDF** is the simplest path: author the world once, hand-place tag models. **Pro:** no tooling, no build step. **Con:** the SDF tag positions and the shared YAML can drift, violating Tenet 2 (one source of truth).
**Generate-from-YAML** keeps a single source of truth. **Pro:** YAML is authoritative; positions can't drift. **Con:** adds a generation step and tooling to maintain.
**Why deferred to design:** both satisfy the FRs; the trade-off is single-source-of-truth (Tenet 2) vs. simplicity (Tenet 1), and the design is the right place to weigh it — if static is chosen, a position-consistency check mitigates drift.

### Simulator: Gazebo Harmonic vs. Isaac Sim (selected: Gazebo Harmonic)
**Gazebo Harmonic** is the settled choice. **Pro:** native to Ubuntu 24.04, PX4's modern integration target, sufficient for crude checkpoint worlds, light on the Phase 1 spec. **Trade-off accepted:** no photorealism — acceptable because M5 explicitly defers fidelity to Phase 5.
**Isaac Sim** offers high fidelity. **Why not chosen:** it is heavier, out of scope for Phase 1, and its value (trail data, RL) lands in Phase 5+/8. This is a settled constraint, not an open trade-off.

### Do nothing / status quo
Without this docset, SITL has only an empty plane: no checkpoints, no fiducials, no camera. **Why not acceptable:** M6 (perception capture) and M7 (logging of imagery) have no stage and cannot be built or tested; the integrative end-to-end patrol (exit item 1) is unreachable; and Phase 1 cannot exit, blocking the Phase 2 hardware-purchase decision.

## Milestones

### Phase 1: Camera + world skeleton (validate the riskiest contract first)
- Attach the RGB camera to the SITL airframe; verify the `sensor_msgs/Image` + `CompressedImage` topics publish at a steady rate (SIM-3).
- Author the crude world (terrain + boxes + trees) loading cleanly under SITL (SIM-1).
- **Validation:** `ros2 topic hz` shows a steady camera rate on both topics; the world renders with no load errors.

### Phase 2: Checkpoints + config + capture geometry
- Define the shared checkpoint-config YAML schema; place ≥3 AprilTag models from it (SIM-2).
- Choose and document the AprilTag family/ID range for hardware reuse (SIM-7).
- Tune camera mount pose/FOV so a tag is in-frame at a checkpoint hover (SIM-4).
- **Validation:** ≥3 markers appear at YAML positions; a checkpoint hover frame contains a resolvable tag.

### Phase 3: Integration + portability
- Run the M4 patrol against the checkpoint positions; confirm it visits each in turn (SIM-5).
- Confirm a fresh containerized checkout loads with no host paths (SIM-6).
- **Validation:** the M4 patrol traverses every checkpoint in SITL; the world loads clean from a fresh in-container checkout.

## Open Questions

| # | Question | Status | Decision target | Rationale (why open / what would resolve it) |
|---|----------|--------|-----------------|----------------------------------------------|
| OQ-1 | AprilTag family and ID range (e.g. tag36h11 vs tag25h9) | Open | Design | Affects detection robustness and must be the family used unmodified in Phase 4; the plan leaves the family open. Resolved by a documented choice in the design (default leaning tag36h11 per H3). |
| OQ-2 | Checkpoint-config file location, name, and exact schema | **Provisional (pending user confirmation)** | Design — joint with 02 §7 + 04 §7 | The world, the patrol waypoints (02), and perception (04) must agree on one schema. **Settled default to confirm:** `sim/config/checkpoints.yaml`, list of `{checkpoint_id: string, position {x,y,z} in world/ENU frame, tag_family: string, tag_id: int}`. Flagged for the human's combined 5-pair review. |
| OQ-3 | World authored as static SDF vs. generated from the YAML at launch | Open | Design | Static SDF is simplest; generation keeps one source of truth for checkpoint positions but adds tooling. Resolved by the design choosing one (with a drift check if static). |
| OQ-4 | RGB camera topic name, resolution, frame rate, and `frame_id` | **Provisional (pending user confirmation)** | Design — joint with 04 + 05 | Becomes the contract 04 and 05 bind to; resolution/rate trade sim load against image usefulness and bag size. **Settled default to confirm:** a `sensor_msgs/Image` topic plus a companion `sensor_msgs/CompressedImage` topic that 05 records (per M7). Concrete name/resolution/rate/frame_id to be fixed in design. |
| OQ-5 | Camera mount pose and FOV on the airframe | Open | Design | Must let the drone see a tag when hovering at a checkpoint (SIM-4); affects waypoint approach geometry in 02. Resolved by tuning against a reference checkpoint and locking the contract. |
| OQ-6 | World extent and obstacle layout specifics | Open | Design | Enough geometry to be "meaningful" without slowing physics; the plan says "flat plane, building-like boxes, a few trees" but leaves specifics open. Resolved by the design fixing extent + layout, validated by H1. |
| OQ-7 | Which SITL airframe target the camera attaches to | Open | Design | `gz_x500` is the M1 default; attaching a camera may require an override/variant. Resolved by validating SIM-3 against the chosen airframe target early. |
| OQ-8 | Whether the near-real-time sim performance target (H1 / SIM-1 / Success Metrics row) should be promoted to a binding acceptance criterion | Open | Design / human review | The DoD sets no perf contract, so this PRD keeps it a non-binding derived signal. If the human review (or design-phase profiling) decides minimum-spec performance is a hard gate, an RTF threshold becomes a binding AC. Resolved by the combined human review or by a measured RTF threshold in design. **[INFERRED — residual review finding, deferred rather than invented.]** |

## Appendix B: User Acceptance Criteria

> Every P1 FR has a corresponding UAC in Given/When/Then form. UAC IDs match the FR ID.

### UAC-SIM-1: Custom patrol world loads in Gazebo Harmonic under SITL
**GIVEN** the project repo and a running PX4 SITL session
**WHEN** the custom world is loaded
**THEN** Gazebo Harmonic renders the patrol environment (flat terrain + building-like boxes + trees) without load errors, and the world file lives under `sim/worlds/`. *(DoD AC-1. The near-real-time performance expectation is a non-binding target per H1 — not asserted as a pass/fail condition here.)*

### UAC-SIM-2: AprilTag checkpoint markers at YAML-configured positions
**GIVEN** a checkpoint-positions YAML with ≥3 entries (`checkpoint_id`, `position`, `tag_family`, `tag_id`)
**WHEN** the world is generated/loaded
**THEN** ≥3 AprilTag checkpoint markers (textured Gazebo models under `sim/models/`) appear at the YAML-configured world positions, and editing a position + relaunching moves the marker with no SDF edit. *(DoD AC-2)*

### UAC-SIM-3: Simulated RGB camera publishes ROS 2 image topics
**GIVEN** SITL running with the drone in the world
**WHEN** ROS 2 inspects topics
**THEN** a simulated RGB camera publishes a `sensor_msgs/Image` topic at a steady, non-zero rate AND a companion `sensor_msgs/CompressedImage` topic for the bag, using the documented topic name(s)/resolution/frame rate/`frame_id`. *(DoD AC-4; the CompressedImage companion is the settled cross-docset contract default per M7 — pending user confirmation.)*

### UAC-SIM-4: Camera mount/FOV lets the drone see a tag at a checkpoint
**GIVEN** the camera mounted at its fixed, documented pose and FOV, and a configured checkpoint hover pose
**WHEN** the drone hovers at the checkpoint per the approach geometry
**THEN** a checkpoint AprilTag of the chosen family/size is within the camera frame and large enough to be resolvable.

### UAC-SIM-5: M4 patrol traverses every checkpoint in the world
**GIVEN** the world and the M4 patrol mission configured to the same checkpoint positions from the shared YAML
**WHEN** the patrol runs in SITL
**THEN** the drone visits each checkpoint AprilTag in turn, with positions expressed in (or convertible to) the frame the M4 mission consumes. *(DoD AC-3)*

### UAC-SIM-6: World loads from a fresh containerized checkout with no host paths
**GIVEN** a fresh checkout inside the 01-platform containerized sim environment
**WHEN** the world is launched
**THEN** it loads with no host-specific assets and no absolute paths — all referenced models/textures live under `sim/` and are repo-checked-in. *(DoD AC-5)*
