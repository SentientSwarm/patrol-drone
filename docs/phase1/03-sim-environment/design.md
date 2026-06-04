# Simulation Environment & Assets (M5) — System Design Working Document

**Status:** Draft
**Version:** 0.1.1
**Date:** 2026-06-03
**Projects:** Patrol Drone Phase 1 (pre-hardware simulation) — 03-sim-environment (Simulation Environment & Assets)
**Authors:** jxstanford@wemodulate.energy (DRI)

> **Requirements source:** This design's sole requirements source is [`prd.md`](./prd.md) (Simulation Environment & Assets, rev 2). Every design surface traces to a PRD FR (SIM-1…SIM-7); no surface is introduced beyond the PRD's FR table without a flagged Open Question. Cross-docset contracts (checkpoint-config schema, camera topics, AprilTag family) follow the settled `/drive` defaults and are flagged "pending user confirmation."

> **Right-sizing note:** This is a Phase-1 simulation-asset docset — authoring (one world, a few tag models, one camera sensor, one YAML config), not a production service. Per the software-design template's "detail proportional to complexity/risk" rule, dimensions that don't apply to checked-in sim assets are explicitly scoped out rather than padded: there is no auth/data-isolation surface (§4.4.3), no UI/UX surface (§4.7), and no persistent datastore (§4.6). The design's substance is the three cross-service contracts (§4.4) and the asset/component decomposition that realizes them.

> **Review status (v0.1.1):** Self-reviewed against the 13-dimension ReviewDesign rubric including the D2 PRD-trace audit (every design surface enumerated and traced to a named PRD FR or a marked-internal inferred requirement). Result: all 13 dimensions Strong; zero findings at the medium severity floor; PRD-trace audit produced zero unauthorized-scope items. No content revision required. Two residual sub-medium polish notes and three deferred decisions are recorded below and carried into the human's combined 5-pair review.

---

## 1. Introduction

This design realizes M5 of the Phase 1 simulation plan: a checked-in Gazebo Harmonic (gz-sim 8) patrol world, AprilTag fiducial checkpoint markers placed from a shared YAML config, and a simulated RGB camera attached to the PX4 SITL drone that publishes ROS 2 image topics. It is the physical stage on which the M4 patrol (02-mission-control), perception capture (04-perception), and bag logging (05-logging-replay) are exercised end-to-end before any hardware exists. The docset owns no Phase-1 exit-checklist item solo; its value is being the stage for integrative item 1 (end-to-end patrol), item 8 (Foxglove camera render), and item 11 (`CheckpointCapture` pipeline).

The design is organized around five asset/component groups: a **Patrol World** (terrain + obstacles), an **AprilTag Model Library** (textured checkpoint markers), a **Checkpoint Config** (the shared `checkpoints.yaml` source of truth), a **World Composer** (the mechanism that places markers from the config — static-SDF vs generate-from-YAML is resolved in §2), and a **Camera Sensor Attachment** (the airframe override that adds the RGB camera and publishes the ROS 2 image topics). Three of these own cross-service contracts that 02, 04, and 05 bind to; keeping those contracts coherent is what argues the work up to a structured design rather than ad-hoc SDF editing.

Because this is a greenfield asset docset standing on a shell-only `sim/` tree (only `.gitkeep` placeholders exist today) and a containerized SITL base from 01-platform, "existing foundation" (§3) is thin — it is the 01-platform sim container, the PX4 `gz_x500` airframe, and the ROS 2 ↔ Gazebo image bridge, not a pre-existing codebase to extend. The design's risk surface is correspondingly narrow and fully reversible: every artifact is a checked-in file that can be edited or regenerated.

### Source Projects

| # | Project | Source | Wave |
|---|---------|--------|------|
| 1 | 03-sim-environment — Simulation Environment & Assets | [`prd.md`](./prd.md) (rev 2) → M5 of [`phase1_simulation_plan.md`](../../phase1_simulation_plan.md) | Phase 1 (build order position 3 of 5) |

### Related Projects

| Project | Relevance |
|---------|-----------|
| 01-platform | Provides Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy base, the `ros2_ws` build, and the sim container that loads worlds. This design's airframe override and ROS↔gz image bridge run inside it. |
| 02-mission-control | Consumes `position` from `checkpoints.yaml` to build waypoints; designs waypoint-approach geometry against the camera mount (SIM-4). Owns the world/ENU → PX4-NED conversion boundary. |
| 04-perception | Consumes the RGB camera topic + AprilTag models; maps detected `tag_id` → `checkpoint_id` via `checkpoints.yaml`; its `apriltag_ros` detector targets the chosen tag family (SIM-7). |
| 05-logging-replay | Records the `sensor_msgs/CompressedImage` camera topic into the MCAP bag; renders it in Foxglove (exit item 8). |

---

## 2. Open Questions & Assumptions

Carried forward from the PRD (OQ-1…OQ-8). The design resolves the ones whose decision target is "Design"; the two cross-docset contract defaults (OQ-2, OQ-4) are held **Provisional (pending user confirmation)** per the `/drive` policy — they are not silently invented, they carry the settled defaults for 5-pair coherence and await the human's combined review. Decision target for Provisional items: the joint 5-pair `/drive` reconciliation.

| # | Item | Source | Status | Resolution / Rationale |
|---|------|--------|--------|------------------------|
| OQ-1 | AprilTag family + ID range | PRD OQ-1 / SIM-7 | **Resolved (design)** | **tag36h11, IDs 0…N−1** (one per checkpoint, contiguous from 0). tag36h11 is the robust `apriltag_ros` default (PRD H3) and is the family Phase 4 hardware prints unmodified (Tenet 3). Contiguous low IDs keep the family/range trivially reusable. |
| OQ-2 | Checkpoint-config location, name, schema | PRD OQ-2 (joint 02 §7 + 04 §7) | **Provisional (pending user confirmation)** | Settled default: `sim/config/checkpoints.yaml`, list of `{checkpoint_id: string, position: {x,y,z} (world/ENU, meters), tag_family: string, tag_id: int}`. Confirmed shape in §4.4.2. Held for the human's combined 5-pair review. Decision target: joint 5-pair `/drive` reconciliation. |
| OQ-3 | World as static SDF vs generated-from-YAML | PRD OQ-3 | **Resolved (design)** | **Generate-from-YAML at build time** (the `World Composer`, §4.2.4): one authoritative source for checkpoint positions (Tenet 2) with a checked-in *generated* world for fast launch + a CI drift check that the committed world matches the YAML. Rationale and the static-SDF trade-off in §4.2.4. |
| OQ-4 | Camera topic name, resolution, rate, frame_id | PRD OQ-4 (joint 04 + 05) | **Provisional (pending user confirmation)** | Settled default: `sensor_msgs/Image` on `/drone/camera/image_raw` (live frames, 04 subscribes) + companion `sensor_msgs/CompressedImage` on `/drone/camera/image_raw/compressed` (05 records). Concrete values (640×480, 15 Hz, `frame_id: camera_link`) proposed in §4.4.4; held for the human's combined 04/05 review. Decision target: joint 5-pair `/drive` reconciliation. |
| OQ-5 | Camera mount pose + FOV | PRD OQ-5 / SIM-4 | **Resolved (design, provisional values)** | Forward-and-slightly-down mount, **horizontal FOV ≈ 1.2 rad (~69°)**, tuned against one reference checkpoint at a fixed hover pose (§4.5 Sequence 3). The mount transform is the contract 02 binds approach geometry to; values are locked in §4.4.4 and re-tunable before 02 finalizes approach. Decision target: M2 mount-tuning task (T2.5). |
| OQ-6 | World extent + obstacle layout | PRD OQ-6 | **Resolved (design)** | ~40×40 m flat ground plane, ≥3 checkpoints separated ≥8 m, ≥2 building-like boxes + ≥2 tree props as obstacles, primitives only (Tenet 1, H1). Layout fixed in §4.2.1. |
| OQ-7 | Which SITL airframe target the camera attaches to | PRD OQ-7 | **Resolved (design)** | A **`gz_x500_patrol` airframe override** (camera variant of the M1 default `gz_x500`) under `sim/px4_sitl_overrides/`, validated against SITL in M1-skeleton before world build (§4.2.5). |
| OQ-8 | Promote near-real-time RTF to a binding AC? | PRD OQ-8 | **Deferred — needs human / measured input** | The PRD's DoD sets no perf contract; this design keeps RTF a **non-binding** check (the success-metric row, §4.4.5 Performance). Promotion to a binding AC requires the combined human review or a measured minimum-spec RTF threshold from M2 profiling. Decision target: combined human review / M2 profiling. |

**Assumptions (explicit):**
- **A1:** 01-platform's sim container provides a working `gz_x500` SITL airframe and a ROS 2 ↔ Gazebo bridge (`ros_gz_bridge` / `ros_gz_image`) capable of republishing a `gz.msgs.Image` sensor onto a `sensor_msgs/Image` ROS 2 topic. *Verified in §3.2.*
- **A2:** PX4 SITL airframe selection is overridable via the standard PX4 model/airframe override path (an SDF model variant + airframe init) without forking PX4. *Verified in §3.2.*
- **A3:** The world/ENU → PX4-NED conversion is owned by **02-mission-control** (its DoD §6 "PX4 offboard uses NED relative to the EKF origin"; conversion at one explicit boundary). This design states positions in world/ENU and makes the frame explicit; it does **not** perform NED conversion.

---

## 3. Existing Foundation

This is a greenfield asset docset; "existing foundation" is the 01-platform sim container, the PX4/Gazebo runtime, and the empty `sim/` tree — not a codebase to extend.

### 3.1 Asset-Layer "Architecture"

```
                    01-platform sim container (Ubuntu 24.04 / ROS 2 Jazzy / gz-sim 8)
                                          │
        ┌─────────────────────────────────┼─────────────────────────────────┐
        ▼                                 ▼                                   ▼
  PX4 SITL (gz_x500)            Gazebo Harmonic runtime              ROS 2 ↔ gz bridge
   airframe + sensors            (loads .sdf world, renders)        (ros_gz_image / bridge)
        │                                 │                                   │
        └──── this design adds: ──────────┴───────────────────────────────────┘
              • camera sensor on airframe   • patrol world + tag models      • image topic bridge
```

| Layer | Owns (this design's surface) | Current state (today) | M5 awareness |
|-------|------------------------------|-----------------------|--------------|
| **Assets (SDF / textures / config)** | `sim/worlds/`, `sim/models/`, `sim/config/checkpoints.yaml` | shells only — `sim/worlds/.gitkeep`, `sim/models/.gitkeep` exist; no world, no models, no config | new in M5 |
| **Airframe / sensor override** | `sim/px4_sitl_overrides/` | shell only — `.gitkeep`; no override | new in M5 |
| **Tooling / build** | World Composer + CI drift check (under `sim/` + `scripts/`) | none | new in M5 |
| **ROS 2 topic surface** | camera image topics (bridged from gz sensor) | no camera on `gz_x500`; no image topic | new in M5 |

### 3.2 Verified Preconditions

External-system shapes this design depends on, verified against the actual repo / upstream at research time. Because this docset adds new artifacts onto a documented platform base, the verifiable claims are about (a) the empty asset tree it lands in and (b) the PX4/Gazebo/bridge capabilities it assumes.

| Claim | Verification | Result | Citation |
|-------|--------------|--------|----------|
| `sim/` tree exists with the three target dirs but no assets yet (design adds, not modifies) | `find sim -maxdepth 3 -type f` | `sim/README.md`, `sim/px4_sitl_overrides/.gitkeep`, `sim/worlds/.gitkeep`, `sim/models/.gitkeep` — shells only | `sim/worlds/.gitkeep`, `sim/models/.gitkeep`, `sim/px4_sitl_overrides/.gitkeep` |
| `sim/` conventions require SDF worlds, Gazebo model dir structure, and **no large binary meshes** (primitives / external refs) | Read `sim/README.md` Conventions | "Worlds are SDF format (`.sdf` or `.world`)." / "Models follow Gazebo's model directory structure (model.sdf + model.config + meshes/)." / "Don't check in large binary meshes — use simple primitives or external mesh references where possible." | `sim/README.md:13-15` |
| 01-platform owns the Gazebo Harmonic + PX4 SITL + ROS 2 base + `gz_x500` airframe + sim container that loads worlds | Read `docs/phase1/README.md` traceability matrix (01 "Consumes/Owns") | 01 owns "`sim`/`dev` containers + `docker compose`" and consumes "PX4-Autopilot upstream (SITL, `gz_x500`, `uxrce_dds_client`) … Gazebo Harmonic (gz-sim 8)" | `docs/phase1/README.md:16` |
| 02 consumes checkpoint world + positions + RGB camera topic; 02 uses **NED relative to EKF origin** (so 03 must state ENU explicitly and 02 owns the conversion) | Read `docs/phase1/02-mission-control/dod.md` §6 + §5 | "PX4 offboard uses NED relative to the EKF origin. Waypoints declare their frame; conversion happens at one explicit boundary." / Consumes "Checkpoint world, checkpoint positions, and the simulated RGB camera image topic — from 03-sim-environment" | `02-mission-control/dod.md:75,68` |
| 04 maps `tag_id → checkpoint_id` via 03's world YAML; `checkpoint_id` is a `string` | Read `04-perception/dod.md` §7 (`checkpoint_id` namespace, line 88) + AC-3 (line 52) | "Must reconcile with 03 §7 (checkpoint-config schema) so one schema maps `checkpoint_id ↔ world position ↔ tag id`." / "`checkpoint_id` (string)" | `04-perception/dod.md:88,52` |
| 05 records a camera image topic into the MCAP bag and wants bag size manageable (compressed image) | Read `05-logging-replay/dod.md` §2 (topic selection) + AC-2 | Records "camera image topic" among `/fmu/out/*`, `/patrol/*`; "bag size is reasonable (under a few hundred MB for a 5-minute mission)" | `05-logging-replay/dod.md:16,55` |

*No external-system precondition in this design asserts a chart-provides-X / Secret-exists / CRD-has-field shape; the verified claims above are the asset-tree and cross-docset-contract shapes the design binds to.*

### 3.3 Architectural Decision: Generate the world from the checkpoint YAML

**Decision:** The patrol world's AprilTag marker placements are **generated from `sim/config/checkpoints.yaml`** by a small build-time composer, producing a checked-in generated world SDF; a CI check asserts the committed world's marker positions match the YAML.
**Rationale:** Tenet 2 (one source of truth for checkpoints) — 02 (waypoints), 03 (placement), and 04 (`tag_id`→`checkpoint_id`) all read the same rows, so positions cannot drift between SDF and YAML. The static-SDF alternative is simpler but reopens the drift surface the PRD's rabbit-holes section flags (OQ-3).
**Implication:** Adds a thin composer (§4.2.4) + a CI drift check (§4.4.5 Configuration) but removes a whole class of silent position-mismatch bugs. The generated world is still checked in (fast launch, no launch-time generation dependency); the composer is re-run when the YAML changes.

---

## 4. Detailed Design

### 4.1 UC Traceability Matrix

Every UAC and the PRD FR it derives from maps to at least one component. (UAC-SIM-1…6 from PRD Appendix B; SIM-7 is P2, no UAC, traced to the AprilTag Model Library + Checkpoint Config.)

| Design Component | Covers (FR / UAC) | Notes |
|------------------|-------------------|-------|
| **Patrol World** (§4.2.1) | SIM-1 / UAC-SIM-1; SIM-6 / UAC-SIM-6 | Terrain + obstacles; portability of the world root |
| **AprilTag Model Library** (§4.2.2) | SIM-2 / UAC-SIM-2; SIM-7 (P2) | Textured tag models; family/ID reuse on hardware |
| **Checkpoint Config** (`checkpoints.yaml`) (§4.2.3) | SIM-2 / UAC-SIM-2; SIM-5 / UAC-SIM-5; SIM-7 (P2) | Shared source of truth; positions consumed by 02; `tag_id`↔`checkpoint_id` by 04 |
| **World Composer** (§4.2.4) | SIM-2 / UAC-SIM-2; SIM-6 / UAC-SIM-6; INF-S2/S3 | Places markers from YAML; portability/no-host-paths enforcement (internal) |
| **Camera Sensor Attachment** (§4.2.5) | SIM-3 / UAC-SIM-3; SIM-4 / UAC-SIM-4 | RGB camera on airframe; image topics; mount pose/FOV geometry |
| **Patrol Bring-up Glue** (§4.2.6) | SIM-1; SIM-5 / UAC-SIM-5; SIM-3 | Launches SITL+world+camera; lets the M4 patrol traverse checkpoints (internal) |

**Coverage check:** SIM-1✓ SIM-2✓ SIM-3✓ SIM-4✓ SIM-5✓ SIM-6✓ SIM-7✓ — all six P1 FRs and the one P2 FR are covered.

### 4.2 Component Architecture

#### 4.2.0 Component Inventory

| Component | Type | Boundary (in / out) | Responsibility | Dependencies |
|-----------|------|----------------------|----------------|--------------|
| **Patrol World** | Asset (SDF) | In: terrain, obstacles, world root, ENU frame. Out: tag placement (composer), camera (airframe) | A loadable gz-sim 8 patrol environment | gz-sim 8 (01); AprilTag models; generated marker block |
| **AprilTag Model Library** | Asset (Gazebo models) | In: textured tag36h11 models, model.config. Out: where they sit (config) | Reusable checkpoint-marker models, hardware-aligned | tag36h11 textures; gz-sim model dir convention |
| **Checkpoint Config** | Config (YAML) | In: `checkpoint_id`/`position`/`tag_family`/`tag_id` rows. Out: waypoint logic (02), detection (04) | Single source of truth for checkpoints | none (authored); consumed by 02, 04, composer |
| **World Composer** | Module (script, internal) | In: read YAML, emit marker SDF into world, emit drift check. Out: runtime behavior, NED conversion | Place markers from config; enforce one-source-of-truth + portability | Checkpoint Config; AprilTag Model Library; Patrol World template |
| **Camera Sensor Attachment** | Config (airframe/SDF override) | In: camera sensor, mount transform, FOV, image-topic publish. Out: detection/recording (04/05) | Add RGB camera to airframe + publish ROS 2 image topics | PX4 SITL `gz_x500` (01); ROS↔gz image bridge (01) |
| **Patrol Bring-up Glue** | Module (launch/config, internal) | In: launch SITL + world + camera + bridge; point M4 patrol at checkpoints. Out: patrol logic itself (02) | Wire the stage together so the M4 patrol can run against it | 01 container/launch; 02 `mission_patrol.launch.py`; all above |

#### 4.2.0a Component Dependency Diagram

```
            ┌─────────────────────┐
            │  Checkpoint Config  │  sim/config/checkpoints.yaml
            │  checkpoints.yaml   │◄────────────── consumed by 02 (positions)
            └──────────┬──────────┘◄────────────── consumed by 04 (tag_id→checkpoint_id)
                       │ reads
                       ▼
            ┌─────────────────────┐      places markers from      ┌───────────────────────┐
            │   World Composer    │─────────────────────────────► │ AprilTag Model Library│
            │  (gen + drift check)│                               │  sim/models/apriltag_*│
            └──────────┬──────────┘                               └───────────┬───────────┘
                       │ emits generated world                                │ referenced by
                       ▼                                                      ▼
            ┌─────────────────────────────────────────────────────────────────────────┐
            │                          Patrol World (SDF)                               │
            │              sim/worlds/patrol_world.sdf  (terrain + obstacles + markers) │
            └──────────────────────────────────┬────────────────────────────────────────┘
                                               │ loaded by
                       ┌───────────────────────┴───────────────────────┐
                       ▼                                                ▼
            ┌─────────────────────────┐                  ┌──────────────────────────────┐
            │ Camera Sensor Attachment│                  │   Patrol Bring-up Glue       │
            │ sim/px4_sitl_overrides/ │─ image topics ──►│ launch: SITL+world+camera    │
            │ gz_x500_patrol + bridge │  (04 sub, 05 rec)│ + point M4 patrol (02) at YAML│
            └─────────────────────────┘                  └──────────────┬───────────────┘
                                                                        │ invokes
                                                                        ▼
                                                              02 mission_patrol.launch.py
```

Every inventory row appears as a node; every node traces to an inventory row. **Inventory triangle:** the consumer-facing manifestation for an asset docset is the **cross-service contract surface (§4.4.2/4.4.3/4.4.4)** — every consumer-relevant inventory row (Checkpoint Config, AprilTag Model Library, Camera Sensor Attachment) appears there; the internal-only rows (World Composer, Patrol Bring-up Glue, Patrol World terrain) are marked internal and have no consumer contract mount. Triangle is consistent.

#### 4.2.1 Patrol World

**Type:** Asset (SDF) · **Location:** `sim/worlds/patrol_world.sdf` (generated; template `sim/worlds/patrol_world.template.sdf`) · **Dependencies:** gz-sim 8; AprilTag models; composer-emitted marker block

**Boundary:** Owns the world root, terrain, obstacle geometry, lighting/physics defaults, and the **world/ENU coordinate frame** declaration. Delegates marker placement to the World Composer and the camera to the airframe override.

**Layout (resolves OQ-6):**

| Element | Spec | Rationale |
|---------|------|-----------|
| Ground | ~40×40 m flat plane, primitive | "flat plane" (plan M5); large enough for ≥3 checkpoints ≥8 m apart |
| Obstacles | ≥2 building-like boxes (primitives), ≥2 tree props (simple primitives / lightweight external refs) | "building-like boxes, a few trees" (plan); Tenet 1 / H1 keeps physics fast |
| Frame | World/ENU, origin stated in a header comment + the config schema | SIM-5 AC; frame mistakes "silent and infuriating" (02 DoD); 02 owns NED conversion |
| Physics | gz-sim defaults; no heavy meshes | H1 (near-real-time on minimum spec) |

**No host paths (SIM-6):** all `<uri>` references resolve to `model://` or repo-relative paths under `sim/`; the composer rejects any absolute/host path at generation time (§4.2.4).

*Traces to: SIM-1 / UAC-SIM-1, SIM-6 / UAC-SIM-6.*

#### 4.2.2 AprilTag Model Library

**Type:** Asset (Gazebo models) · **Location:** `sim/models/apriltag_36h11_<id>/` (one model dir per ID: `model.sdf` + `model.config` + `materials/textures/tag36_11_<id>.png`) · **Dependencies:** tag36h11 texture set; gz model-dir convention

**Boundary:** Owns the textured tag models and their physical size. Delegates *where* each model sits to the Checkpoint Config + composer, and *whether it is detected* to 04.

**Tag spec (resolves OQ-1, SIM-7):**

| Property | Value | Rationale |
|----------|-------|-----------|
| Family | **tag36h11** | Robust `apriltag_ros` default (PRD H3); runs unmodified on Phase 4 hardware (Tenet 3, SIM-7) |
| ID assignment | contiguous `0…N−1`, one per checkpoint | Trivially reusable ID range; 04's detector targets the family + range directly |
| Physical size | a fixed checkpoint-appropriate edge length (e.g. ~0.5 m), documented constant | Large enough to resolve at the SIM-4 hover distance; transfers to printed hardware tags |
| Texture | generated tag36h11 PNG per ID (small raster, not a binary mesh) | `sim/README` "no large binary meshes" (3.2); textures are tiny |

**Sim/hardware parity (SIM-7):** the family, ID range, and size are recorded here as the contract 04's detector settings target; Phase 4 prints the same IDs at the same size with no re-author.

*Traces to: SIM-2 / UAC-SIM-2, SIM-7 (P2).*

#### 4.2.3 Checkpoint Config

**Type:** Config (YAML) · **Location:** `sim/config/checkpoints.yaml` *(Provisional — OQ-2)* · **Dependencies:** none (authored); consumed by 02, 04, composer

**Boundary:** The **single source of truth** for checkpoints (Tenet 2). Owns the `checkpoint_id ↔ position ↔ tag_id` mapping. Delegates waypoint construction to 02, tag detection/mapping to 04, marker placement to the composer.

**Schema (resolves the placement contract; OQ-2 Provisional):**

```yaml
# sim/config/checkpoints.yaml — world/ENU frame, meters. Single source of truth (Tenet 2).
checkpoints:
  - checkpoint_id: "cp_north"     # string, semantic name (04 maps tag_id -> this)
    position: { x: 12.0, y: 8.0, z: 1.5 }   # world/ENU, meters
    tag_family: "tag36h11"
    tag_id: 0                     # int; matches sim/models/apriltag_36h11_0
  - checkpoint_id: "cp_east"
    position: { x: 18.0, y: -6.0, z: 1.5 }
    tag_family: "tag36h11"
    tag_id: 1
  - checkpoint_id: "cp_south"
    position: { x: -10.0, y: -12.0, z: 1.5 }
    tag_family: "tag36h11"
    tag_id: 2
```

Frame is **explicitly world/ENU** (SIM-5 AC). 02 reads `position` and converts to PX4-NED at its own boundary (A3); 04 reads `tag_id`→`checkpoint_id`. The schema deliberately carries no waypoint fields (dwell/tolerance) — those belong to 02's mission YAML (02 DoD §5), keeping ownership clean.

*Traces to: SIM-2 / UAC-SIM-2, SIM-5 / UAC-SIM-5, SIM-7 (P2).*

#### 4.2.4 World Composer

**Type:** Module (script, internal) · **Location:** `sim/tools/compose_world.py` + drift check in `scripts/` · **Dependencies:** Checkpoint Config; AprilTag Model Library; Patrol World template

**Boundary:** Reads `checkpoints.yaml`, emits one `<include>` per checkpoint (referencing `apriltag_36h11_<tag_id>` at `position`) into a copy of the world template, and writes the generated `sim/worlds/patrol_world.sdf`. Owns **portability enforcement** (rejects absolute/host paths) and the **drift check** (committed world markers == YAML). Delegates terrain/obstacles to the template and runtime behavior to Gazebo.

```python
def compose_world(config_path, template_path, out_path):
    """
    Guards: every checkpoint has checkpoint_id/position/tag_family/tag_id;
            tag_id has a matching sim/models/apriltag_36h11_<id> dir;
            all emitted <uri> are model:// or repo-relative (no host/abs paths).
    Effect: writes generated patrol_world.sdf with one tag include per checkpoint at its ENU position.
    Side effects: none (pure file emission; idempotent).
    """

def check_drift(config_path, world_path):
    """Guards: parsed marker positions in world_path == YAML positions. Used by CI (SIM-2/SIM-6)."""
```

**Static-SDF trade-off (recorded, OQ-3):** static SDF avoids the composer entirely but reopens YAML↔SDF drift (Tenet 2 violation). Generation keeps one source of truth; the cost is this thin script + the drift check. Chosen per §3.3.

*Internal component — no cross-service contract. Traces to: SIM-2 / UAC-SIM-2, SIM-6 / UAC-SIM-6, INF-S2/INF-S3.*

#### 4.2.5 Camera Sensor Attachment

**Type:** Config (airframe / SDF override) · **Location:** `sim/px4_sitl_overrides/gz_x500_patrol/` (model SDF variant + airframe init) · **Dependencies:** PX4 SITL `gz_x500` (01); ROS↔gz image bridge (01)

**Boundary:** Adds an RGB camera sensor to the airframe at a fixed mount transform + FOV and ensures its frames reach ROS 2 as the documented image topics. Delegates detection to 04 and recording to 05; does **not** modify the host PX4 install (override files only — A2).

**Mount + sensor (resolves OQ-5, OQ-7; SIM-3, SIM-4):**

| Property | Value (Provisional values; contract fixed) | Rationale |
|----------|--------------------------------------------|-----------|
| Airframe target | **`gz_x500_patrol`** (camera variant of `gz_x500`) | OQ-7; M1 default + camera, no PX4 fork (A2) |
| Mount transform | forward, pitched slightly down, fixed offset from body origin | so a tag is in-frame at a checkpoint hover (SIM-4); the geometry 02 designs approach against (frozen contract) |
| Horizontal FOV | ~1.2 rad (~69°) | wide enough to hold a ~0.5 m tag at hover distance (Sequence 3, §4.5) |
| Resolution / rate | 640×480 @ 15 Hz *(Provisional — OQ-4)* | trades sim load vs image usefulness vs bag size (05); minimum-spec friendly (H1) |
| `frame_id` | `camera_link` *(Provisional — OQ-4)* | stable TF frame for 04's pose handling |

**Image topics (the 04/05 contract — see §4.4.4):** `sensor_msgs/Image` on `/drone/camera/image_raw` (04 subscribes) and a companion `sensor_msgs/CompressedImage` on `/drone/camera/image_raw/compressed` (05 records — keeps the bag manageable per 05 DoD AC-2). Both are bridged from the gz camera sensor via `ros_gz_image`.

*Traces to: SIM-3 / UAC-SIM-3, SIM-4 / UAC-SIM-4.*

#### 4.2.6 Patrol Bring-up Glue

**Type:** Module (launch/config, internal) · **Location:** a sim launch include under `sim/` (and/or a thin include consumed by 02's `mission_patrol.launch.py`) · **Dependencies:** 01 container/launch; 02 `mission_patrol.launch.py`; all components above

**Boundary:** Launches SITL with `gz_x500_patrol` + `patrol_world.sdf` + the image bridge, so that the M4 patrol (02) pointed at the same `checkpoints.yaml` positions traverses each checkpoint (SIM-5). Owns *wiring the stage*; delegates patrol logic, waypoint completion, and abort to 02.

**SIM-5 mechanism:** the glue exposes the checkpoint positions (from `checkpoints.yaml`) such that 02's patrol config is driven by the same source; "the drone visits each checkpoint AprilTag in turn" is verified by running 02's `mission_patrol.launch.py` against the world (§4.5 Sequence 4).

*Internal component — no cross-service contract. Traces to: SIM-1, SIM-3, SIM-5 / UAC-SIM-5.*

### 4.3 Layer View

#### 4.3.1 Layer Mapping

Layers are the asset layers from §3.1 (this is an asset docset; there is no UI/API/DB stack).

| Layer | Components | Key Responsibilities |
|-------|-----------|----------------------|
| **Config (source of truth)** | Checkpoint Config | The `checkpoint_id ↔ position ↔ tag_id` rows everything reads |
| **Assets (SDF / textures)** | Patrol World, AprilTag Model Library | Loadable world + reusable, hardware-aligned tag models |
| **Tooling / build** | World Composer (+ drift check) | Generate the world from config; enforce one-source-of-truth + portability |
| **Airframe / sensor override** | Camera Sensor Attachment | Add the RGB camera + publish image topics |
| **Bring-up / integration** | Patrol Bring-up Glue | Launch the stage; let the M4 patrol traverse it |

#### 4.3.2 Assets layer — Design Notes
**Conventions:** SDF worlds, Gazebo model-dir structure, no large binary meshes (`sim/README.md:13-15`).
**New in this design:** the patrol world template, the tag36h11 model library, and the generated world.
**Integration points:** assets are referenced by `model://` / repo-relative URIs only (SIM-6); the world is loaded by the bring-up layer.

#### 4.3.3 Tooling / Airframe layers — Design Notes
**Conventions:** Python tooling lives under `sim/tools/` + `scripts/`; PX4 overrides live under `sim/px4_sitl_overrides/` (sim/README.md:9) and never edit the host install (A2).
**New in this design:** the composer + drift check; the `gz_x500_patrol` camera override + ROS↔gz image bridge wiring.
**Integration points:** the composer reads Config and writes Assets; the airframe override is consumed by the bring-up layer and produces the image topics 04/05 bind to.

### 4.4 Systemic / Platform Interfaces

For an asset docset, the "systemic interfaces" are the **cross-service contracts** (the three this docset owns) plus configuration/portability and performance. There is no observability/security/auth surface in checked-in sim assets — those rows are explicitly scoped out below rather than padded with boilerplate.

#### 4.4.1 Interface Integration Summary

| Interface | Current State (§3) | Design Changes | Priority |
|-----------|--------------------|----------------|----------|
| Checkpoint-config contract (owned) | none | `sim/config/checkpoints.yaml` schema, consumed by 02 + 04 | P1 |
| AprilTag asset contract (owned) | none | tag36h11 family + ID range + size, 04's detector targets | P1 |
| Camera topic contract (owned) | no camera | `Image` + companion `CompressedImage` topics, 04 subs / 05 records | P1 |
| Configuration / portability | empty `sim/` tree | composer enforces repo-relative paths; CI drift + fresh-container load checks | P1 |
| Performance / capacity (sim RTF) | n/a | primitives-first authoring; non-binding RTF metric (H1) | P2 |
| Observability / Security / Auth | n/a | `[OOS: checked-in sim assets — no logging/auth/data surface; failures are visible at launch]` | — |

#### 4.4.2 Checkpoint-config contract (owned by 03; consumed by 02, 04)
**Current state:** none. **Design changes:** introduces `sim/config/checkpoints.yaml` (§4.2.3 schema). 02 reads `position` (world/ENU) to build waypoints and converts to PX4-NED at its boundary (A3); 04 reads `tag_id`→`checkpoint_id`. *(Provisional — OQ-2; pending the human's combined 5-pair review.)*
**Failure mode:** missing/malformed YAML → composer fails loudly at generation (Guards, §4.2.4); a position drift between YAML and the committed world is caught by the CI drift check (§4.4.5 Configuration).

#### 4.4.3 AprilTag asset contract (owned by 03; consumed by 04, reused Phase 4)
**Current state:** none. **Design changes:** tag36h11, IDs `0…N−1`, fixed physical size (§4.2.2), recorded so 04's `apriltag_ros` settings target it and Phase 4 hardware prints it unmodified (SIM-7).
**Failure mode:** a `tag_id` in the YAML with no matching model dir → composer Guard fails at generation (no silent missing marker).
**Security note:** `[OOS: AprilTag models are static textured assets; no auth, no data isolation, no trust boundary. The whole docset has no security-relevant surface — PRD §"Security & operational implications".]`

#### 4.4.4 Camera topic contract (owned by 03; consumed by 04, 05)
**Current state:** no camera on `gz_x500`. **Design changes (Provisional — OQ-4):**

| Topic | Type | Consumer | Notes |
|-------|------|----------|-------|
| `/drone/camera/image_raw` | `sensor_msgs/Image` | **04** (live frames for detection/capture) | bridged from gz camera via `ros_gz_image` |
| `/drone/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | **05** (records into MCAP bag) | companion; "compressed image to keep bag size manageable" (05 DoD AC-2; settled default #2) |

`frame_id: camera_link`, 640×480 @ 15 Hz (Provisional values; the names/types are the fixed contract surface, the numeric values await the joint 04/05 confirmation). **Relationship to `CheckpointCapture` (04's contract, recorded for coherence):** 04's `CheckpointCapture` carries `image_path` (a stored-path string), **not** pixels by-value; the live `Image` topic feeds 04's capture and the `CompressedImage` topic is what 05's bag records (settled default #2). This docset owns neither `CheckpointCapture` nor the recording — only the two camera topics.
**Failure mode:** if the camera sensor or bridge fails to publish, `ros2 topic hz` shows zero rate at bring-up (SIM-3 AC) — a loud, launch-time, fully-reversible failure (edit the override and relaunch).

#### 4.4.5 Cross-cutting Failure Modes

Environment-introduced failure states a deployed-in-sim asset set can find itself in. Categories that don't apply to checked-in sim assets are explicitly `[OOS]`.

| Category | Failure mode | Detection | Degraded behavior | Recovery |
|----------|--------------|-----------|-------------------|----------|
| **Persistent state (assets on disk)** | Missing model/texture referenced by world (e.g. deleted/untracked file, absolute path) | World load error in Gazebo; composer Guard rejects abs/host paths; fresh-container load check (SIM-6) | World fails to render at launch (loud, immediate) | Restore the missing asset under `sim/` / fix the URI; relaunch — fully reversible |
| **Persistent state (config↔world drift)** | Committed `patrol_world.sdf` markers diverge from `checkpoints.yaml` | CI drift check `check_drift()` (§4.2.4) | CI fails the PR; no drifted world reaches `main` | Re-run the composer, commit the regenerated world |
| **Network dependency (ROS↔gz bridge)** | Image bridge not up / camera sensor not publishing | `ros2 topic list` lacks the topic; `ros2 topic hz` = 0 (SIM-3 AC) | 04 has no frames to detect; 05 has nothing to record — caught at bring-up, not in flight | Fix the airframe override / bridge launch; relaunch |
| **Performance / capacity** | Too much geometry → physics drops below real-time on minimum spec (H1) | Gazebo RTF < ~1.0 during the M4 patrol (non-binding metric) | Patrol still completes, just slower-than-real-time | Simplify geometry (Tenet 1: primitives-first); re-profile |
| **Configuration** | Malformed YAML (missing field, dup `tag_id`, non-numeric position) | Composer Guards fail at generation | Generation aborts; no bad world produced | Fix the YAML; re-run composer |
| Identity provider | (all sub-modes) | `[OOS: no auth/identity surface in sim assets]` |
| Mesh / cross-cluster | (all sub-modes) | `[OOS: single-host sim; no mesh/cluster]` |
| Plugin / extension | (all sub-modes) | `[OOS: no runtime plugins; assets are static SDF/config loaded by Gazebo]` |

### 4.5 Key Interaction Sequences

#### Sequence 1: Author/edit a checkpoint → world reflects it (SIM-2 happy path)
```
Developer            checkpoints.yaml      World Composer          patrol_world.sdf      Gazebo (SITL)
  |                       |                     |                        |                    |
  ├─ edit position ──────►│                     |                        |                    |
  ├─ run compose_world ──────────────────────► │                        |                    |
  │                       │◄── read rows ───────┤                        |                    |
  │                       │   (Guards: ids/paths)│── emit marker block ─►│                    |
  ├─ relaunch SITL ─────────────────────────────────────────────────────┼── load world ─────►│
  │                                                                       │   ◄── markers at ──┤
  │   ◄────────────── ≥3 tag markers at YAML ENU positions, no SDF edit ──────────────────────┤
```

#### Sequence 2: SITL up → camera topics publish (SIM-3 happy path)
```
Bring-up Glue        PX4 SITL (gz_x500_patrol)   gz camera sensor    ros_gz_image bridge     ROS 2
  ├─ launch ─────────►│                              |                    |                    |
  │                   ├─ spawn airframe + camera ───►│                    |                    |
  │                   │                              ├─ gz.msgs.Image ───►│                    |
  │                   │                              │                    ├─ sensor_msgs/Image ►│ /drone/camera/image_raw
  │                   │                              │                    ├─ CompressedImage ──►│ /drone/camera/image_raw/compressed
  │   ◄── ros2 topic hz shows steady non-zero rate on both topics ──────────────────────────────┤
```

#### Sequence 3: Camera sees a tag at a checkpoint hover (SIM-4 — the riskiest geometry)
```
Drone @ checkpoint hover    Camera (mount pose + FOV)        Frame
  ├─ hover at cp_north per approach geometry ─►│              |
  │                                            ├─ project tag (size, distance) ─►│
  │   ◄──── tag36h11 (id 0) within frame, large enough to resolve ──────────────┤
  │   (if not: re-tune mount pitch / FOV against this reference checkpoint, lock the contract)
```

#### Sequence 4: M4 patrol traverses every checkpoint (SIM-5 — integrative)
```
Operator        02 mission_patrol.launch.py     Bring-up Glue / World     checkpoints.yaml
  ├─ launch patrol ─►│                               |                         |
  │                  ├─ read checkpoint positions ───────────────────────────► │
  │                  │  (02 converts ENU→PX4-NED at its boundary — A3)         |
  │                  ├─ fly to cp_north ─► hover ─► fly to cp_east ─► … ───────►│ (world renders markers)
  │   ◄──────────── drone visits each checkpoint AprilTag in turn ─────────────┤
```

### 4.6 Data Model Changes (Consolidated)

No runtime datastore. Persistent artifacts are checked-in repo files (the "data model" is the asset/config tree).

| Artifact | Change | Detail |
|----------|--------|--------|
| `sim/config/checkpoints.yaml` | **New** | Shared checkpoint schema (§4.2.3); the source of truth |
| `sim/worlds/patrol_world.template.sdf` | **New** | Terrain + obstacles template (no markers) |
| `sim/worlds/patrol_world.sdf` | **New (generated, checked-in)** | Composer output: template + marker includes |
| `sim/models/apriltag_36h11_<id>/` | **New** | One tag36h11 model dir per checkpoint ID |
| `sim/px4_sitl_overrides/gz_x500_patrol/` | **New** | Camera airframe variant + mount transform/FOV + bridge config |
| `sim/tools/compose_world.py` + drift check | **New** | World Composer + CI drift check |

### 4.7 UX Mocks

`[OOS: no UI/UX surface. The "interface" is a launched Gazebo render of the world + markers and ROS 2 topics, exercised via CLI (gz sim, ros2 topic hz/list). No screens, no state matrix — verification is by launch + topic inspection, covered in §4.5 and §6 testing.]`

---

## 5. Design Questions FAQ

### Q1: Main components and interactions
Six components (§4.2.0): **Checkpoint Config** (`checkpoints.yaml`, the source of truth) feeds the **World Composer**, which places **AprilTag Model Library** markers into the **Patrol World**; the **Camera Sensor Attachment** adds an RGB camera to the airframe and publishes the image topics; the **Patrol Bring-up Glue** launches SITL + world + camera so the M4 patrol (02) traverses the checkpoints. Build order: Camera Attachment + world skeleton first (riskiest contract), then config + tags + capture geometry, then patrol integration + portability (§6).

### Q2: Core contracts and data models
This docset has no API/DB; its contracts are: (1) the `checkpoints.yaml` schema `{checkpoint_id:string, position:{x,y,z} ENU, tag_family:string, tag_id:int}` (§4.2.3 / §4.4.2); (2) the camera topics `/drone/camera/image_raw` (`sensor_msgs/Image`) + `/drone/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`) (§4.2.5 / §4.4.4); (3) the tag36h11 family + ID range + size (§4.2.2 / §4.4.3). The "data model" is the checked-in asset tree (§4.6). All topic names/types in Q2 match §4.2/§4.4 exactly.

### Q3: Deployment and infrastructure dependencies
Runs inside 01-platform's sim container (no new infra). Provisions: a `gz_x500_patrol` airframe override under `sim/px4_sitl_overrides/`, a ROS↔gz image bridge (`ros_gz_image`, from 01's base), and a generated world under `sim/worlds/`. No new services, ports, or datastores. The only build step is the World Composer + CI drift check.

### Q4: External components and interfaces
External deps: **gz-sim 8 / PX4 SITL `gz_x500` / ROS 2 Jazzy + `ros_gz` bridge** (all from 01-platform, §3.2 verified). Cross-docset consumers: **02** (positions), **04** (camera topic + tag models + `tag_id`→`checkpoint_id`), **05** (`CompressedImage` recording). Each external/consumer interface has a §4.4 row.

### Q5: Testing strategy
- **Unit:** composer Guards + `check_drift()` (parse YAML, reject abs paths, dup `tag_id`, emit-vs-expected positions) — no ROS/Gazebo needed (mirrors the plan's "don't mock the simulator; do test construction logic" discipline).
- **Integration (SITL):** SIM-1 (world loads, no errors), SIM-2 (≥3 markers at YAML positions), SIM-3 (`ros2 topic hz` steady on both topics), SIM-4 (tag in-frame at a hover), SIM-5 (run 02's `mission_patrol.launch.py`, drone visits each).
- **Portability:** SIM-6 fresh-container clone + launch, no host/abs-path errors.
- **Non-binding perf:** read Gazebo RTF during the M4 patrol (H1).

### Q6: Security implications and auth interactions
None. Checked-in sim assets and config: no auth, no user data, no network surface, no persistent/destructive operations (PRD §"Security & operational implications"). Failures are visible at launch and reversible by editing/regenerating assets. `[Security model dimension is N/A for this docset — explicitly, not by omission.]`

### Q7: Technical risks and open questions
- **OQ-2 (checkpoint schema)** — Provisional, pending human 5-pair review (joint 02/04). §4.4.2.
- **OQ-4 (camera topic name/res/rate/frame_id)** — Provisional, pending human 04/05 review. §4.4.4.
- **OQ-5 (camera-sees-tag geometry)** — Resolved with provisional values; the coupled mount×FOV×size×approach tuning is the scariest unknown (PRD rabbit hole); contained by tuning one reference checkpoint (Sequence 3) and freezing the contract before 02 designs approach.
- **OQ-7 (airframe camera attach)** — Resolved (`gz_x500_patrol`); residual risk a camera variant is needed vs stock; bought down by validating SIM-3 in M1-skeleton before world build.
- **OQ-8 (promote RTF to binding AC)** — Deferred to human/M2 profiling; kept non-binding (the DoD sets no perf contract).

All Q7 statuses match §2 (OQ-2/OQ-4 Provisional, OQ-1/3/5/6/7 Resolved, OQ-8 Deferred).

---

## 6. Implementation Plan

### 6.0 Linear Project
**Project:** TBD (no upstream Linear project bound at design time; retrofitted post-approval).
**Team:** TBD · **Initiative:** Phase 1 (pre-hardware simulation) · **Created from:** Section 6 of this document.

### 6.1 Milestone Overview

Walking-skeleton: each milestone is a launchable, demonstrable stage — never a half-built world. The "skeleton" is the riskiest contract (camera publishing) validated first; each later layer adds a customer-visible capability to the same running world.

| # | Milestone | Type | Shippable Demo | Scope | Deps | Exit Criteria | Linear |
|---|-----------|------|----------------|-------|------|---------------|--------|
| M1 | Camera + world skeleton | skeleton | Launch SITL against a crude world and see `ros2 topic hz` stream steady frames on both camera topics | `gz_x500_patrol` override + image bridge (SIM-3); minimal terrain+obstacle world that loads (SIM-1) | 01 (sim container, `gz_x500`, bridge) | World renders with no load errors; both camera topics publish at steady non-zero rate | *post-approval* |
| M2 | Checkpoints + config + capture geometry | layer 1: checkpoints & capture | Edit `checkpoints.yaml`, regenerate, relaunch, and see ≥3 tag markers at the new positions with a tag in-frame at a hover | Checkpoint Config schema + composer + drift check (SIM-2); tag36h11 library + ID range (SIM-7); mount pose/FOV tuned so a tag is in-frame (SIM-4) | M1 | ≥3 markers at YAML ENU positions; editing a position + relaunch moves the marker (no SDF edit); a hover frame contains a resolvable tag | *post-approval* |
| M3 | Integration + portability | layer 2: end-to-end patrol & portability | Clone fresh, launch in-container, run the M4 patrol, and watch the drone visit each tagged checkpoint in turn | Patrol Bring-up Glue points 02's `mission_patrol.launch.py` at the same positions (SIM-5); fresh-container no-host-path load (SIM-6) | M2; 02 M4 patrol | M4 patrol traverses every checkpoint in SITL; fresh-container checkout loads clean with no host/abs-path errors | *post-approval* |

### 6.2 Milestone Details

#### M1: Camera + world skeleton
**Type:** skeleton · **Goal:** the thinnest launchable stage — a crude world that loads and a drone whose camera publishes — exercising the asset, airframe-override, and ROS-topic layers at minimum thickness.
**Shippable demo:** "Launch SITL against `patrol_world.sdf` and run `ros2 topic hz /drone/camera/image_raw` and `/drone/camera/image_raw/compressed` — both show a steady non-zero rate, and Gazebo renders the crude world with no load errors."
**Dependencies:** 01-platform sim container, `gz_x500`, `ros_gz` bridge.
**Exit criteria:** SIM-1 (world loads clean) + SIM-3 (both topics publish steadily) demonstrated end-to-end.

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| Checkpoint markers + config | design §4.2.3/§4.2.4 (SIM-2) | M2 |
| Camera-sees-tag geometry tuning | design §4.2.5 (SIM-4) | M2 |
| M4 patrol traversal | design §4.2.6 (SIM-5) | M3 |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Deps |
|---|------|---------------|-----------|-------|------|------|
| T1.1 | Author crude world template (terrain + ≥2 boxes + ≥2 trees, primitives) | `sim/worlds/patrol_world.template.sdf` (new) | Patrol World | Assets | M | — |
| T1.2 | Author `gz_x500_patrol` camera airframe override (sensor + mount transform + FOV) | `sim/px4_sitl_overrides/gz_x500_patrol/` (new) | Camera Sensor Attachment | Airframe | L | T1.1 |
| T1.3 | Wire ROS↔gz image bridge for `Image` + `CompressedImage` topics | `sim/px4_sitl_overrides/gz_x500_patrol/` (modify); bring-up include (new) | Camera Sensor Attachment | Bring-up | M | T1.2 |
| T1.4 | Bring-up launch: SITL + world + camera bridge | sim launch include (new) | Patrol Bring-up Glue | Bring-up | M | T1.1, T1.3 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Integration | World load + camera publish | World renders no errors (SIM-1); `ros2 topic list` shows both topics; `ros2 topic hz` steady non-zero (SIM-3) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| `sim/README.md` update | developer | How to launch the patrol world + camera; topic names |

#### M2: Checkpoints + config + capture geometry
**Type:** layer 1: checkpoints & capture · **Goal:** add the checkpoint source-of-truth, the placed tag markers, and the mount geometry that lets the drone see a tag.
**Shippable demo:** "Edit a `position` in `checkpoints.yaml`, run the composer, relaunch — the corresponding tag marker has moved (no SDF edit), and at a checkpoint hover the camera frame contains a resolvable tag36h11."
**Dependencies:** M1.
**Exit criteria:** SIM-2 (≥3 markers at YAML positions; edit-and-move), SIM-7 (tag36h11 + ID range documented), SIM-4 (tag in-frame at hover).

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| M4 patrol traversal of checkpoints | design §4.2.6 (SIM-5) | M3 |
| Fresh-container portability sign-off | design §4.4.5 (SIM-6) | M3 |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Deps |
|---|------|---------------|-----------|-------|------|------|
| T2.1 | Define `checkpoints.yaml` schema + ≥3 example checkpoints (ENU) | `sim/config/checkpoints.yaml` (new) | Checkpoint Config | Config | S | T1.1 |
| T2.2 | Author tag36h11 model library (one dir per ID 0…N−1, textures) | `sim/models/apriltag_36h11_<id>/` (new) | AprilTag Model Library | Assets | M | T2.1 |
| T2.3 | World Composer: generate world from YAML + portability Guards | `sim/tools/compose_world.py` (new) | World Composer | Tooling | L | T2.1, T2.2 |
| T2.4 | CI drift check: committed world markers == YAML | `scripts/` drift check (new); CI config (modify) | World Composer | Tooling | M | T2.3 |
| T2.5 | Tune mount pose/FOV against a reference checkpoint; lock contract | `sim/px4_sitl_overrides/gz_x500_patrol/` (modify) | Camera Sensor Attachment | Airframe | M | T1.2, T2.2 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | Composer Guards + drift | Reject abs/host paths; reject dup/missing `tag_id`; emitted positions == YAML; `check_drift()` flags a mismatch |
| Integration | Markers + geometry | ≥3 markers at YAML ENU positions (SIM-2); edit position + relaunch moves marker; hover frame contains a resolvable tag (SIM-4) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| `sim/README.md` + schema doc | developer / 02 / 04 | `checkpoints.yaml` schema; tag family/ID range; how to add a checkpoint |

#### M3: Integration + portability
**Type:** layer 2: end-to-end patrol & portability · **Goal:** the full stage runs end-to-end (M4 patrol traverses every checkpoint) and loads clean from a fresh containerized checkout.
**Shippable demo:** "On a fresh clone, launch in-container and run 02's `mission_patrol.launch.py` against the world — the drone flies to each tagged checkpoint in turn, with no missing-asset or host-path errors."
**Dependencies:** M2; 02's M4 patrol.
**Exit criteria:** SIM-5 (M4 patrol traverses every checkpoint), SIM-6 (fresh-container load, no host paths).

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| Perception capture / `CheckpointCapture` | PRD Out-of-Scope (owned by 04) | never (other docset) |
| Bag recording of the camera feed | PRD Out-of-Scope (owned by 05) | never (other docset) |
| Promoting RTF to a binding AC | design §2 OQ-8 | combined human review / M2 profiling |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Deps |
|---|------|---------------|-----------|-------|------|------|
| T3.1 | Bring-up glue exposes checkpoint positions so 02's patrol is driven by the same YAML | sim launch include (modify); 02 launch include hook | Patrol Bring-up Glue | Bring-up | M | T2.1, T1.4 |
| T3.2 | Run M4 patrol against the world; verify each checkpoint visited in turn | tests/integration (new) | Patrol Bring-up Glue | Bring-up | M | T3.1 |
| T3.3 | Fresh-container portability check: clone + launch, assert no host/abs-path errors | `scripts/` portability check (new); CI (modify) | World Composer / Patrol World | Tooling | M | T2.3, T1.4 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Integration | Patrol traversal | Run `mission_patrol.launch.py`; drone visits each checkpoint AprilTag in turn (SIM-5) |
| E2E / Portability | Fresh-container load | Clone fresh, launch in sim container; world + models + textures load with no host/abs-path errors (SIM-6) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| `sim/README.md` end-to-end | developer | Fresh-checkout → launch → run patrol against checkpoints |

### 6.3 Layered Delivery Sequence

**Skeleton + layering rationale:**
1. **M1 (skeleton, camera + world)** is the thinnest end-to-end slice: a world that loads *and* a drone whose camera publishes. It crosses the assets, airframe-override, and ROS-topic layers at minimum thickness. It is first because the **camera-publishing contract is the riskiest** (airframe camera attach, OQ-7 / PRD rabbit hole) — if `gz_x500` can't get a camera topic, everything downstream is blocked, so we validate it before authoring the full world.
2. **M2 (layer 1: checkpoints & capture)** thickens the running world with the checkpoint source-of-truth, placed tag markers, and the mount geometry that makes a tag visible. After M2 the demo shows config-driven markers moving and a tag in-frame — the SIM-4 geometry (second-riskiest, coupled tuning) is locked here, before 02 designs approach geometry against it.
3. **M3 (layer 2: end-to-end patrol & portability)** adds the integrative payoff: the M4 patrol traverses every checkpoint and the whole thing loads from a fresh container. This is the stage that feeds integrative exit item 1.

**What gets demoable, when:**
- After M1: launch the world, see steady camera frames.
- After M2: M1 + config-driven tag markers + a tag in-frame at a hover.
- After M3: M2 + the M4 patrol visiting every checkpoint, from a fresh container.

**Scope-shedding plan:** if schedule slips, shed M3 (the patrol-integration + portability layer is the last to add); M2 alone still demonstrates the config-driven checkpoint world + camera; M1 alone is still a shippable "world loads + camera publishes" demo. Hard floor: the camera-publishing contract (M1) — nothing downstream is demoable without it.

**Parallel work opportunities:** within M2, the tag model library (T2.2) and the composer (T2.3) can be drafted in parallel once the schema (T2.1) is fixed; M1's world template (T1.1) and airframe override (T1.2) can be drafted concurrently. M1 itself is inherently serial (skeleton).

### 6.4 Definition of Done
A milestone is complete when:
- [ ] All tasks implemented and reviewed
- [ ] All specified tests pass (unit composer/drift; integration SITL; E2E portability)
- [ ] **Shippable demo runs end-to-end** (M1: world loads + camera publishes; M2: + config-driven markers + tag in-frame; M3: + M4 patrol traversal from fresh container)
- [ ] `sim/README.md` documentation updated for the milestone
- [ ] No host-specific / absolute asset paths (SIM-6 discipline) introduced
- [ ] Cross-service contract surfaces (§4.4) unchanged or, if changed, the PRD is revised first (Scope Authority)

---

## 7. Changelog

### v0.1.1 — 2026-06-03
**Self-review pass (ReviewDesign, 13-dimension rubric incl. D2 PRD-trace audit).** Result: all 13 dimensions Strong; zero findings at the medium severity floor; PRD-trace audit enumerated every design surface (6 components, 3 cross-service contracts, 2 topics, 6 §4.6 artifacts, 12 §6.2 tasks) and found zero unauthorized scope — every surface traces to a named PRD FR (SIM-1…SIM-7) or to a marked-internal inferred requirement (World Composer + drift check → INF-S2/S3, Tenet 2). No content revision required.
**Sections touched:** §4.1 (added INF-S2/S3 trace + internal markers to World Composer row), §4.2.4 (trace footer cites INF-S2/INF-S3), §2 OQ-5 (added explicit decision target T2.5), §3.2 (04 DoD citation upgraded from a generic `§7 + AC-3` section reference toward file:line `04-perception/dod.md:88,52`), §7 (this entry). Two residual sub-medium polish notes and three deferred decisions recorded for the human's combined 5-pair review.

### v0.1.0 — 2026-06-03
**Initial version** — Created via CreateDesign workflow from `prd.md` (rev 2). Resolved design-target OQs (OQ-1 tag36h11/contiguous IDs; OQ-3 generate-from-YAML; OQ-5 mount/FOV provisional; OQ-6 layout; OQ-7 `gz_x500_patrol`). Held OQ-2 (checkpoint schema) and OQ-4 (camera topics) Provisional pending the human's combined 5-pair review; held OQ-8 (RTF-as-binding-AC) Deferred. Self-reviewed against the 13-dimension ReviewDesign rubric.

---

## Appendix B: User Acceptance Criteria

Carried from `prd.md` Appendix B (UAC-SIM-1…6), each traced into §4.1.

**UAC-SIM-1 (Patrol World):** GIVEN the repo + running PX4 SITL, WHEN the custom world loads, THEN Gazebo Harmonic renders terrain + boxes + trees with no load errors and the world lives under `sim/worlds/`. *(near-real-time = non-binding target per H1)*
**UAC-SIM-2 (Checkpoint Config + Composer + Model Library):** GIVEN a `checkpoints.yaml` with ≥3 entries, WHEN the world is generated/loaded, THEN ≥3 tag36h11 models appear at the YAML ENU positions and editing a position + relaunching moves the marker with no SDF edit.
**UAC-SIM-3 (Camera Sensor Attachment):** GIVEN SITL running, WHEN ROS 2 inspects topics, THEN `/drone/camera/image_raw` (`sensor_msgs/Image`) publishes at a steady non-zero rate AND a companion `/drone/camera/image_raw/compressed` (`sensor_msgs/CompressedImage`) publishes for the bag, at the documented name/res/rate/`frame_id`. *(CompressedImage companion is the settled cross-docset default — Provisional, OQ-4.)*
**UAC-SIM-4 (Camera Sensor Attachment):** GIVEN the camera at its fixed mount/FOV and a configured hover pose, WHEN the drone hovers at the checkpoint, THEN a tag36h11 of the chosen size is within frame and resolvable.
**UAC-SIM-5 (Patrol Bring-up Glue + Checkpoint Config):** GIVEN the world + 02's M4 patrol configured to the same YAML positions, WHEN the patrol runs in SITL, THEN the drone visits each checkpoint in turn, with positions in world/ENU (02 converts to PX4-NED at its boundary).
**UAC-SIM-6 (Patrol World + World Composer):** GIVEN a fresh checkout in the 01-platform sim container, WHEN the world launches, THEN it loads with no host-specific assets and no absolute paths — all models/textures live under `sim/`.

#### Inferred Requirements [INFERRED]
**INF-S1: Companion CompressedImage topic** *(ref: UAC-SIM-3)* — GIVEN SITL running, WHEN 05 records the bag, THEN a `sensor_msgs/CompressedImage` companion topic exists to keep bag size manageable. *(Settled cross-docset default #2 + 05 DoD AC-2; Provisional — OQ-4.)*
**INF-S2: Config↔world drift guard** *(ref: UAC-SIM-2)* — GIVEN a generated world, WHEN CI runs, THEN a drift check asserts the committed world's marker positions match `checkpoints.yaml` (one-source-of-truth, Tenet 2; resolves the OQ-3 static-vs-generated drift risk). *(Internal infrastructure; no cross-service contract.)*
**INF-S3: Generation/load Guards fail loudly** *(ref: UAC-SIM-2/6)* — GIVEN a malformed YAML or a `tag_id` with no model dir or an absolute path, WHEN the composer runs, THEN generation aborts with a clear error rather than emitting a broken/partial world. *(Internal infrastructure; no cross-service contract.)*
