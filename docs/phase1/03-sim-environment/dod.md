# Definition of Done — Simulation Environment & Assets

**Phase 1 docset:** 3 of 5 · **Milestones:** M5
**Lifecycle status:** DoD ✅ · PRD ✅ · Design ✅
**Source:** docs/phase1_simulation_plan.md — M5 ("Custom world with checkpoints and AprilTags"); cross-cutting: "Repo structure", "Containerization", "What's explicitly NOT in Phase 1", "Phase 1 exit checklist"
**Stakeholders:** Project owner (solo dev) — operator/maintainer authoring and patrolling the world; downstream — Phase 1 consumers 02-mission-control (patrol target positions) and 04-perception (AprilTag detection + camera frames), plus Phase 4 indoor-VIO which reuses the same AprilTag assets and detection on hardware; reviewers — PR reviewers gating merge to a working-in-sim `main`.
**Depends on:** 01-platform (Gazebo Harmonic / PX4 SITL / ROS 2 base, `colcon build`, container env) — foundation for loading a world and attaching a sensor; 02-mission-control (patrol mission able to fly a waypoint sequence — used to exercise the world's checkpoints).
**Consumed by:** 04-perception (AprilTag models + RGB image topic feed the detection/capture node); 02-mission-control (checkpoint world positions correspond to patrol waypoints); later phases — Phase 4 (same AprilTag assets for indoor relocalization).

## 1. Intent
Deliver a checked-in Gazebo Harmonic patrol environment — a crude-but-meaningful world with AprilTag checkpoint markers at known, configurable positions and a simulated RGB camera on the drone — so the M4 patrol mission can visit each checkpoint and "look at" it. This is the physical stage on which perception (M6) and logging (M7) are exercised end-to-end before any hardware exists.

## 2. Scope
**In scope:**
- A custom Gazebo Harmonic world (SDF) representing a patrol environment: flat terrain, a few building-like boxes, a few trees as obstacles.
- AprilTag fiducial markers as textured Gazebo models, placed at >=3 known checkpoint positions driven by a YAML config.
- A simulated RGB camera attached to the SITL drone that publishes a ROS 2 `sensor_msgs/Image` topic.
- The checkpoint-position config schema (YAML) that defines where tags sit in the world and how they map to checkpoint identifiers.
- Asset organization under `sim/worlds/`, `sim/models/`, and any `sim/px4_sitl_overrides/` needed to attach the camera sensor.

**Out of scope (explicit deferrals — item · rationale · target):**
- Photorealistic / high-fidelity environments · M5 explicitly says "start crude … the point isn't photorealism" · Phase 5 (Isaac Sim, trail navigation).
- Isaac Sim · Gazebo is enough for now · Phase 5+ for trail data, Phase 8 for RL.
- Using AprilTags for relocalization / VIO / SLAM · sim provides ground-truth pose, tags are for exercising the perception pipeline only · Phase 3 (optional fusion) / Phase 4 (VIO-only indoor).
- The AprilTag detection node and `CheckpointCapture` message · owned by 04-perception · this docset only provides the tags and camera feed it consumes.
- Mission/waypoint logic and the state machine that drives the drone between checkpoints · owned by 02-mission-control · this docset only provides the checkpoint positions to patrol between.
- Real object detection (YOLO/TensorRT) · AprilTag scaffolding is enough · Phase 3.

## 3. Capabilities (must-do — seeds the PRD's functional requirements)
1. **(P1) Custom patrol world loads in Gazebo Harmonic.** A checked-in SDF world with flat terrain plus a few building-like boxes and trees as obstacles loads cleanly under PX4 SITL.
   - *Customer scenario:* the dev launches SITL against the project world instead of a vanilla empty plane and sees a recognizable patrol environment.
   - *Pain removed:* without a known-geometry world there is nowhere to place checkpoints and no environment to patrol — the rest of M5–M7 has no stage.
2. **(P1) AprilTag checkpoint markers at YAML-configured positions.** >=3 AprilTag fiducials, as textured Gazebo models, appear in the world at positions defined by a YAML config (each tag mapped to a checkpoint identifier).
   - *Customer scenario:* the dev edits one YAML file to move/add a checkpoint and the world reflects it on next launch.
   - *Pain removed:* hard-coded tag positions buried in SDF would block perception (M6) from being exercised end-to-end and would not transfer to the Phase 4 hardware checkpoints.
3. **(P1) Simulated RGB camera publishes a ROS 2 image topic.** A camera sensor attached to the SITL drone publishes a `sensor_msgs/Image` topic visible to ROS 2 while SITL runs.
   - *Customer scenario:* the perception node (04) subscribes to a live camera feed and the logging pipeline (05) records it, exactly as a real camera would feed them.
   - *Pain removed:* with no camera topic, perception capture and bag recording of imagery cannot be built or tested at all.
4. **(P1) M4 patrol traverses every checkpoint.** The existing M4 patrol mission, pointed at the world's checkpoint positions, visits each checkpoint AprilTag in turn.
   - *Customer scenario:* the operator runs the patrol and watches the drone fly to each tagged checkpoint and hold there.
   - *Pain removed:* a world whose checkpoints the mission can't actually reach proves nothing about the integrated patrol-plus-perception path.
5. **(P2) AprilTag family/ID assignment that is reusable on hardware.** Tag family and ID assignment chosen so the same fiducials and detection settings carry unchanged into Phase 4 indoor relocalization.

## 4. Acceptance criteria / Definition of Done (falsifiable — seeds the PRD's UACs)
*Sourced from the M5 Exit criteria. This docset is a primary contributor to integrative exit-checklist item 1 (patrol with image capture) and a hard dependency for items 8 and 11, but owns no exit-checklist item solo — see §9.*

- [ ] **AC-1** — GIVEN the project repo and a running PX4 SITL session, WHEN the custom world is loaded, THEN Gazebo Harmonic renders the patrol environment (flat terrain + building-like boxes + trees) without load errors. (M5 Exit: "custom world loads".)
- [ ] **AC-2** — GIVEN a checkpoint-positions YAML with >=3 entries, WHEN the world is generated/loaded, THEN >=3 AprilTag checkpoint markers appear at the YAML-configured world positions. (M5 Exit: "custom world loads with 3+ checkpoint AprilTags at YAML-configured positions".)
- [ ] **AC-3** — GIVEN the world and the M4 patrol mission configured to the same checkpoint positions, WHEN the patrol runs in SITL, THEN the drone visits each checkpoint AprilTag in turn. (M5 Exit: "Patrol mission from M4 visits each in turn"; contributes to exit-checklist item 1.)
- [ ] **AC-4** — GIVEN SITL running with the drone in the world, WHEN ROS 2 inspects topics, THEN a simulated RGB camera publishes a `sensor_msgs/Image` topic at a steady rate. (M5 Exit: "Simulated RGB camera attached to the drone publishes a ROS 2 image topic".)
- [ ] **AC-5** — GIVEN a fresh checkout inside the containerized sim environment (per 01-platform), WHEN the world is launched, THEN it loads with no host-specific assets or absolute paths (all referenced models/textures live in `sim/` and are repo-checked-in). (Cross-cuts "Containerization"; supports exit-checklist item 9's "works in-container" intent.)

## 5. Interfaces
**Owns (contracts this docset defines that others depend on):**
- `sim/worlds/<patrol_world>.sdf` (or `.world`) — the canonical Phase 1 patrol world.
- `sim/models/<apriltag_*>/` — AprilTag fiducial Gazebo models (model.sdf + model.config + texture), reusable on hardware in Phase 4.
- Checkpoint-positions config: a YAML file (location/name TBD — see §7) defining `{checkpoint_id, position (world frame), tag_family, tag_id}` per checkpoint. This is the shared map between the world, the patrol waypoints (02), and perception's checkpoint identification (04).
- RGB camera ROS 2 topic — a `sensor_msgs/Image` topic from the drone-mounted camera (topic name TBD — see §7).
- Camera sensor attachment to the SITL airframe (via `sim/px4_sitl_overrides/` or world/model SDF).

**Consumes (from other docsets / PX4):**
- From 01-platform: Gazebo Harmonic + PX4 SITL + ROS 2 Jazzy base, the `ros2_ws` build, and the container environment that loads worlds.
- From 02-mission-control: the M4 patrol mission capable of flying a waypoint sequence (used to traverse the checkpoints).
- From PX4: SITL airframe (e.g. `gz_x500`) and the simulator runtime that hosts the world and sensor.

## 6. Settled constraints (do NOT relitigate — cite the source)
- **Gazebo Harmonic (gz-sim 8), not Gazebo Classic.** Classic is deprecated; PX4 modern integration and NVIDIA tooling target Harmonic. (Phase 1 plan, "Target stack" + "What I'd ask not to relitigate"; ADR-0001 "Neutral".)
- **Ubuntu 24.04 + ROS 2 Jazzy.** Project-wide stack. (ADR-0001 Decision; Phase 1 plan "Target stack".)
- **AprilTags are for exercising the perception pipeline, not relocalization in Phase 1.** "In sim, you don't need them for relocalization — the simulator gives you ground-truth pose." (Phase 1 plan, M5.)
- **Same AprilTag assets/detection must run unmodified on hardware in Phase 4.** This sim/hardware alignment is a stated design value. (Phase 1 plan, M5: "the AprilTag detection node you'll write *also* runs unmodified on real hardware in Phase 4".)
- **Start crude.** No photorealism in Phase 1; higher-fidelity environments are Isaac Sim / Phase 5. (Phase 1 plan, M5 + "What's explicitly NOT in Phase 1".)
- **Checkpoint positions are config-driven (YAML), not hard-coded.** (Phase 1 plan, M5 Exit; consistent with "Mission config loaded from a YAML file" discipline.)
- **Assets checked into the monorepo under `sim/`; avoid large binary meshes — prefer simple primitives or external references.** (sim/README.md conventions; Phase 1 plan "Repo structure".)

## 7. Open decisions (handed to /drive — each: question · decision target · why open)
- **AprilTag family and ID range** · decide in PRD/Design · choice (e.g. tag36h11 vs tag25h9) affects detection robustness and must be the family used unmodified in Phase 4; the plan leaves the specific family open.
- **Checkpoint-config file location, name, and exact schema** · decide in PRD/Design · the world, the patrol waypoints (02), and perception (04) must agree on one schema mapping `checkpoint_id ↔ world position ↔ tag id`; whether this lives under `sim/`, `patrol_bringup`, or is shared is unresolved. (Must be reconciled jointly with 02 §7 "YAML schema shape" and 04 §7 "checkpoint_id namespace / mapping" during /drive so one schema satisfies all three consumers.)
- **Whether the world is authored as static SDF or generated from the YAML at launch** · decide in PRD/Design · static SDF is simplest; generation keeps a single source of truth for checkpoint positions but adds tooling.
- **RGB camera topic name, resolution, frame rate, and frame_id** · decide in PRD/Design · these become the contract perception (04) and logging (05) bind to; resolution/rate trade sim load against image usefulness and bag size.
- **Camera mount pose and FOV on the airframe** · decide in PRD/Design · must let the drone actually "see" a tag when hovering at a checkpoint; affects waypoint approach geometry in 02.
- **World extent and obstacle layout** · decide in PRD/Design · enough geometry to be "meaningful" without slowing physics; the plan says "a flat plane, some building-like boxes, a few trees" but leaves specifics open.
- **Which SITL airframe target the camera attaches to** · decide in PRD/Design · `gz_x500` is the M1 default; attaching a camera may require an override/variant.

## 8. Assessment signals (so prd-engine right-sizes the PRD)
| Dimension | Value | One-line justification |
|---|---|---|
| Nature | greenfield / infrastructure | New simulation assets and sensor config; no existing behavior modified. |
| Complexity | moderate | A crude world, a few tag models, one camera sensor, and a YAML config — bounded, mostly authoring not algorithms. |
| Urgency | standard | Sequenced milestone (M5); blocks M6/M7 but no external deadline. |
| Risk | low | Sim-only assets; failures are visible at launch and fully reversible; no hardware, data, or auth impact. |
| Reversibility | fully-reversible | Assets and config are edited/regenerated freely; nothing persistent or destructive. |
| Scope | cross-service | Defines the camera topic + checkpoint-config contracts that 02, 04, and 05 all bind to. |
| Audience | developer | Single dev / small team building and patrolling the world. |
**Suggested PRD tier:** Lightweight (Complexity moderate × Risk low → Lightweight per prd-engine's Complexity×Risk matrix; the cross-service interface contracts are the only thing arguing upward, but they're thin and stated here, so Lightweight holds).

## 9. Traceability
- **Milestones:** M5 — "Custom world with checkpoints and AprilTags" (docs/phase1_simulation_plan.md#m5--custom-world-with-checkpoints-and-apriltags).
- **Exit-checklist items owned:** none solo. This docset is a hard dependency for / primary contributor to: item 1 (integrative — patrol with image capture at each checkpoint; primary owner 02, this docset provides the world + checkpoints + camera), item 8 (Foxglove renders camera feed — depends on this camera topic; owned 05), item 11 (`CheckpointCapture` used by perception — depends on the AprilTags + camera this docset provides; message owned 04). M5 has no dedicated exit-checklist line; its value is enabling items 1/8/11.
- **Packages / dirs:** `sim/worlds/`, `sim/models/`, `sim/px4_sitl_overrides/` (camera attachment); plus the shared checkpoint-config YAML (location TBD, §7).
- **Lifecycle:** dod.md (this) → prd.md (via /drive) → design.md (via /drive).
