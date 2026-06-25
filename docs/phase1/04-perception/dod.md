# Definition of Done — Perception & Checkpoint Capture

**Phase 1 docset:** 4 of 5 · **Milestones:** M6
**Lifecycle status:** DoD ✅ · PRD ✅ · Design ✅
**Source:** docs/phase1_simulation_plan.md — M6 ("Perception scaffolding, image capture at checkpoint"); cross-cutting: "Test strategy", "What's explicitly NOT in Phase 1", "Phase 1 exit checklist"
**Stakeholders:** Project owner (solo dev) — operator/maintainer of the perception node and the `patrol_interfaces` contract; downstream — Phase 3 (YOLO/TensorRT), Phase 4 (indoor AprilTag relocalization), Phase 6 (anomaly detection), and docset 05-logging-replay (bag pipeline consumer of `CheckpointCapture`); reviewers — PR reviewers on `main` (working-in-sim gate)
**Depends on:** 03-sim-environment (checkpoint AprilTags placed at YAML-configured world positions + drone RGB camera publishing a ROS 2 image topic); 02-mission-control (mission signals a checkpoint arrival / "looking at" event); 01-platform (`/fmu/out/*` pose telemetry, container + `colcon build`)
**Consumed by:** 05-logging-replay (records `/patrol/checkpoint_capture` and the on-disk capture artifacts into the MCAP bag); 02-mission-control (mission may use checkpoint identification as a per-waypoint completion signal); later phases (Phase 3/4/6 perception all hang off this capture pattern and this message)

## 1. Intent
Deliver the Phase 1 perception scaffold: at each patrol checkpoint the drone captures the current camera frame, an AprilTag detector identifies which checkpoint it is, and the `(image, checkpoint_id, pose, timestamp)` tuple is published on a structured ROS 2 topic and written to disk. This is the reusable "capture and tag an image at a known pose" pattern every later perception phase builds on, and it locks down the `CheckpointCapture` message that every downstream consumer depends on.

## 2. Scope
**In scope:**
- A `patrol_perception` ROS 2 node that captures a camera frame at each checkpoint and publishes a structured capture message.
- AprilTag detection wired in via the existing `apriltag_ros` (or equivalent) ROS 2 package — used to derive `checkpoint_id`, not rolled by hand.
- The `CheckpointCapture` message defined in `patrol_interfaces` (header, checkpoint_id, pose, image-or-path, metadata).
- Per-checkpoint on-disk artifacts: captured images plus per-image metadata sidecars in a directory.
- The live `/patrol/checkpoint_capture` topic carrying the same data in real time.
- Unit tests covering the capture node's message construction.

**Out of scope (explicit deferrals — item · rationale · target):**
- Real object detection (YOLO / TensorRT) · AprilTag identification is sufficient scaffolding for Phase 1 · Phase 3
- Anomaly-detection models · not a Phase 1 capability · Phase 6
- VIO / SLAM-derived pose · simulator provides ground-truth pose, so capture pose comes from sim/PX4 telemetry · Phase 3 (optional GPS fusion) / Phase 4 (indoor VIO-only)
- AprilTag relocalization (using tags to correct pose) · tags are detection targets only in sim; relocalization needs real sensors · Phase 4
- Writing the capture into the rosbag / MCAP pipeline · that is the consuming side, owned by 05 · this phase only emits the topic + on-disk artifacts (05 records them)
- Authoring/placing the AprilTag world models and the RGB camera sensor · owned by 03-sim-environment · M5

## 3. Capabilities (must-do — seeds the PRD's functional requirements)
1. **(P1) Capture a camera frame at each checkpoint.** When the patrol reaches a checkpoint, the node samples the current RGB image topic and produces one capture per checkpoint visit.
   - *Customer scenario:* operator runs a patrol and gets one tagged image per checkpoint to inspect after the flight.
   - *Pain removed:* without a deterministic capture-at-checkpoint scaffold, every later perception phase (YOLO, anomaly) would have to re-invent "grab a frame at a known pose," and there would be no inspectable record of what the drone saw.
2. **(P1) Identify which checkpoint via AprilTag detection.** The node uses `apriltag_ros` (or equivalent) detections to populate `checkpoint_id` for the capture.
   - *Customer scenario:* operator reviewing captures knows each image is labeled with the checkpoint it belongs to, not just a timestamp.
   - *Pain removed:* removes manual correlation of images to checkpoints and gives a sim/hardware-identical detection path (the same node runs unmodified on real hardware in Phase 4).
3. **(P1) Publish a structured `CheckpointCapture` on `/patrol/checkpoint_capture`.** Each capture is emitted as one `patrol_interfaces/msg/CheckpointCapture` carrying header, checkpoint_id, pose, image-or-path, and metadata.
   - *Customer scenario:* the bag pipeline (05) and Foxglove subscribe to one well-formed topic to record and visualize what was captured.
   - *Pain removed:* a stable message contract now prevents bag-format / schema migrations two phases later, which the plan explicitly calls out as the reason to "get the schema right now."
4. **(P1) Persist captures to disk.** The node writes captured images plus a per-image metadata sidecar into a known output directory.
   - *Customer scenario:* operator browses a directory of per-checkpoint images and metadata after a mission without replaying a bag.
   - *Pain removed:* gives an inspectable artifact independent of the bag, and seeds the image-handling decision (by-value vs path) the message must support.
5. **(P1) Define and own the `CheckpointCapture` message in `patrol_interfaces`.** The message is defined in the shared interfaces package so the perception node and the bag pipeline use the identical type.
   - *Customer scenario:* docset 05 consumes the same compiled message type the perception node publishes, with no duplicate/forked definition.
   - *Pain removed:* satisfies exit-checklist item 11 (one message, used by perception AND the bag pipeline) and prevents interface drift across packages.
6. **(P2) Emit free-form metadata key-value pairs on each capture.** Capture metadata (e.g., mission id, waypoint index, detection confidence) is carried in the message's metadata field and in the sidecar.
   - *Customer scenario:* analyst filters captures by mission/run when triaging a flight.

## 4. Acceptance criteria / Definition of Done (falsifiable — seeds the PRD's UACs)
- [ ] **AC-1 (M6 exit; supports checklist item 1):** GIVEN the M5 custom world with 3+ checkpoint AprilTags and a patrol mission, WHEN the mission runs to completion, THEN it produces a directory of captured images with one per-image metadata file each.
- [ ] **AC-2 (M6 exit):** GIVEN a running patrol, WHEN each checkpoint is reached, THEN a `/patrol/checkpoint_capture` message is published in real time carrying the same `(image-or-path, checkpoint_id, pose, timestamp)` data as the on-disk artifact for that checkpoint.
- [ ] **AC-3 (checklist item 11):** GIVEN the `patrol_interfaces` package, WHEN it is built, THEN a `patrol_interfaces/msg/CheckpointCapture` message exists with `header`, `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), an image field (sensor_msgs/Image or a stored-path reference), and free-form `metadata`; AND it is published by the perception node (this docset) and consumable by the bag pipeline (docset 05).
- [ ] **AC-4 (M6 exit):** GIVEN a checkpoint AprilTag in view, WHEN the node captures, THEN `checkpoint_id` is populated from the AprilTag detection (via `apriltag_ros` or equivalent), not hand-rolled.
- [ ] **AC-5 (M6 exit; Test strategy — unit):** GIVEN the capture node's message-construction logic, WHEN the unit test suite runs, THEN message-construction tests pass without spinning up ROS, Gazebo, or PX4, and complete in well under a second.
- [ ] **AC-6:** GIVEN a checkpoint visit, WHEN exactly one checkpoint is in the field of view, THEN exactly one capture is produced for that checkpoint (no duplicate/dropped captures for a single visit) — capture cardinality is deterministic per visit.
- [ ] **AC-7 (container parity; supports checklist item 9):** GIVEN the sim/dev container, WHEN `colcon build` runs, THEN `patrol_interfaces` and `patrol_perception` build cleanly inside the container.

## 5. Interfaces
**Owns (contracts this docset defines that others depend on):**
- `patrol_interfaces/msg/CheckpointCapture` — fields: `header` (std_msgs/Header), `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), image field (sensor_msgs/Image OR a stored-path string — see §7), `metadata` (free-form key-value pairs). Exact field types/names are the contract; the by-value-vs-path image decision is open (§7).
- Topic `/patrol/checkpoint_capture` (type `patrol_interfaces/msg/CheckpointCapture`) — the real-time capture stream.
- On-disk capture artifacts: a directory of captured images plus per-image metadata sidecar files in a known output location (directory layout / sidecar format — see §7).
- ROS 2 package `patrol_perception` (the capture/detection node) and `patrol_interfaces` (the message package).

**Consumes (from other docsets / PX4):**
- RGB camera image topic published by the drone — from 03-sim-environment (M5).
- Checkpoint AprilTag world models at known positions — from 03-sim-environment (M5).
- Checkpoint-arrival / "looking-at" signal or per-checkpoint trigger — from 02-mission-control.
- Pose telemetry (ground-truth in sim) — from `/fmu/out/*` (PX4 via 01-platform) and/or the TF tree.
- `apriltag_ros` (or equivalent) ROS 2 package — external dependency.
- Container base + `colcon` workspace — from 01-platform.

## 6. Settled constraints (do NOT relitigate — cite the source)
- **AprilTag detection uses `apriltag_ros` or equivalent — do not roll your own.** (plan M6: "It's a solved problem and the ROS 2 package is good.")
- **`CheckpointCapture` lives in `patrol_interfaces` from day one**, even though that package may otherwise be near-empty. (plan "Repo structure" notes; M6.)
- **The message must carry header / checkpoint_id / pose / image / metadata.** The field set is fixed; only the image representation is open. (plan M6 message sketch.)
- **uXRCE-DDS native (not MAVROS); pose telemetry arrives as native ROS 2 `/fmu/out/*` topics.** (plan "Target stack"; ADR-0001 Consequences/Neutral.)
- **Ground-truth pose from the simulator — no VIO/SLAM in Phase 1.** (plan "What's explicitly NOT in Phase 1.")
- **AprilTags in sim are detection targets, not relocalization inputs** in Phase 1; the same detection node must run unmodified on hardware later. (plan M5/M6.)
- **Python 3.12, ROS 2 Jazzy, Ubuntu 24.04, Gazebo Harmonic.** (plan "Target stack"; ADR-0001.)
- **Unit tests mock the PX4 / sensor interface; do not mock the simulator.** Message construction and capture logic are tested without ROS/Gazebo/PX4. (plan "Test strategy.")

## 7. Open decisions (handed to /drive — each: question · decision target · why open)
- **Image by-value vs by-path in `CheckpointCapture`** · resolve in PRD/Design before message is finalized · plan explicitly hedges ("sensor_msgs/Image, or a reference to a stored path for large images"); affects bag size (05) and Foxglove rendering (item 8). Must be settled jointly with 05 §7 (bag-size impact).
- **`metadata` representation** (e.g., `diagnostic_msgs/KeyValue[]` vs a JSON string vs a custom KV array) · Design · plan says "free-form"; the chosen shape is a cross-docset contract that 05 records and analysis reads.
- **Capture trigger mechanism** (mission publishes an explicit "capture now" signal vs perception node infers a checkpoint visit from AprilTag-in-view + dwell) · Design, jointly with 02 · determines the 02↔04 interface; plan describes the behavior, not the trigger contract.
- **On-disk layout + sidecar format** (directory structure, filename convention, sidecar as JSON/YAML, relation to the bag output dir) · Design · plan says "directory of captured images with per-image metadata files" without fixing the format; should align with 05's known output location.
- **`checkpoint_id` namespace / mapping** (raw AprilTag id vs a semantic checkpoint name, and where the tag-id→checkpoint mapping lives) · Design, jointly with 03 · plan says "string, from AprilTag" but the mapping source (config in 03's world YAML vs 04) is unspecified. Must reconcile with 03 §7 (checkpoint-config schema) so one schema maps `checkpoint_id ↔ world position ↔ tag id`.
- **Camera-frame sampling discipline** (latest frame on trigger vs time-synced to pose, and which TF frame the pose is expressed in) · Design · plan flags coordinate-frame mistakes as "silent and infuriating" (M4); capture pose must be unambiguous.

## 8. Assessment signals (so prd-engine right-sizes the PRD)
| Dimension | Value | One-line justification |
|---|---|---|
| Nature | greenfield | New `patrol_perception` node and new `CheckpointCapture` message, no prior code. |
| Complexity | moderate | A focused node plus one well-scoped message; AprilTag detection is off-the-shelf. |
| Urgency | standard | Sequenced in the M1–M8 build order; not emergency, not pure exploration. |
| Risk | medium | The message is consumed by 4+ later phases — a wrong schema is costly to undo even though the node itself is simple. |
| Reversibility | costly-to-reverse | The node is easily changed, but the `CheckpointCapture` contract, once consumed by 05 and later phases, is expensive to migrate (the plan's explicit warning). |
| Scope | cross-service | Spans `patrol_perception` + `patrol_interfaces` and forms a contract consumed by 02, 05, and later phases. |
| Audience | developer | Solo dev / maintainer; downstream consumers are other docsets and phases. |
**Suggested PRD tier:** Standard (the raw Moderate×Low-Medium cell in prd-engine's matrix is Lightweight, but two of the engine's own modifiers bump it to Standard: (1) the conflict rule — "risk always wins, bump up to the next tier" — applies because Reversibility=costly-to-reverse modifies the `CheckpointCapture` contract's effective risk upward once 05 and Phases 3/4/6 consume it; and (2) Scope=cross-service triggers the conditional Cross-Service-Impact section. The durable, multi-phase message contract is what justifies requirement IDs, FR↔UAC pairing, and an explicit Out-of-Scope over Lightweight; it does not reach Complex/High, so it holds at Standard rather than Comprehensive).

## 9. Traceability
- **Milestones:** M6 — perception scaffolding, image capture at checkpoint, `CheckpointCapture` defined and published (docs/phase1_simulation_plan.md#m6-perception-scaffolding-image-capture-at-checkpoint)
- **Exit-checklist items owned:** 11 (primary — `CheckpointCapture` defined in `patrol_interfaces`; this docset owns the message and its emission; docset 05 consumes it in the bag pipeline). Contributes to: 1 (image capture at each checkpoint, integrative — primary 02).
- **Packages / dirs:** ros2_ws/src/patrol_perception, ros2_ws/src/patrol_interfaces; consumes sim/models + sim/worlds (03) and `/fmu/out/*` (01)
- **Lifecycle:** dod.md (this) → prd.md (via /drive) → design.md (via /drive)
