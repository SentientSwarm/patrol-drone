# Perception & Checkpoint Capture — Phase 1 PRD

> **One-liner:** Define the durable `CheckpointCapture` message and a `patrol_perception` node that, at each patrol checkpoint, grabs a camera frame, identifies the checkpoint via AprilTag detection, and emits the `(image, checkpoint_id, pose, timestamp)` tuple on a topic and to disk — the reusable capture pattern every later perception phase builds on.

**Date:** 2026-06-03
**Status:** Draft (rev 2 — self-review pass applied)
**Owner:** Project owner (solo dev) — perception node + `patrol_interfaces` maintainer
**DRI:** jxstanford@wemodulate.energy

**Docset:** docs/phase1/04-perception · **Milestone:** M6 · **Exit-checklist item owned:** 11 (contributes to 1)
**Source of truth:** docs/phase1/04-perception/dod.md (Formal DoD) · docs/phase1_simulation_plan.md §M6

## Overview

Phase 1 needs a perception scaffold: when the patrol reaches a checkpoint, the drone captures the current RGB camera frame, an off-the-shelf AprilTag detector identifies *which* checkpoint it is, and the result is published as one structured ROS 2 message (`patrol_interfaces/msg/CheckpointCapture`) and written to disk as an inspectable artifact. This is deliberately the *thinnest* perception capability — AprilTag identification, not object detection — but it locks down the message contract that four-plus later phases (Phase 3 YOLO/TensorRT, Phase 4 indoor AprilTag relocalization, Phase 6 anomaly detection) and the Phase 1 logging pipeline (docset 05) all consume.

The node itself is simple and easily changed; the `CheckpointCapture` schema is not. Once docset 05 records it into MCAP bags and later phases hang their detectors off the same message, a wrong schema is expensive to migrate — which is exactly why the plan says "get the schema right now and you won't be migrating bag formats two phases from now." That durability is what makes this a contract-defining docset rather than a throwaway node.

## Problem Statement

> **When** a developer runs a patrol in sim (and later flies a real drone) and wants an inspectable, per-checkpoint record of what the drone saw,
> **they struggle with** the absence of any capture-at-a-known-pose primitive — there is no structured message, no on-disk artifact, and no sim/hardware-identical detection path,
> **which means** every later perception phase would re-invent "grab a frame at a known pose and tag it," and the logging pipeline would have no stable schema to record, forcing a bag-format migration two phases later.

Today there is no perception code at all in the repo (`patrol_perception` does not exist; `patrol_interfaces` is a near-empty package shell owned by 01-platform). The only "record of what the drone saw" would be a raw, unlabeled camera stream with no correlation between an image and the checkpoint it depicts. The plan calls this out directly: the capture pipeline is "the scaffold for everything that comes later," and the `CheckpointCapture` message is "worth designing carefully because every later phase consumes it." Why now: M6 is sequenced immediately after the M5 world (3+ AprilTags + RGB camera) lands, and docset 05 (M7) cannot record `/patrol/checkpoint_capture` until this message and topic exist.

## Goals

### Business goals
- Lock down a durable `CheckpointCapture` contract once, so Phases 3/4/6 and docset 05 build on it without a schema migration (satisfies exit-checklist item 11).
- Establish the reusable "capture-and-tag-an-image-at-a-known-pose" pattern that all later perception work reuses unchanged.

### User goals
- After a patrol, the operator can browse a directory of per-checkpoint images plus metadata sidecars without replaying a bag.
- Each captured image is labeled with the checkpoint it belongs to (via AprilTag), not just a timestamp.
- The bag pipeline (05) and Foxglove can subscribe to one well-formed topic to record and visualize captures in real time.

### Non-goals
- Real object detection (YOLO/TensorRT) — deferred to Phase 3; AprilTag identification is sufficient Phase 1 scaffolding.
- Anomaly-detection models — deferred to Phase 6.
- VIO/SLAM-derived pose and AprilTag relocalization — sim provides ground-truth pose; tags are detection targets only in Phase 1.
- Recording the capture into the MCAP bag — that is the consuming side, owned by docset 05.
- Authoring/placing the AprilTag world models and the RGB camera sensor — owned by docset 03 (M5).

> Brief non-goals above are orientation only. The contract-level deferral commitments live in §Out of Scope below.

## Out of Scope

> Items explicitly **not** part of this M6 MVP. Each row is a contract-level commitment that prevents scope creep downstream.

| Item | Status | Rationale | Target | Added |
|------|--------|-----------|--------|-------|
| Real object detection (YOLO / TensorRT) | Deferred | AprilTag identification is sufficient scaffolding for Phase 1; learned detection needs the GPU/TensorRT path | Phase 3 | 2026-06-03 |
| Anomaly-detection models | Deferred | Not a Phase 1 capability; arrives with its own message types | Phase 6 | 2026-06-03 |
| VIO / SLAM-derived capture pose | Deferred | Simulator provides ground-truth pose; capture pose comes from sim/PX4 telemetry | Phase 3 (optional GPS fusion) / Phase 4 (indoor VIO) | 2026-06-03 |
| AprilTag relocalization (tags correcting pose) | Deferred | Tags are detection targets only in sim; relocalization needs real sensors | Phase 4 | 2026-06-03 |
| Writing captures into the rosbag / MCAP pipeline | Out of scope (this docset) | Consuming side owned by 05-logging-replay; this docset only emits the topic + on-disk artifacts | docset 05 (M7) | 2026-06-03 |
| Authoring/placing AprilTag world models + RGB camera sensor | Out of scope (this docset) | Owned by 03-sim-environment | docset 03 (M5) | 2026-06-03 |
| Defining the live camera image topic shape (name/resolution/rate/frame_id) | Out of scope (this docset) | Camera topic is owned by 03; this docset consumes it | docset 03 (M5) | 2026-06-03 |
| Owning/creating the live-frame `sensor_msgs/CompressedImage` topic that the bag records | Out of scope (this docset) | Per the settled image-representation default a CompressedImage live-frame topic carries frames for the bag, but its owner is a 03 (camera-topic shape) / 05 (recorded-image compression) decision — not a topic this docset newly creates (see OQ-8) | docset 03 (M5) / docset 05 (M7) | 2026-06-03 (rev 2) |

## Key Hypotheses

- **H1:** We believe deriving `checkpoint_id` from `apriltag_ros` (mapping detected `tag_id` → `checkpoint_id` via the shared 03 checkpoint config) — rather than hand-rolling detection — will give a sim/hardware-identical detection path that runs unmodified in Phase 4, because the same ROS 2 package and tag family carry to real hardware. *Signal: the Phase 4 indoor relocalization work reuses this node without forking it; in Phase 1, AC-4 passes against the M5 tags.*
- **H2:** We believe carrying the image as an on-disk path string in `CheckpointCapture` — with the live frames travelling on a separate `sensor_msgs/CompressedImage` topic (owner per OQ-8) that the bag records — rather than full pixels by-value, will keep the recorded bag under the plan's "few hundred MB / 5-min mission" budget while still giving Foxglove a renderable image stream. *Signal: docset 05's bag stays within budget (their AC-2) with `/patrol/checkpoint_capture` recorded; capture messages remain small.*
- **H3:** We believe putting the capture/message-construction logic behind a ROS-free seam will let the unit suite validate message construction in well under a second without ROS/Gazebo/PX4, because the same separation worked for the M3 `MissionStateMachine`. *Signal: AC-5 — the unit suite runs green with no ROS spin-up and completes sub-second.*

## Tenets

1. **Get the contract right, keep the node disposable.** When a trade-off pits message-schema clarity against node convenience, favor the schema — the node is cheap to change, the contract is not. *(unless you know better ones)*
2. **Sim path == hardware path.** Prefer the choice that lets this exact detection/capture node run unmodified on real hardware in Phase 4 over a sim-only shortcut.
3. **Off-the-shelf over hand-rolled.** AprilTag detection is a solved problem; use `apriltag_ros` rather than reimplementing it.
4. **Inspectable without infrastructure.** A capture must be reviewable as an on-disk artifact, independent of the bag pipeline.
5. **Coordinate frames are explicit, never implicit.** Capture pose must name its frame; "silent frame mistakes are silent and infuriating."

## Functional Requirements

> No REST endpoints or SDK module paths are in scope (this is a ROS 2 node + message contract), so the Path & SDK conventions subsection is omitted. The interface surface is the `CheckpointCapture` message, the `/patrol/checkpoint_capture` topic, and the on-disk artifact layout.

### P1: Critical (must ship)

#### PCAP-1: Capture a camera frame at each checkpoint
WHEN the patrol reaches a checkpoint, the system SHALL sample the current RGB image from the camera topic and produce exactly one capture for that checkpoint visit.

**Customer scenario:** the operator runs a patrol and, for each checkpoint visited, gets one tagged image to inspect after the flight.

**Pain removed:** without a deterministic capture-at-checkpoint scaffold, every later perception phase would re-invent "grab a frame at a known pose," and there would be no inspectable record of what the drone saw at each checkpoint.

**Acceptance criteria:**
- For a single checkpoint visit with exactly one checkpoint in the field of view, exactly one capture is produced (no duplicate, no dropped capture) — capture cardinality is deterministic per visit (DoD AC-6).
- The sampled frame is the RGB image published by the 03-owned camera topic; the node does not synthesize or hand-author the image.
- The capture trigger derives from the checkpoint-arrival / "looking-at" signal contract owned by 02-mission-control (see OQ-3 for the exact trigger mechanism).

**Trace:** UAC-PCAP-1 (Appendix B)

#### PCAP-2: Identify the checkpoint via AprilTag detection
WHEN a checkpoint AprilTag is in view at capture time, the system SHALL populate `checkpoint_id` from the AprilTag detection produced by `apriltag_ros` (or equivalent), by mapping the detected `tag_id` to a `checkpoint_id` — not by hand-rolling detection.

**Customer scenario:** the operator reviewing captures knows each image is labeled with the checkpoint it belongs to, not just a timestamp.

**Pain removed:** removes manual correlation of images to checkpoints and gives a sim/hardware-identical detection path (the same node runs unmodified on real hardware in Phase 4).

**Acceptance criteria:**
- `checkpoint_id` is populated from an `apriltag_ros` (or equivalent) detection — verifiable by inspecting that the value originates from a detected tag, not a hardcoded constant (DoD AC-4).
- The `tag_id` → `checkpoint_id` mapping is resolved against the single shared checkpoint configuration owned by 03-sim-environment (`sim/config/checkpoints.yaml`, list of `{checkpoint_id, position {x,y,z} ENU, tag_family, tag_id}`); this docset reads that file's `tag_id`↔`checkpoint_id` relation and does not maintain a forked mapping (see OQ-5, pending confirmation).
- AprilTag detection is wired via the existing ROS 2 package; no bespoke detector is implemented (settled constraint, DoD §6).

**Trace:** UAC-PCAP-2 (Appendix B)

#### PCAP-3: Publish a structured `CheckpointCapture` on `/patrol/checkpoint_capture`
WHEN a capture is produced, the system SHALL publish it in real time as one `patrol_interfaces/msg/CheckpointCapture` message on the `/patrol/checkpoint_capture` topic, carrying the same `(image reference, checkpoint_id, pose, timestamp)` data as the on-disk artifact for that checkpoint.

**Customer scenario:** the bag pipeline (05) and Foxglove subscribe to one well-formed topic to record and visualize what was captured.

**Pain removed:** a stable, single message contract prevents bag-format / schema migrations two phases later, which the plan explicitly calls out as the reason to "get the schema right now."

**Acceptance criteria:**
- A `/patrol/checkpoint_capture` message is published in real time at each checkpoint, carrying the same `(image-or-path, checkpoint_id, pose, timestamp)` data as the on-disk artifact for that checkpoint (DoD AC-2).
- The published message is a `patrol_interfaces/msg/CheckpointCapture` (the type defined by PCAP-4), with no forked/duplicate definition.
- The message is consumable by the bag pipeline (docset 05) using the identical compiled type.

**Trace:** UAC-PCAP-3 (Appendix B)

#### PCAP-4: Define and own the `CheckpointCapture` message in `patrol_interfaces`
The system SHALL define `patrol_interfaces/msg/CheckpointCapture` in the shared `patrol_interfaces` package with fields `header` (std_msgs/Header), `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), an image field, and free-form `metadata`, such that the perception node and the bag pipeline use the identical compiled type.

**Customer scenario:** docset 05 consumes the same compiled message type the perception node publishes, with no duplicate or forked definition.

**Pain removed:** satisfies exit-checklist item 11 (one message, used by perception AND the bag pipeline) and prevents interface drift across packages two phases out.

**Acceptance criteria:**
- When `patrol_interfaces` is built, a `patrol_interfaces/msg/CheckpointCapture` message exists with `header`, `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), an image field, and free-form `metadata` (DoD AC-3).
- The image field carries a stored-path reference (`string image_path` to a PNG/JPEG written to disk) rather than full pixels by-value; live frames travel on a separate `sensor_msgs/CompressedImage` topic that the bag records (whose owner is open — see OQ-8) — settled image-representation default per the cross-docset contract, **pending user confirmation** (see OQ-1; jointly owned with docset 05).
- The message lives in `patrol_interfaces` from day one even though that package is otherwise near-empty (settled constraint, DoD §6).
- The same compiled type is published by this docset and recorded by docset 05 (exit-checklist item 11).

**Trace:** UAC-PCAP-4 (Appendix B)

#### PCAP-5: Persist captures to disk
WHEN a capture is produced, the system SHALL write the captured image plus a per-image metadata sidecar into a known output directory.

**Customer scenario:** the operator browses a directory of per-checkpoint images and metadata after a mission without replaying a bag.

**Pain removed:** gives an inspectable artifact independent of the bag, and provides the on-disk file that `image_path` in the message references (see PCAP-4).

**Acceptance criteria:**
- Running a patrol to completion against the M5 world (3+ checkpoint AprilTags) produces a directory of captured images with one per-image metadata file each (DoD AC-1).
- The on-disk image referenced by a capture's `image_path` is the same image whose data the corresponding `/patrol/checkpoint_capture` message describes (consistency between artifact and topic, DoD AC-2).
- The output directory location, filename convention, and sidecar format are defined and align with docset 05's known output location (see OQ-4).

**Trace:** UAC-PCAP-5 (Appendix B)

### P2: Important (should ship)

#### PCAP-6: Emit free-form metadata key-value pairs on each capture
The system SHALL carry free-form capture metadata (e.g., mission id, waypoint index, detection confidence) in the message's `metadata` field and in the on-disk sidecar.

**Customer scenario:** an analyst filters captures by mission/run when triaging a flight.

**Acceptance criteria:**
- The `metadata` field carries key-value pairs and round-trips identically into the on-disk sidecar for the same capture.
- The concrete `metadata` representation (see OQ-2) is a single shape used by both the message and the sidecar.

### P3: Nice to have (stretch)

#### PCAP-7: Detection-confidence passthrough
The system SHALL include the AprilTag detection confidence (when the detector provides one) among the `metadata` key-value pairs, to support later triage and threshold tuning.

## Scope Authority

The FR table above is the **contract** for this PRD. The design document (`docs/phase1/04-perception/design.md` — to be completed via /drive) realizes these FRs as components, sequences, and milestone tasks.

**The design must not introduce surface area beyond this PRD's FR table without a corresponding PRD revision.** If the design proposes a new topic, message field, on-disk artifact type, or external dependency not authorized by an FR, the PRD must be updated first — adding the FR through the PRD's revision flow.

Conversely, **this PRD must not specify implementation detail beyond the FR shape.** The internal node structure, the precise camera-frame sampling discipline, the chosen `metadata` ROS shape, threading, and the directory/sidecar serialization format belong in the design, not the PRD. Where this PRD names a settled default (e.g., `image_path` over by-value pixels), it does so because the choice is a cross-docset *contract* commitment, not because it prescribes node internals.

This discipline keeps the design honest and the PRD lean.

## Success Metrics

| Metric | Baseline (current) | Target | How Measured | Timeline |
|--------|-------------------|--------|--------------|----------|
| Captures produced per checkpoint visit (single tag in FOV) | N/A (new) | Exactly 1 (0 dropped, 0 duplicate) | Count on-disk artifacts + `/patrol/checkpoint_capture` messages vs checkpoints visited in an M5-world patrol | M6 exit |
| On-disk artifacts match checkpoints traversed | N/A (new) | 1 image + 1 sidecar per checkpoint, for all 3+ M5 checkpoints | Directory listing after a full patrol (DoD AC-1) | M6 exit |
| `checkpoint_id` correctly sourced from AprilTag | N/A (new) | 100% of captures populate `checkpoint_id` from a tag detection (no hardcoded values) | Inspect message + detection provenance (DoD AC-4) | M6 exit |
| Message-construction unit suite runtime | N/A (new) | Passes with no ROS/Gazebo/PX4 spin-up, well under 1 s | `pytest` timing in CI (DoD AC-5) | Per-PR (CI) |
| `patrol_interfaces` + `patrol_perception` build in container | N/A (new) | `colcon build` succeeds cleanly in the sim/dev container | CI / container build (DoD AC-7) | Per-PR (CI) |
| `CheckpointCapture` consumed by 05 unchanged | N/A (new) | Bag pipeline records `/patrol/checkpoint_capture` using the identical compiled type, no fork | Cross-check against docset 05's AC-7 | M7 (downstream) |

## Technical Considerations

### Integration points
- **Consumes (03-sim-environment, M5):** the RGB camera `sensor_msgs/Image` topic and the AprilTag world models at known positions. The camera topic's name/resolution/rate/frame_id is owned by 03 (this docset binds to whatever 03 publishes — see OQ flagged in 03's PRD).
- **Consumes (02-mission-control):** the checkpoint-arrival / "looking-at" capture trigger semantic. The exact trigger contract (explicit "capture now" signal vs node-inferred dwell) is a 02↔04 joint decision (OQ-3).
- **Consumes (01-platform / PX4):** `/fmu/out/*` ground-truth pose telemetry and/or the TF tree for the capture pose; container base + `colcon` workspace.
- **Consumes (external):** `apriltag_ros` (or equivalent) ROS 2 package on Jazzy.
- **Consumes (cross-docset, owner open):** the separate `sensor_msgs/CompressedImage` live-frame topic that the bag records under the settled image-representation default. This docset does **not** own/create that topic — its owner is a 03 (camera-topic shape: raw vs compressed) / 05 (recorded-image compression, their OQ-7) decision; flagged as OQ-8 here.
- **Produces (owned):** `patrol_interfaces/msg/CheckpointCapture`, the `/patrol/checkpoint_capture` topic, and the on-disk capture artifacts.

### Data storage
- Per-checkpoint on-disk artifacts: captured images (PNG/JPEG) plus per-image metadata sidecars, in a known output directory whose location aligns with docset 05's output convention (OQ-4). No database; files only at Phase 1 scale.
- Captured pose comes from ground-truth sim/PX4 telemetry; no VIO/SLAM-derived pose is stored (settled constraint).

### Scalability
- Phase 1 scale is a single drone visiting a handful of checkpoints per ~5-minute mission; capture is event-driven (one per checkpoint visit), not a streaming workload. The bag-size budget (under a few hundred MB / 5-min mission, owned by 05) is the binding constraint, and is the reason `CheckpointCapture` carries an image path rather than pixels by-value.

### Rabbit holes
- **The image-representation decision (`image_path` vs by-value pixels).** Looks like a one-line message field but determines bag size (05), Foxglove rendering (exit item 8), and the on-disk persistence design. Contain by adopting the settled default (path reference in the message + a separate `CompressedImage` live-frame topic the bag records, whose owner is settled per OQ-8) and resolving jointly with 05 before the message is finalized; do not let it reopen mid-implementation.
- **`checkpoint_id` namespace / mapping source.** Whether `checkpoint_id` is the raw AprilTag id or a semantic name, and where the `tag_id`↔`checkpoint_id` map lives, must reconcile with 03's `sim/config/checkpoints.yaml` so one schema maps `checkpoint_id ↔ world position ↔ tag id`. Contain by binding to 03's single shared config; do not introduce a second mapping store in `patrol_perception`.
- **Camera-frame sampling discipline / coordinate frame.** "Latest frame on trigger" vs time-synced-to-pose, and which TF frame the pose is expressed in, can silently produce a pose that doesn't match the image. Contain by fixing the sampling rule and the pose frame explicitly in the design (tenet 5).
- **External dependency reality check on `apriltag_ros` for Jazzy.** The plan assumes a good ROS 2 AprilTag package exists; confirm an `apriltag_ros`-equivalent is available/buildable on ROS 2 Jazzy in-container before committing the detection path (verification target, see OQ-6).

### Potential challenges
- **Schema durability vs YAGNI.** The message is consumed by 4+ later phases, so under-specifying it forces a migration; over-specifying it adds fields no Phase 1 consumer reads. Mitigation: ship exactly the DoD-mandated field set (header / checkpoint_id / pose / image / metadata), and use the free-form `metadata` field as the extension point for later phases rather than adding typed fields now.
- **Trigger coupling with 02.** If the capture trigger contract with 02 is wrong, captures fire at the wrong time or miss checkpoints. Mitigation: settle OQ-3 jointly with 02 during design, before wiring the node.

## Cross-Service Impact

### Affected services (docsets)

| Service (docset) | Impact | Changes required |
|---------|--------|-----------------|
| 03-sim-environment | This docset *consumes* 03's RGB camera topic and AprilTag models, and reads 03's shared checkpoint config for the `tag_id`↔`checkpoint_id` mapping. The CompressedImage-vs-raw shape of the camera/live-frame topic is part of 03's camera-topic-shape decision (OQ-8) | None to 03's deliverables; this docset must bind to 03's `sim/config/checkpoints.yaml` schema and camera topic name (joint reconciliation of the checkpoint-config schema); confirm with 03 whether the live-frame topic the bag records is raw or `CompressedImage` |
| 02-mission-control | This docset consumes 02's checkpoint-arrival / "looking-at" capture-trigger semantic | Joint agreement on the trigger contract (OQ-3): explicit "capture now" signal vs node-inferred dwell |
| 05-logging-replay | This docset *produces* the `CheckpointCapture` message + topic that 05 records into the MCAP bag; 05 is the named second consumer (exit item 11). The recorded-image compression (raw vs `CompressedImage`) is part of 05's bag-schema decision (their OQ-7) | 05 binds to the identical compiled `patrol_interfaces/msg/CheckpointCapture` type and records `/patrol/checkpoint_capture`; the image-representation decision (OQ-1) and the live-frame-topic shape/owner (OQ-8) are jointly owned with 04/03 |
| 01-platform | Provides `patrol_interfaces` package shell, container, `colcon` workspace, `/fmu/out/*` pose | None; this docset fills the `patrol_interfaces` package contents (message) and adds `patrol_perception` |

### Interface changes
- **New shared message:** `patrol_interfaces/msg/CheckpointCapture` — fields `header`, `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), image field (settled default: `string image_path`), `metadata` (free-form). This is the contract item; field types/names are the binding surface.
- **New topic (owned):** `/patrol/checkpoint_capture` (type `patrol_interfaces/msg/CheckpointCapture`).
- **Live-frame topic (owner open, NOT owned by this docset):** under the settled image-representation default a separate `sensor_msgs/CompressedImage` topic carries live frames and is what the bag records. Its owner is a 03 camera-topic-shape / 05 recorded-image-compression decision (OQ-8) — this docset references and consumes it, it does not create it.
- **New on-disk artifact contract:** directory of images + per-image metadata sidecars (layout/format to be fixed in design, aligned with 05's output location).

### Deployment coordination
- Build/dependency order is `03 → 04 → 05` (mirrors M5 → M6 → M7). The camera topic + AprilTag world (03) must exist before this node can be exercised end-to-end; the `CheckpointCapture` message (04) must exist before 05 can record it.
- The message contract should be settled (especially OQ-1 image representation, OQ-5 checkpoint mapping, and OQ-8 live-frame-topic owner) *before* docset 05 begins recording against it, to avoid a re-record/migration.

### Testing implications
- **Unit (this docset, per-PR):** message-construction tests run without ROS/Gazebo/PX4, sub-second (DoD AC-5). These are the contract-shape tests.
- **Container build (this docset, per-PR):** `colcon build` of `patrol_interfaces` + `patrol_perception` in-container (DoD AC-7).
- **Integration (cross-docset, downstream):** the end-to-end patrol (02) over the M5 world (03) producing on-disk artifacts (DoD AC-1) and the live topic (DoD AC-2); docset 05's replay/record test consuming the same message type (their AC-7). This docset contributes the capture half of integrative exit-checklist item 1.

## Milestones

This docset is milestone M6 (single milestone). The phases below are the walking-skeleton decomposition within M6 — each ends in an observable demo.

### Phase A: Message contract first (thinnest end-to-end)
- Define `patrol_interfaces/msg/CheckpointCapture` (PCAP-4) with the settled field set + `image_path` image representation.
- Write message-construction unit tests (PCAP-3/PCAP-4 shape) that run with no ROS/Gazebo/PX4.
- **Validation:** `colcon build` produces the compiled `CheckpointCapture` type in-container (DoD AC-3, AC-7); the unit suite passes sub-second (DoD AC-5). A reviewer can build the package and see the message type and green tests.

### Phase B: Capture + detect + publish
- Implement `patrol_perception`: subscribe to 03's camera topic, wire `apriltag_ros` for `checkpoint_id` (PCAP-2), trigger one capture per checkpoint visit (PCAP-1), publish `/patrol/checkpoint_capture` (PCAP-3).
- **Validation:** in a running SITL patrol over the M5 world, each checkpoint produces exactly one `/patrol/checkpoint_capture` message with a tag-sourced `checkpoint_id` (DoD AC-2, AC-4, AC-6). A reviewer can `ros2 topic echo` the topic during a patrol and see one well-formed message per checkpoint.

### Phase C: Persist + metadata
- Write per-checkpoint images + metadata sidecars to the known output directory (PCAP-5); populate the `metadata` field/sidecar (PCAP-6).
- **Validation:** a completed patrol over the M5 world leaves a directory with one image + one sidecar per checkpoint, consistent with the topic data (DoD AC-1). A reviewer can browse the output directory after a mission.

## Open Questions

| # | Question | Status | Decision target | Rationale (why open / what would resolve it) |
|---|----------|--------|-----------------|----------------------------------------------|
| OQ-1 | Image representation in `CheckpointCapture`: `string image_path` (path to a PNG/JPEG on disk) with live frames on a separate `sensor_msgs/CompressedImage` topic that the bag records — vs `sensor_msgs/Image` by-value pixels. | Provisional (settled default pending user confirmation) | M6 design phase, jointly with docset 05 | Settled default is **path + separate CompressedImage live topic** (DEFERRED to human: the by-value-vs-path call is a cross-docset contract with 05's bag-size budget per M7's "compressed image to keep bag size manageable"). Resolving it finalizes PCAP-4's image field and the on-disk persistence design. Must match docset 05 §7. The *owner* of the live CompressedImage topic is tracked separately in OQ-8. |
| OQ-2 | `metadata` representation: `diagnostic_msgs/KeyValue[]` vs a JSON string vs a custom KV array. | Open | M6 design phase | Plan says "free-form"; the chosen shape is a cross-docset contract that 05 records and analysis reads. Resolving fixes PCAP-6's field shape and the sidecar serialization. |
| OQ-3 | Capture trigger mechanism: mission (02) publishes an explicit "capture now" signal vs perception node infers a checkpoint visit from AprilTag-in-view + dwell. | Open | M6 design phase, jointly with 02-mission-control | Determines the 02↔04 interface; the plan describes the behavior, not the trigger contract. Resolving fixes how PCAP-1 fires. |
| OQ-4 | On-disk layout + sidecar format: directory structure, filename convention, sidecar as JSON vs YAML, and relation to the bag output dir. | Open | M6 design phase | Plan says "directory of captured images with per-image metadata files" without fixing the format; should align with 05's known output location. Resolving fixes PCAP-5's artifact contract. |
| OQ-5 | `checkpoint_id` namespace / mapping: raw AprilTag id vs a semantic checkpoint name, and where the `tag_id`↔`checkpoint_id` mapping lives. | Provisional (settled default pending user confirmation) | M6 design phase, jointly with 03-sim-environment | Settled default: 04 maps detected `tag_id` → `checkpoint_id` using **03's single shared `sim/config/checkpoints.yaml`** (`{checkpoint_id, position {x,y,z} ENU, tag_family, tag_id}`); 04 does not fork the mapping (DEFERRED to human — cross-docset schema with 03, consumed also by 02). Must reconcile with 03 §7. |
| OQ-6 | `apriltag_ros` (or equivalent) availability/buildability on ROS 2 Jazzy in-container. | Needs Input | Before Phase B (M6 design phase) | Plan assumes a good ROS 2 AprilTag package exists; this is an external-upstream dependency. Clarification action: confirm an installable/buildable `apriltag_ros`-equivalent on Jazzy in the sim/dev container before committing the detection path (settled constraint forbids hand-rolling). |
| OQ-7 | Camera-frame sampling discipline: latest-frame-on-trigger vs time-synced-to-pose, and which TF frame the capture pose is expressed in. | Open | M6 design phase | Plan flags coordinate-frame mistakes as "silent and infuriating"; capture pose must be unambiguous (tenet 5). Resolving fixes how PCAP-1 binds image to pose. |
| OQ-8 | Owner of the live-frame `sensor_msgs/CompressedImage` topic the bag records: is it 03's camera topic published as `CompressedImage` (camera-topic shape), a 05 recorded-image-compression step, or a republisher? This docset does **not** own/create it. | Open (cross-docset, owner unassigned) | M6 design phase, jointly with 03 + 05 | The settled image-representation default (OQ-1) presumes a CompressedImage live-frame topic exists and the bag records it, but neither this docset's DoD §5 nor the Phase 1 README ownership matrix assigns such a topic to 04 (03 owns the camera `sensor_msgs/Image` topic; 05 owns recorded-topic/compression per their OQ-7). Resolving it names the topic's owner so 04 binds to it instead of inventing it — prevents an unowned interface. |

## Appendix B: User Acceptance Criteria

> Every P1 FR has a corresponding UAC in Given/When/Then form. UAC IDs match the FR ID they cover.

### UAC-PCAP-1: Capture a camera frame at each checkpoint
**GIVEN** the M5 custom world with 3+ checkpoint AprilTags and a running patrol mission, with exactly one checkpoint in the camera field of view at the visit
**WHEN** the patrol reaches that checkpoint and the capture trigger fires
**THEN** exactly one capture is produced for that visit (no duplicate, no dropped capture), sampled from the 03-owned RGB camera topic.

### UAC-PCAP-2: Identify the checkpoint via AprilTag detection
**GIVEN** a checkpoint AprilTag in view at capture time and the shared 03 checkpoint config providing the `tag_id`↔`checkpoint_id` mapping
**WHEN** the node captures
**THEN** `checkpoint_id` is populated from an `apriltag_ros` (or equivalent) detection — its value traces to a detected `tag_id` mapped through the shared config, not to a hardcoded constant.

### UAC-PCAP-3: Publish a structured `CheckpointCapture` on `/patrol/checkpoint_capture`
**GIVEN** a running patrol producing captures
**WHEN** each checkpoint is reached
**THEN** a `/patrol/checkpoint_capture` message of type `patrol_interfaces/msg/CheckpointCapture` is published in real time, carrying the same `(image-or-path, checkpoint_id, pose, timestamp)` data as the on-disk artifact for that checkpoint, consumable by docset 05 via the identical compiled type.

### UAC-PCAP-4: Define and own the `CheckpointCapture` message in `patrol_interfaces`
**GIVEN** the `patrol_interfaces` package
**WHEN** it is built with `colcon build`
**THEN** a `patrol_interfaces/msg/CheckpointCapture` message exists with `header` (std_msgs/Header), `checkpoint_id` (string), `pose` (geometry_msgs/PoseStamped), an image field (settled default: `string image_path`), and free-form `metadata`; AND the same compiled type is published by the perception node (this docset) and recorded by the bag pipeline (docset 05).

### UAC-PCAP-5: Persist captures to disk
**GIVEN** the M5 world with 3+ checkpoint AprilTags and a patrol mission
**WHEN** the mission runs to completion
**THEN** the known output directory contains one captured image and one per-image metadata sidecar per checkpoint, and each image referenced by a capture's `image_path` matches the data described in the corresponding `/patrol/checkpoint_capture` message.

## Quality Gate Notes

- **Deferred to human (cross-docset contract defaults applied, pending confirmation):** OQ-1 (image representation = `string image_path` + a separate `sensor_msgs/CompressedImage` live-frame topic the bag records, jointly owned with docset 05) and OQ-5 (`checkpoint_id` from detected `tag_id` mapped via 03's shared `sim/config/checkpoints.yaml`, jointly owned with 03). Both are recorded as Provisional with the settled default applied, not silently invented.
- **Residual cross-docset finding (rev 2):** OQ-8 records that the live-frame `CompressedImage` topic the bag records has no assigned owner in this docset's DoD or the Phase 1 README ownership matrix. The settled image default presumes the topic exists; rev 2 reclassifies it from "owned by 04" to a cross-docset item (03 camera-topic shape / 05 recorded-image compression) and flags it for the human's combined review rather than letting 04 claim an unowned interface.
- **External-dependency verification target:** OQ-6 (`apriltag_ros` on Jazzy in-container) is flagged Needs Input with an explicit clarification action, because the settled constraint forbids hand-rolling detection and the plan's "the ROS 2 package is good" is an unverified upstream claim.
- **UAC bodies** are completed (not stubs) for all five P1 FRs.
- **No API surface:** Path & SDK conventions subsection intentionally omitted (ROS 2 node + message, no REST/SDK paths).
- **PCAP-7 (P3)** has no UAC by design (UAC pairing is a P1 discipline).
- **`apriltag_ros` "or equivalent" loophole:** retained verbatim from DoD §6 / AC-4 as a deliberate, OQ-6-tracked hedge (not a D13 smell to remove); the settled constraint forbids hand-rolling, and OQ-6 is the verification that resolves the hedge before Phase B.
