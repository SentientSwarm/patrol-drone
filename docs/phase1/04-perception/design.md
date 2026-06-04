# Perception & Checkpoint Capture — System Design Working Document

**Status:** Draft
**Version:** 0.2.0
**Date:** 2026-06-03
**Projects:** Patrol-Drone Phase 1 (pre-hardware simulation) — Perception & Checkpoint Capture (docset 04)
**Authors:** Project owner (solo dev) — jxstanford@wemodulate.energy
**Requirements source (sole):** `docs/phase1/04-perception/prd.md` (Perception & Checkpoint Capture — Phase 1 PRD, rev 2)
**Milestone:** M6 · **Exit-checklist item owned:** 11 (contributes to 1)

---

## 1. Introduction

This design realizes the Phase 1 perception scaffold defined by the `04-perception` PRD: a `patrol_perception` ROS 2 node that, when the patrol arrives and dwells at a checkpoint, samples the RGB camera frame, identifies *which* checkpoint it is via an off-the-shelf AprilTag detector, and emits the result both as one structured `patrol_interfaces/msg/CheckpointCapture` message on `/patrol/checkpoint_capture` and as an on-disk artifact (image + metadata sidecar). It is deliberately the thinnest perception capability — AprilTag identification, not object detection — but it locks down the durable `CheckpointCapture` contract that four-plus later phases (Phase 3 YOLO/TensorRT, Phase 4 indoor relocalization, Phase 6 anomaly detection) and the Phase 1 logging pipeline (docset 05) all consume.

The design's central tension, inherited from the PRD's tenets, is **"get the contract right, keep the node disposable."** The node internals (sampling discipline, threading, file serialization) are cheap to change; the `CheckpointCapture` schema and the `/patrol/checkpoint_capture` topic are not, because once docset 05 records them into MCAP bags and later phases hang detectors off the same message, a wrong schema forces a bag-format migration. The design therefore spends most of its rigor on the message contract, the `tag_id → checkpoint_id` mapping source, the image representation, and the capture-pose frame — and keeps the node a simple event-driven structure behind a ROS-free seam so message-construction logic is unit-testable in well under a second.

The work is a single ROS 2 workspace concern spanning two packages — `patrol_interfaces` (the message, a contents-fill of 01's package shell) and `patrol_perception` (the node) — and forms a contract consumed by docsets 02 (trigger), 03 (camera + checkpoint config), and 05 (bag). There is no UI, no REST/SDK surface, no database, and no auth boundary; the design is right-sized to that reality.

### Source Requirements (PRD)

| # | Requirement source | Priority | Realized by |
|---|--------------------|----------|-------------|
| 1 | PCAP-1 — Capture a camera frame at each checkpoint | P1 | CaptureCoordinator, FrameSampler |
| 2 | PCAP-2 — Identify the checkpoint via AprilTag detection | P1 | CheckpointResolver, apriltag_ros (external) |
| 3 | PCAP-3 — Publish `CheckpointCapture` on `/patrol/checkpoint_capture` | P1 | CapturePublisher, CheckpointCaptureBuilder |
| 4 | PCAP-4 — Define and own the `CheckpointCapture` message | P1 | `patrol_interfaces` (CheckpointCapture.msg) |
| 5 | PCAP-5 — Persist captures to disk | P1 | CaptureWriter |
| 6 | PCAP-6 — Emit free-form metadata key-value pairs | P2 | CheckpointCaptureBuilder, CaptureWriter |
| 7 | PCAP-7 — Detection-confidence passthrough | P3 | CheckpointResolver (metadata enrichment) |

### Related docsets

| Docset | Relevance |
|--------|-----------|
| 03-sim-environment (M5) | Produces the RGB camera `sensor_msgs/Image` topic and the AprilTag world models; owns the shared `sim/config/checkpoints.yaml` this design reads for the `tag_id ↔ checkpoint_id` mapping. |
| 02-mission-control (M3/M4) | Produces the checkpoint-arrival / "looking-at" capture-trigger semantic this design keys capture off of. |
| 05-logging-replay (M7) | Named second consumer of `CheckpointCapture`; records `/patrol/checkpoint_capture` into the MCAP bag. Joint owner of the image-representation decision. |
| 01-platform (M1/M2) | Provides the `patrol_interfaces` package shell, the `ros2_ws` + `colcon` build, the container, and `/fmu/out/*` pose telemetry + TF. |

---

## 2. Open Questions & Assumptions

These mirror the PRD's Open Questions, carried into the design with the settled contract defaults applied (per the auto-pilot policy, cross-docset contracts use the settled defaults and are flagged "confirmed at combined review (2026-06-03)"; genuinely unresolved items are scheduled within the M6 design phase as the executor's responsibility).

| # | Item | Source | Status | Decision target / rationale |
|---|------|--------|--------|------------------------------|
| OQ-1 | Image representation in `CheckpointCapture`: `string image_path` (path to PNG/JPEG on disk) + live frames on a separate `sensor_msgs/CompressedImage` topic the bag records — vs `sensor_msgs/Image` by-value. | PRD OQ-1 (P1 FR PCAP-4) | **Provisional — settled default applied: path + separate CompressedImage live topic** | DEFERRED to human (cross-docset contract with 05's bag-size budget). Design adopts `string image_path` (§4.2.7, §4.6). Resolving finalizes PCAP-4's image field. Must match docset 05 §7 (their DoD §7 names the by-value-vs-path call as shared with 04, to be settled jointly). |
| OQ-2 | `metadata` representation: `diagnostic_msgs/KeyValue[]` vs JSON string vs custom KV array. | PRD OQ-2 (PCAP-6) | **Resolved (design decision) → `diagnostic_msgs/KeyValue[]`** | §4.2.4 (ADR-C). Standard ROS type, Foxglove-renderable, round-trips to the sidecar as a JSON object. Recorded by 05; analysis reads it. Promotable to user review but does not block. |
| OQ-3 | Capture trigger mechanism: 02 publishes an explicit "capture now" signal vs 04 infers a checkpoint visit from AprilTag-in-view + dwell. | PRD OQ-3 (PCAP-1) | **Resolved (design decision) → explicit trigger from 02, with tag-in-view as the gate** | §4.2.2, ADR-A. Design keys capture off 02's `/patrol/current_waypoint` arrival/dwell semantic (a 02↔04 joint contract; `/patrol/current_waypoint` is a 02-owned topic per the Phase 1 README matrix); 04 does NOT infer the visit unilaterally. Flagged "pending 02 confirmation of the exact trigger topic/field." |
| OQ-4 | On-disk layout + sidecar format: directory structure, filename convention, sidecar JSON vs YAML, relation to bag output dir. | PRD OQ-4 (PCAP-5) | **Resolved (design decision) → flat run-scoped dir, `NNN_<checkpoint_id>.{png,json}`, JSON sidecar** | §4.2.6, §4.6. Output root is a node parameter defaulting under the run/bag output location (aligns with 05). |
| OQ-5 | `checkpoint_id` namespace / mapping: raw AprilTag id vs semantic name; where the `tag_id ↔ checkpoint_id` map lives. | PRD OQ-5 (PCAP-2) | **Provisional — settled default applied: read 03's shared `sim/config/checkpoints.yaml`; `checkpoint_id` is the semantic string from that file** | DEFERRED to human (cross-docset schema with 03, also consumed by 02). §4.2.5. 04 does NOT fork the mapping. Must reconcile with 03 §7 (03 owns the `{checkpoint_id, world position, tag_family, tag_id}` schema per the README ownership matrix). **Soft gate on T A.2** (the CheckpointConfigLoader is built against this schema — see §6.2 M6.A). |
| OQ-6 | `apriltag_ros` (or equivalent) availability/buildability on ROS 2 Jazzy in-container. | PRD OQ-6 (PCAP-2) | **Needs Input — verification action scheduled before Phase B** | §3.2 Verified Preconditions row VP-1 is currently UNVERIFIED. Clarification action: `apt-cache policy ros-jazzy-apriltag-ros` (or source-build probe) in the sim/dev container; if unavailable, fall back to the `apriltag` C library + a thin detector node (still off-the-shelf detection, not hand-rolled). Blocks committing the detection wiring in Phase B, not the message in Phase A. |
| OQ-7 | Camera-frame sampling discipline: latest-frame-on-trigger vs time-synced-to-pose; which TF frame the capture pose is expressed in. | PRD OQ-7 (PCAP-1) | **Resolved (design decision) → latest-frame-on-trigger; pose expressed in the world/ENU frame, frame_id stamped explicitly** | §4.2.3, §4.2.8, ADR-B (tenet 5: frames are explicit). |
| OQ-8 | Owner of the live-frame `sensor_msgs/CompressedImage` topic the bag records. | PRD OQ-8 (cross-docset) | **Resolved (combined review 2026-06-03): owner = 03-sim-environment (camera owner); topic /drone/camera/image_raw/compressed not created by this docset** | DEFERRED to human. Design binds to whatever live-frame topic exists; it does not create one (§4.4.1, §4.2 boundary note). Owner is a 03 (camera-topic shape: raw vs compressed) / 05 (recorded-image compression, their OQ §7) decision; neither 04's DoD §5 nor the Phase 1 README matrix assigns such a topic to 04. |
| OQ-9 [INFERRED] | `cv_bridge` (Image→numpy/encoded-file) availability on Jazzy in-container, used by FrameSampler/CaptureWriter to decode `sensor_msgs/Image` and encode PNG/JPEG. | Design (implementation of PCAP-1/PCAP-5) | **Needs Input — verification action scheduled with OQ-6** | §3.2 VP-2 UNVERIFIED. Bundled with the OQ-6 dependency probe; if `cv_bridge` or OpenCV encode is unavailable, the encode step falls back to a minimal PNG writer. Low-stakes (standard ROS perception dep). |

---

## 3. Existing Foundation

This is a greenfield perception capability inside an existing (still-thin) ROS 2 workspace. There is no prior perception code; the "existing foundation" is the workspace layout, the package shells from 01, the cross-docset interfaces this design binds to, and the stack pins.

### 3.1 Architecture context (ROS 2 workspace, single-host SITL)

```
                         ┌──────────────────────────────────────────────┐
                         │            sim/dev container (01)             │
                         │   ROS 2 Jazzy · Python 3.12 · Ubuntu 24.04    │
                         │                                               │
  Gazebo Harmonic (03) ──┤  /<camera>/image  (sensor_msgs/Image, 03)    │
  AprilTag world  (03)   │        │                                      │
                         │        ▼                                      │
  PX4 SITL (01) ─────────┤  apriltag_ros (external)  ──► /tag_detections │
   /fmu/out/* + TF       │        │                          │          │
                         │        ▼                          ▼          │
  Mission (02) ──────────┤   patrol_perception node  ◄── trigger (02)   │
   arrival/dwell         │        │            │                        │
                         │        ▼            ▼                        │
                         │  /patrol/checkpoint_capture   on-disk dir    │
                         │  (CheckpointCapture)          (img+sidecar)  │
                         └────────┬──────────────────────────┬─────────┘
                                  ▼                          ▼
                          bag pipeline (05)          operator inspects files
```

There is no UI tier and no API/gateway tier. The architectural "layers" relevant to this design are the ROS layers it actually inhabits: **Interface/Message** (`patrol_interfaces`), **Node/Orchestration** (`patrol_perception` ROS glue — subscriptions, publisher, parameters, lifecycle), **Domain/Logic** (the ROS-free capture-and-build core), and **Persistence** (the on-disk artifact writer). See §4.3.

### 3.2 Verified Preconditions

Each row is a claim this design depends on, to be verified against the actual external system. Rows marked **UNVERIFIED** carry a scheduled verification action and a corresponding OQ; per the auto-pilot policy these are recorded as deferred rather than silently assumed true.

| # | Claim | Verification command | Result | Citation / status |
|---|-------|----------------------|--------|-------------------|
| VP-1 | An `apriltag_ros`-equivalent ROS 2 package installs/builds on Jazzy in the sim/dev container and publishes detections carrying `tag_id` (+ family, +pose/confidence where available). | `apt-cache policy ros-jazzy-apriltag-ros` in the 01 container; else source-build probe in `ros2_ws`. | **UNVERIFIED — scheduled before Phase B** | OQ-6; settled constraint forbids hand-rolling (DoD §6). PRD H1, rabbit-hole §Technical Considerations. |
| VP-2 [INFERRED] | `cv_bridge` + an image encoder (OpenCV PNG/JPEG) are available on Jazzy in-container to decode `sensor_msgs/Image` and write a file. | `python3 -c "import cv_bridge, cv2"` in the 01 container. | **UNVERIFIED — scheduled with VP-1** | OQ-9; needed by FrameSampler/CaptureWriter. Fallback: minimal PNG writer. |
| VP-3 | 03 publishes exactly one RGB camera `sensor_msgs/Image` topic; its name/resolution/rate/frame_id are owned by 03 (TBD at design time). | Read 03 design once landed; at runtime `ros2 topic list \| grep image` + `ros2 topic info`. | **Owned by 03 (name TBD per 03 DoD §5/§7) — bind at config time, not hard-coded** | 03 DoD §5 (RGB camera `sensor_msgs/Image` topic, name TBD), §7 (camera topic name/resolution/rate/frame_id). This design takes the topic name as a node parameter (§4.2.1). |
| VP-4 | 03 ships a single shared checkpoint config (settled default `sim/config/checkpoints.yaml`) as a list of `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`. | Read the file in-repo once 03 lands it; schema-validate on node load. | **Owned by 03 (location/schema TBD per 03 DoD §7) — settled default applied; reconcile jointly** | 03 DoD §5 (checkpoint-position config schema `{checkpoint_id, position, tag_family, tag_id}`), §7 (file location/name/schema open, reconcile with 02+04); PRD OQ-5. This design reads the `tag_id ↔ checkpoint_id` relation only (§4.2.5). |
| VP-5 | 02 emits a checkpoint-arrival/"dwelling at checkpoint N" semantic this node can subscribe to (settled default: 02's waypoint/arrival topic). | Read 02 design; `ros2 topic info /patrol/current_waypoint` (or the agreed trigger topic). | **Owned by 02 (exact topic/field TBD) — `/patrol/current_waypoint` is a 02-owned topic; bind at config time** | 02 DoD §5 + Phase 1 README matrix (02 owns `/patrol/current_waypoint` + the checkpoint-arrival capture-trigger semantic); PRD OQ-3. Trigger topic name is a node parameter (§4.2.1). |
| VP-6 | 01 provides the `patrol_interfaces` package shell, `ros2_ws` + `colcon`, the container, and `/fmu/out/*` pose telemetry + TF. | `colcon list \| grep patrol_interfaces`; `ros2 topic list \| grep fmu/out`. | **Owned by 01 (package shell + telemetry exist)** | 01 DoD / Phase 1 README matrix (01 owns `patrol_interfaces` shell + `/fmu/out/*`); this design fills the shell's contents, it does not create the package. |

(No external precondition is asserted as already-true in §4 prose without a row here; VP-1/VP-2 are the only design-blocking unverified claims and are gated to Phase B.)

### 3.3 Architectural Decisions

**ADR-A — Capture trigger is an explicit signal from 02, gated by tag-in-view (resolves OQ-3).**
*Decision:* The node captures when (a) 02 signals checkpoint arrival/dwell on its waypoint/arrival topic AND (b) a checkpoint AprilTag is currently in view. *Rationale:* PRD tenet "sim path == hardware path" and the 02↔04 contract in both DoDs ("mission signals a checkpoint arrival"). Pure node-inference (AprilTag-in-view + dwell timer, no 02 signal) is rejected because it duplicates mission state the orchestrator already owns and risks firing off-route. *Implication:* PCAP-1 fires on a subscription callback, not a timer; deterministic one-capture-per-visit (AC-6) is enforced by a per-visit latch (§4.2.8). Pending 02 confirmation of the exact trigger topic/field.

**ADR-B — Latest-frame-on-trigger; capture pose in the world/ENU frame with explicit `frame_id` (resolves OQ-7).**
*Decision:* On trigger, sample the most recent buffered camera frame and the most recent pose; stamp the `PoseStamped.header.frame_id` with the world/ENU frame name (matching 03's `checkpoints.yaml` positions). *Rationale:* Phase 1 uses ground-truth sim pose (no VIO); a checkpoint visit is a quasi-static hover, so latest-frame is sufficient and far simpler than time-synchronized message filters. Tenet 5: frames are explicit, never implicit. *Implication:* No `message_filters` time-sync dependency in Phase 1; the design records the (small) sampling skew as acceptable for a hover and notes time-sync as a Phase-3+ thickening if motion-blur becomes an issue.

**ADR-C — `metadata` is `diagnostic_msgs/KeyValue[]` (resolves OQ-2).**
*Decision:* The message `metadata` field and the sidecar both carry the same key-value set; the message uses `diagnostic_msgs/KeyValue[]`, the sidecar serializes it as a JSON object. *Rationale:* Standard ROS type (no custom message-in-a-message), Foxglove-renderable, trivially round-trips to/from a JSON dict; avoids an opaque JSON-string-in-a-string-field that 05/analysis would have to re-parse. *Implication:* PCAP-6's "single shape used by both message and sidecar" is satisfied by one in-memory dict mapped to both surfaces (§4.2.4).

**ADR-D — ROS-free core behind a seam (enables AC-5 sub-second unit tests).**
*Decision:* All message-construction and metadata logic lives in plain-Python classes (`CheckpointCaptureBuilder`, `CheckpointResolver`, `CaptureWriter`) that take plain data, not ROS handles; the ROS node is a thin adapter. *Rationale:* PRD H3 + DoD AC-5 (message-construction tests run with no ROS/Gazebo/PX4, sub-second), mirroring the M3 `MissionStateMachine` separation. *Implication:* Unit tests import the core directly; the ROS node is exercised only in cross-docset integration (downstream).

---

## 4. Detailed Design

### 4.1 UC Traceability Matrix

| Design Component | Covers FRs / UACs |
|------------------|-------------------|
| **CheckpointCapture.msg** (`patrol_interfaces`) | PCAP-4 / UAC-PCAP-4; carries the surface for PCAP-1/2/3/5/6 |
| **CaptureCoordinator** (node) | PCAP-1 / UAC-PCAP-1; AC-6 (cardinality latch) |
| **FrameSampler** (node) | PCAP-1 / UAC-PCAP-1 (sample the 03 camera frame) |
| **PoseSampler** (node) | PCAP-1/PCAP-3 / UAC-PCAP-1, UAC-PCAP-3 (capture pose, explicit frame) |
| **CheckpointResolver** (core) | PCAP-2 / UAC-PCAP-2; PCAP-7 (confidence passthrough) |
| **CheckpointCaptureBuilder** (core) | PCAP-3, PCAP-4, PCAP-6 / UAC-PCAP-3, UAC-PCAP-4 |
| **CapturePublisher** (node) | PCAP-3 / UAC-PCAP-3 |
| **CaptureWriter** (core/persistence) | PCAP-5, PCAP-6 / UAC-PCAP-5 |
| **CheckpointConfig loader** (core) | PCAP-2 / UAC-PCAP-2 (reads 03's shared map) |
| **apriltag_ros** (external) | PCAP-2 / UAC-PCAP-2 (detection source) |

Every P1 UAC (UAC-PCAP-1…5) and every FR (PCAP-1…7) appears above; no orphan FRs.

### 4.2 Component Architecture

#### 4.2.1 Component Inventory

| Component | Type | Boundary (in / out) | Responsibility | Dependencies |
|-----------|------|---------------------|----------------|--------------|
| **CheckpointCapture.msg** | library (message) | IN: field schema. OUT: who fills it | Define the durable shared message in `patrol_interfaces` | std_msgs, geometry_msgs, diagnostic_msgs |
| **CheckpointConfigLoader** | module (core) | IN: read/validate 03's `checkpoints.yaml`. OUT: authoring the file | Load `tag_id ↔ checkpoint_id` (and family) map | PyYAML; 03's config file (VP-4) |
| **CheckpointResolver** | module (core) | IN: detection→checkpoint_id + confidence. OUT: running the detector | Map a detected `tag_id` to `checkpoint_id`; pull confidence | CheckpointConfigLoader |
| **FrameSampler** | module (node) | IN: buffer latest camera frame, encode to bytes. OUT: owning the camera topic | Hold the most recent `sensor_msgs/Image`; decode/encode | rclpy sub, cv_bridge (VP-2) |
| **PoseSampler** | module (node) | IN: buffer latest pose in world/ENU frame. OUT: pose estimation | Hold the most recent ground-truth pose; stamp frame_id | rclpy sub `/fmu/out/*` and/or TF |
| **CaptureCoordinator** | module (node) | IN: on trigger, assemble one capture; enforce cardinality. OUT: detection/encoding internals | Orchestrate trigger→sample→resolve→build→publish→write; per-visit latch | FrameSampler, PoseSampler, CheckpointResolver, CheckpointCaptureBuilder, CapturePublisher, CaptureWriter |
| **CheckpointCaptureBuilder** | module (core) | IN: assemble a `CheckpointCapture` + sidecar dict from plain data. OUT: ROS I/O | Build the message + the consistent sidecar dict | CheckpointCapture.msg types (data only) |
| **CapturePublisher** | module (node) | IN: publish on `/patrol/checkpoint_capture`. OUT: building the message | Own the ROS publisher | rclpy pub |
| **CaptureWriter** | module (persistence) | IN: write image file + JSON sidecar to the run dir. OUT: bag/MCAP | Persist the artifact referenced by `image_path` | filesystem; encoded image bytes |
| **PerceptionNode** | module (node, entrypoint) | IN: wire params/subs/pub/lifecycle. OUT: domain logic | rclpy node: declare params, build subs, host CaptureCoordinator | all node-layer components above |
| **apriltag_ros** | external | IN: consume detections. OUT: this design implements no detector | Detect tags, publish `tag_id`/family/(pose/confidence) | ROS 2 Jazzy (VP-1) |

#### 4.2.2 Component Dependency Diagram

```
                 trigger (02)        /tag_detections (apriltag_ros, external)
                     │                        │
   /<camera>/image   │   /fmu/out/* + TF      │
        │            │        │               │
        ▼            ▼        ▼               ▼
  ┌───────────┐  ┌─────────────────────────────────────────────┐
  │FrameSampler│  │              CaptureCoordinator             │
  └─────┬─────┘  │   (per-visit latch · AC-6 cardinality)       │
        │        └───┬───────────┬───────────┬──────────┬───────┘
        │            │           │           │          │
        ▼            ▼           ▼           ▼          ▼
  (latest frame) PoseSampler  Checkpoint  Checkpoint   (uses)
                     │        Resolver     CaptureBuilder
                     │           │           │
                     │           ▼           ▼
                     │     CheckpointConfig  ┌──────────────┬──────────────┐
                     │       Loader          ▼              ▼              │
                     │      (03 yaml)   CapturePublisher  CaptureWriter    │
                     │                       │              │              │
                     └───────────────────────▼              ▼              │
                                 /patrol/checkpoint_capture  on-disk dir    │
                                 (CheckpointCapture → 05)   (img + sidecar) │
                                                                            │
   PerceptionNode (entrypoint) owns/wires every node-layer box ────────────┘
```

Every inventory row (§4.2.1) appears as a node here, and every node traces back to a row — triangle corner 1↔2 consistent. The consumer-facing manifestation (corner 3) is the message + topic + on-disk contract enumerated in §4.2.3 and §4.6; every consumer-relevant row (CheckpointCapture.msg, CapturePublisher→topic, CaptureWriter→artifact) appears there.

**Boundary note (OQ-8):** This design does **not** create the live-frame `sensor_msgs/CompressedImage` topic the bag records. FrameSampler consumes the 03-owned camera topic; the separate CompressedImage live topic (if any) is owned by 03/05 (§4.4.1).

#### 4.2.3 CheckpointCapture.msg — the contract (PCAP-4)

**Type:** library (ROS message) · **Location:** `ros2_ws/src/patrol_interfaces/msg/CheckpointCapture.msg` · **Dependencies:** `std_msgs`, `geometry_msgs`, `diagnostic_msgs`

```
# patrol_interfaces/msg/CheckpointCapture
std_msgs/Header        header          # stamp + frame_id of the capture event
string                 checkpoint_id   # semantic checkpoint id from 03's checkpoints.yaml (PCAP-2)
geometry_msgs/PoseStamped pose         # capture pose, world/ENU frame, explicit frame_id (ADR-B)
string                 image_path      # path to the PNG/JPEG written by CaptureWriter (OQ-1 settled default)
diagnostic_msgs/KeyValue[] metadata    # free-form key-value pairs (PCAP-6; ADR-C)
```

Field-set is exactly the DoD-mandated set (header / checkpoint_id / pose / image / metadata) — no typed extension fields; `metadata` is the extension point for later phases (PRD §Potential challenges). The image field is `string image_path` per the settled OQ-1 default (path, not by-value pixels); live frames travel on a separate CompressedImage topic the bag records (owner = OQ-8, not this design).

*Traces to: PCAP-4 / UAC-PCAP-4; carries the surface for PCAP-1/2/3/5/6.*

#### 4.2.4 CheckpointCaptureBuilder (core) — message + sidecar from one source (PCAP-3, PCAP-4, PCAP-6)

**Type:** module (ROS-free core) · **Location:** `ros2_ws/src/patrol_perception/patrol_perception/capture_builder.py` · **Dependencies:** message field types as plain data only.

```python
@dataclass
class CaptureRecord:
    stamp: Time                 # capture time (from trigger/frame)
    frame_id: str               # world/ENU frame name (ADR-B)
    checkpoint_id: str          # resolved from tag (PCAP-2)
    pose: Pose                  # x,y,z + orientation, world/ENU
    image_path: str             # relative/abs path written by CaptureWriter
    metadata: dict[str, str]    # single source of KV pairs (ADR-C)

class CheckpointCaptureBuilder:
    def build_message(self, rec: CaptureRecord) -> CheckpointCapture:
        """Effect: returns a fully-populated CheckpointCapture.
        Guards: checkpoint_id non-empty; pose.frame_id == rec.frame_id.
        No ROS handles touched — pure construction (AC-5)."""

    def build_sidecar(self, rec: CaptureRecord) -> dict:
        """Effect: returns the JSON-serializable sidecar dict carrying the
        SAME (checkpoint_id, pose, image_path basename, metadata, stamp) as
        the message (PCAP-3 consistency, PCAP-6 single shape)."""
```

The same `rec.metadata` dict feeds both `metadata` (as `KeyValue[]`) and the sidecar (as a JSON object) — one shape, two surfaces (PCAP-6). `build_message`/`build_sidecar` are the AC-5 unit-test surface: constructed from plain `CaptureRecord` instances, no ROS spin-up, sub-second.

*Traces to: PCAP-3, PCAP-4, PCAP-6 / UAC-PCAP-3, UAC-PCAP-4.*

#### 4.2.5 CheckpointResolver + CheckpointConfigLoader (core) — tag → checkpoint_id (PCAP-2)

**Type:** module (ROS-free core) · **Location:** `…/patrol_perception/checkpoint_resolver.py`, `…/checkpoint_config.py` · **Dependencies:** PyYAML; 03's `checkpoints.yaml` (VP-4).

```python
class CheckpointConfigLoader:
    def load(self, path: str) -> dict[int, CheckpointEntry]:
        """Effect: parse 03's checkpoints.yaml into {tag_id: CheckpointEntry}.
        Guards: schema-validate each row has checkpoint_id, position{x,y,z},
                tag_family, tag_id; reject duplicate tag_id (would break the map).
        04 reads the tag_id↔checkpoint_id relation only; does NOT author/fork it."""

class CheckpointResolver:
    def resolve(self, detection) -> tuple[str, dict[str, str]]:
        """Input: an apriltag_ros detection (tag_id, family, optional confidence).
        Effect: returns (checkpoint_id, extra_metadata).
        Guards: tag_id present in the loaded map; family matches the config's
                tag_family (silent-frame/silent-tag mistakes are infuriating).
        extra_metadata includes 'tag_id' and, when present, 'detection_confidence'
        (PCAP-7)."""
```

`checkpoint_id` is the **semantic string** from 03's config (settled OQ-5 default), not the raw integer tag id — so one schema maps `checkpoint_id ↔ world position ↔ tag_id` and 02/03/04 stay coherent. No second mapping store lives in `patrol_perception`. Because the `checkpoints.yaml` schema is a cross-docset contract still pending confirmation (OQ-5), `CheckpointConfigLoader`'s parse/validate is built against the settled-default schema and is the first thing to re-check if 03's confirmed schema differs (see §6.2 M6.A dependency note).

*Traces to: PCAP-2 / UAC-PCAP-2; PCAP-7.*

#### 4.2.6 CaptureWriter (persistence) — on-disk artifact (PCAP-5)

**Type:** module (persistence) · **Location:** `…/patrol_perception/capture_writer.py` · **Dependencies:** filesystem; encoded image bytes from FrameSampler.

**Layout (resolves OQ-4):**
```
<output_root>/<mission_id_or_runts>/
    000_<checkpoint_id>.png        # image referenced by image_path
    000_<checkpoint_id>.json       # sidecar (build_sidecar dict)
    001_<checkpoint_id>.png
    001_<checkpoint_id>.json
    ...
```
- `<output_root>` is a node parameter (`output_root`), defaulting under the run/bag output location so it aligns with 05's known output convention.
- Index prefix (`NNN`) is the monotonically increasing per-visit counter → guarantees one image + one sidecar per checkpoint visit (AC-1) and gives stable ordering even if a `checkpoint_id` repeats across a multi-loop patrol.
- Sidecar is **JSON** (ADR-C: trivial to round-trip the KeyValue set; ubiquitous; Foxglove/analysis-friendly).

```python
class CaptureWriter:
    def write(self, rec: CaptureRecord, image_bytes: bytes) -> str:
        """Effect: writes <root>/<run>/NNN_<checkpoint_id>.{png,json}.
        Returns the image path that goes into rec.image_path (PCAP-5 consistency
        with the published message, UAC-PCAP-5).
        Guards: output dir exists/creatable; write image THEN sidecar so a
        sidecar never references a missing image."""
```

*Traces to: PCAP-5, PCAP-6 / UAC-PCAP-5.*

#### 4.2.7 FrameSampler + PoseSampler (node) — sampling the world (PCAP-1)

**Type:** module (node) · **Location:** `…/patrol_perception/samplers.py` · **Dependencies:** rclpy subscriptions; cv_bridge (VP-2); TF/`/fmu/out/*`.

- **FrameSampler** subscribes to the 03 camera topic (name = `camera_topic` parameter, VP-3), keeps only the latest `sensor_msgs/Image`, and on demand returns `(image_msg, encoded_bytes)`. Encoding (`sensor_msgs/Image`→PNG/JPEG) happens here so CaptureWriter stays filesystem-only and the core stays ROS-free.
- **PoseSampler** subscribes to `/fmu/out/*` (and/or reads TF) for ground-truth pose, keeps the latest, and returns a `Pose` stamped with the world/ENU `frame_id` (ADR-B). No VIO/SLAM (settled constraint).

Latest-frame-on-trigger (ADR-B): both samplers return their most-recent buffered value when CaptureCoordinator fires; no time-sync filter in Phase 1.

*Traces to: PCAP-1 / UAC-PCAP-1.*

#### 4.2.8 CaptureCoordinator (node) — one capture per visit (PCAP-1, AC-6)

**Type:** module (node) · **Location:** `…/patrol_perception/coordinator.py` · **Dependencies:** all samplers + core modules + publisher + writer.

```python
class CaptureCoordinator:
    def on_trigger(self, checkpoint_visit_token) -> None:
        """Guards (AC-6 cardinality latch): if this visit token is already
            latched, return (no duplicate). If no tag currently in view
            (ADR-A gate), skip/log (no capture without identification) and
            DO NOT latch — a re-trigger for the same visit may retry.
        Effect: sample frame+pose → resolve checkpoint_id (PCAP-2) →
            CaptureWriter.write (PCAP-5) → CheckpointCaptureBuilder.build_message →
            CapturePublisher.publish (PCAP-3). Latch the visit token ONLY after a
            successful capture (latch is the final step), so skipped visits
            (no-frame, no-tag-in-view, unmapped-tag_id) remain retryable.
        Side effects: one /patrol/checkpoint_capture message + one img + one sidecar."""
```

The **per-visit latch** is the AC-6 mechanism: a visit token (from 02's arrival/waypoint signal) is captured at most once; a new token (next checkpoint, or a re-visit on a second loop) re-arms. The latch is set **only on a successful capture** (it is the last action in the effect path), so any guard that skips the visit — no buffered frame, no tag in view, or a detected `tag_id` not present in 03's config — leaves the token unlatched and lets a re-trigger retry. This makes capture cardinality deterministic per *successful* visit (no dup; a genuinely identifiable checkpoint is never dropped) while honoring AC-4's "no fabricated checkpoint_id" — a skip yields zero captures rather than a hardcoded one. The per-skip degraded behavior is enumerated in §4.4.5.

*Traces to: PCAP-1 / UAC-PCAP-1; AC-6.*

#### 4.2.9 PerceptionNode + CapturePublisher (node entrypoint) (PCAP-3)

**Type:** module (node entrypoint) · **Location:** `…/patrol_perception/perception_node.py` · **Dependencies:** all node-layer components.

Declares node parameters, constructs subscriptions (camera, trigger, pose, `/tag_detections`), constructs the `CapturePublisher` on `/patrol/checkpoint_capture` (type `patrol_interfaces/msg/CheckpointCapture`), and hosts the `CaptureCoordinator`. **Node parameters** (the bind-at-config seam, so nothing cross-docset is hard-coded):

| Parameter | Default | Binds to | OQ |
|-----------|---------|----------|----|
| `camera_topic` | (unset — required) | 03 camera `Image` topic name | VP-3 |
| `trigger_topic` | `/patrol/current_waypoint` | 02 arrival/dwell semantic | OQ-3/VP-5 |
| `detections_topic` | `/tag_detections` | apriltag_ros output | VP-1 |
| `checkpoint_config_path` | `sim/config/checkpoints.yaml` | 03 shared map | OQ-5/VP-4 |
| `output_root` | `<run/bag output dir>` | 05 output alignment | OQ-4 |
| `world_frame` | (from config) | ENU frame name | ADR-B |

All six parameters are internal configuration seams whose values trace to a cross-docset interface (the OQ/VP column) or a resolved ADR — none introduces a new requirement surface beyond the PRD's FR table.

*Traces to: PCAP-3 / UAC-PCAP-3.*

### 4.3 Layer View

#### 4.3.1 Layer Mapping (ROS-workspace layers, derived from §3.1)

| Layer | Components | Key responsibilities |
|-------|-----------|----------------------|
| **Interface / Message** | CheckpointCapture.msg (`patrol_interfaces`) | The durable shared contract; compiled type used by 04 and 05 |
| **Node / Orchestration** | PerceptionNode, CaptureCoordinator, CapturePublisher, FrameSampler, PoseSampler | rclpy glue: params, subs/pub, lifecycle, trigger handling, per-visit latch |
| **Domain / Logic (ROS-free)** | CheckpointResolver, CheckpointConfigLoader, CheckpointCaptureBuilder | Pure logic: map tag→checkpoint, build message+sidecar (AC-5 testable) |
| **Persistence** | CaptureWriter | On-disk image + JSON sidecar |
| **External** | apriltag_ros | Off-the-shelf detection (not implemented here) |

#### 4.3.2 Domain/Logic layer — design notes

**Conventions:** mirrors the M3 `MissionStateMachine` ROS-free seam (PRD H3). **New in this design:** the `CaptureRecord` dataclass as the boundary object between node and core. **Integration points:** the Node layer adapts ROS messages into plain `CaptureRecord`/detection data before calling the core; the core never imports rclpy. This is the layer that makes AC-5 (sub-second, no ROS) achievable and is the layer that does not leak ROS concerns downward.

**No layer violations:** the Node layer never writes files directly (delegates to CaptureWriter); the Persistence layer never publishes; the Message layer is data-only.

### 4.4 Systemic / Platform Interfaces

Interface categories elicited from the actual system (a single-host SITL ROS 2 workspace). There is **no auth/security tier, no UI tier, no database, and no multi-cluster/mesh** in this design; those rows are marked OOS with rationale below rather than padded with boilerplate.

#### 4.4.1 Interface Integration Summary

| Interface | Current state (§3) | Design changes | Priority |
|-----------|--------------------|----------------|----------|
| Messaging / Topics (ROS 2 DDS) | 01 provides DDS + `/fmu/out/*`; 03 will publish camera; 02 will publish trigger | **New publisher** `/patrol/checkpoint_capture`; new subscriptions (camera, trigger, `/tag_detections`, pose) | P1 |
| Configuration (ROS params + shared YAML) | 03 owns `checkpoints.yaml`; node params via launch | All cross-docset bindings are node parameters (no hard-coding); read-only consume of 03's YAML | P1 |
| Persistence (filesystem) | container filesystem; 05 owns bag/MCAP output dir | New on-disk capture dir (images + JSON sidecars) under `output_root` | P1 |
| Observability (rosout logging) | rclpy logger / `rosout` (01 baseline) | Structured log lines on capture, skip (no tag-in-view), config-load failure, encode/write failure | P2 |
| Live-frame CompressedImage topic (OQ-8) | **Not owned here** — 03 camera-shape / 05 recorded-image-compression decision | This design **consumes** it if present; does not create it | (cross-docset) |
| Security / Auth | none — single-host SITL, no users, no network auth surface | `[OOS: Phase 1 single-host SITL; no authN/authZ surface, no tenants. Foxglove visualization auth is 05's concern.]` | — |
| UI | none — operator inspects files / Foxglove (05) | `[OOS: no UI in this docset; Foxglove panels are 05/exit-item-8.]` | — |

#### 4.4.2 Messaging / Topics

**Current state:** 01 provides the DDS layer and PX4 telemetry; 03/02 will add camera/trigger topics. **Design changes:** publish one `CheckpointCapture` per visit on `/patrol/checkpoint_capture`; subscribe to camera (`camera_topic` param), trigger (`trigger_topic` param), `/tag_detections`, and pose. QoS: publisher uses reliable, depth-1 (latched-style) so 05's recorder reliably captures each low-rate event message; camera subscription uses sensor-data QoS (best-effort) to match 03's stream.

**Failure mode:** if the camera topic is silent at trigger time, FrameSampler has no buffered frame → CaptureCoordinator logs `no_frame_available` and skips that visit (no malformed capture). If `/tag_detections` is empty (no tag in view), ADR-A gate skips (no capture without identification).

#### 4.4.3 Configuration

**Current state:** node params via launch; 03 owns `checkpoints.yaml`. **Design changes:** every cross-docset name is a parameter (§4.2.9 table), so a rename in 03/02 is a config change, not a code change. The checkpoint config is loaded once at node start and schema-validated. **Failure mode:** config missing/malformed/duplicate-tag_id → node fails fast at startup with a clear error (don't run blind); a tag detected whose id is absent from the map → log `unmapped_tag_id` and skip that capture (don't emit a `checkpoint_id` the config doesn't define).

#### 4.4.4 Persistence (filesystem)

**Current state:** container fs; 05 owns the bag/MCAP output dir. **Design changes:** CaptureWriter creates `<output_root>/<run>/` and writes image-then-sidecar. **Failure mode:** see §4.4.5.

#### 4.4.5 Cross-cutting Failure Modes

| Category | Failure mode | Detection | Degraded behavior | Recovery |
|----------|--------------|-----------|-------------------|----------|
| Persistent state | `output_root` not writable / disk full mid-mission | `OSError` on `write()` | Log `capture_write_failed`; still publish the `/patrol/checkpoint_capture` message (topic is the bag's source of truth) but with `image_path` flagged unwritten; continue patrol | Operator frees space / fixes mount; subsequent captures resume |
| Persistent state | Sidecar written but image write failed (partial artifact) | image-then-sidecar ordering check | Skip sidecar when image write fails (ordering guarantees no dangling sidecar) | Next visit writes a complete pair |
| Network dependency (topic) | Camera topic stale/silent at trigger | No buffered frame in FrameSampler | Log `no_frame_available`; skip the visit (no malformed capture); cardinality latch NOT set so a re-trigger can retry | Auto-recovers when frames resume |
| Network dependency (topic) | `/tag_detections` empty (no tag in view) | ADR-A gate | Skip capture (PCAP-2 requires tag-sourced id); log `no_tag_in_view`; latch NOT set (re-trigger may retry) | Next checkpoint with a visible tag captures normally |
| Network dependency (topic) | Detected `tag_id` not in 03's config | Resolver map lookup miss | Log `unmapped_tag_id`; skip (don't fabricate a checkpoint_id, AC-4); latch NOT set | Fix 03 config / family mismatch; reconcile (OQ-5) |
| Config | `checkpoints.yaml` missing/malformed/dup tag_id at startup | Schema validation on load | **Fail fast** at node start (don't run blind on a broken map) | Fix the shared config; restart node |
| Identity provider | (auth/identity) | — | `[OOS: no identity/auth in single-host SITL]` | — |
| Mesh / cross-cluster | (multi-cluster drift) | — | `[OOS: single-host SITL, no mesh/cross-cluster in Phase 1]` | — |
| Plugin / extension | apriltag_ros crash / not loaded | No `/tag_detections` publisher | No captures produced (same as no-tag-in-view, but persistent); log on first trigger with no detections subscriber | Restart/rebuild detector; VP-1 gates this before Phase B |

#### 4.4.6 Failure-mode completeness note

The three "skip" rows (no-frame, no-tag-in-view, unmapped-tag_id) all share the **latch-not-set** invariant from §4.2.8: a skipped visit is retryable, and only a *successful* capture latches the token. This is the single rule that reconciles AC-6 (deterministic, ≤1 per visit) with AC-4 (no fabricated `checkpoint_id`) — a visit that cannot be honestly identified produces zero captures, not a wrong one, and is not silently swallowed (every skip logs).

### 4.5 Key Interaction Sequences

#### Sequence 1: Happy path — capture at a checkpoint (PCAP-1/2/3/5)

```
Mission(02)      apriltag_ros   PerceptionNode   FrameSampler  PoseSampler  Resolver   Writer    Publisher   Bag(05)
   |                  |              |                |             |           |          |          |          |
   ├ arrival/dwell ──────────────► on_trigger        |             |           |          |          |          |
   |                  |              ├─ in view? ─────┼─────────────┼──────────►│(tag set?)│          |          |
   |               (/tag_detections)─┼───────────────┼─────────────┼──────────►│          |          |          |
   |                  |              ├─ get frame ───►│ latest img  │           |          |          |          |
   |                  |              ├─ get pose ─────┼────────────►│ world/ENU │          |          |          |
   |                  |              ├─ resolve ──────┼─────────────┼──────────►│ checkpoint_id+conf  |          |
   |                  |              ├─ write(img,sidecar) ─────────┼───────────┼─────────►│ image_path          |
   |                  |              ├─ build_message + publish ────┼───────────┼──────────┼─────────►│ ─────────►│ records
   |                  |              └─ latch visit (AC-6, only after success)  |          |          |          |
```

#### Sequence 2: No tag in view (error/edge — ADR-A gate)

```
Mission(02)        PerceptionNode    apriltag_ros
   ├ arrival/dwell ──► on_trigger        |
   |                     ├─ in view? ───►│ (no detections)
   |                     └─ skip + log 'no_tag_in_view'; latch NOT set (re-trigger may retry)
```

#### Sequence 3: Duplicate-trigger suppression (AC-6 cardinality)

```
Mission(02)        PerceptionNode
   ├ arrival(token=K) ──► on_trigger ──► capture once, latch K
   ├ arrival(token=K) ──► on_trigger ──► token K latched → return (no duplicate capture)
   ├ arrival(token=K+1)─► on_trigger ──► new token → capture (re-armed)
```

#### Sequence 4: Unit-test path (AC-5 — no ROS)

```
pytest            CheckpointCaptureBuilder      CaptureWriter (tmp dir)
   ├ CaptureRecord(...) ─► build_message ──► assert fields populated (sub-second)
   ├ CaptureRecord(...) ─► build_sidecar ──► assert sidecar == message KV set (PCAP-6 consistency)
   ├ Resolver.resolve(fake_detection) ──► assert checkpoint_id from config map, not constant (AC-4)
   └ (no rclpy / Gazebo / PX4 imported)
```

### 4.6 Data Model Changes (Consolidated)

No database. Two data surfaces:

| Surface | Change | Detail |
|---------|--------|--------|
| `patrol_interfaces/msg/CheckpointCapture` | **New message** | `header`, `checkpoint_id` (string), `pose` (PoseStamped, world/ENU frame), `image_path` (string), `metadata` (`diagnostic_msgs/KeyValue[]`). Compiled once, used by 04 (publish) and 05 (record). |
| On-disk capture artifact | **New artifact contract** | `<output_root>/<run>/NNN_<checkpoint_id>.png` + `.json`; JSON sidecar carries the same `(checkpoint_id, pose, image_path, metadata, stamp)` as the message. |
| 03's `sim/config/checkpoints.yaml` | **Consumed read-only (owned by 03)** | `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`; 04 reads the `tag_id↔checkpoint_id` relation only. |

### 4.7 UX Mocks

`[OOS: This docset has no UI. The operator-facing surface is (a) a browsable directory of `NNN_<checkpoint_id>.png` + `.json` files — the layout in §4.2.6 is the "wireframe" — and (b) Foxglove panels rendering `/patrol/checkpoint_capture`, which are owned by docset 05 / exit-item 8. No screens, states, or role variations apply.]`

---

## 5. Design Questions FAQ

### Q1: Main components and interactions
New package contents under two packages. `patrol_interfaces` gains **CheckpointCapture.msg** (the durable contract). `patrol_perception` gains the node: **PerceptionNode** (entrypoint/params) hosting **CaptureCoordinator** (trigger→capture orchestration + AC-6 latch), **FrameSampler**/**PoseSampler** (latest frame + ground-truth pose), and the ROS-free core — **CheckpointResolver** + **CheckpointConfigLoader** (tag→checkpoint_id via 03's YAML), **CheckpointCaptureBuilder** (message + sidecar), **CapturePublisher** (`/patrol/checkpoint_capture`), **CaptureWriter** (on-disk artifact). Detection is delegated to external **apriltag_ros**. Build order: message first (Phase A), then node (Phase B), then persistence/metadata (Phase C). Every component in this prose appears in §4.2.1 inventory.

### Q2: Core API contracts and data models
There is no REST/SDK API. The contracts are: (1) the **message** `patrol_interfaces/msg/CheckpointCapture` (§4.2.3 — `header`/`checkpoint_id` string/`pose` PoseStamped/`image_path` string/`metadata` KeyValue[]); (2) the **topic** `/patrol/checkpoint_capture` (reliable, depth-1); (3) the **on-disk artifact** `<output_root>/<run>/NNN_<checkpoint_id>.{png,json}` (§4.2.6); (4) the **consumed** 03 `checkpoints.yaml` schema (read-only). The core class contracts (`build_message`/`build_sidecar`/`resolve`/`write`) are in §4.2.4–4.2.6. Image is `string image_path` (OQ-1 settled default), `metadata` is `diagnostic_msgs/KeyValue[]` (ADR-C).

### Q3: Deployment and infrastructure dependencies
Runs inside 01's sim/dev container on ROS 2 Jazzy / Python 3.12 / Ubuntu 24.04. No new infrastructure, no DB, no daemon. New dependencies to install in-container: **apriltag_ros** (VP-1, OQ-6 — verify before Phase B) and **cv_bridge/OpenCV** (VP-2, OQ-9). Configuration is via launch parameters (§4.2.9 table: `camera_topic`, `trigger_topic`, `detections_topic`, `checkpoint_config_path`, `output_root`, `world_frame`) — every cross-docset binding is a parameter, no hard-coding. `colcon build` of `patrol_interfaces` + `patrol_perception` must succeed in-container (AC-7). No scaling concern: event-driven, ~one capture per checkpoint per ~5-min mission.

### Q4: External components and interfaces
**apriltag_ros** (external, VP-1) — consumes detections (`tag_id`, family, optional pose/confidence) on `/tag_detections`; this design implements no detector (settled constraint; "or equivalent" hedge retained, OQ-6). **03 camera topic** (`sensor_msgs/Image`, name TBD, VP-3) — consumed via `camera_topic` param. **03 checkpoint config** (`checkpoints.yaml`, VP-4) — read-only. **02 trigger** (arrival/dwell semantic, VP-5) — consumed via `trigger_topic` param. **01 pose** (`/fmu/out/*` + TF, VP-6). Every external dep here has a §4.4 row (Messaging/Config) and a §3.2 VP row. The OQ-8 CompressedImage live-frame topic is consumed-if-present, not owned.

### Q5: Testing strategy (unit, integration, E2E)
**Unit (this docset, per-PR, AC-5):** import the ROS-free core directly (no rclpy/Gazebo/PX4) and assert: `build_message` populates every field; `build_sidecar` carries the identical KV set (PCAP-6); `resolve` returns a `checkpoint_id` sourced from the config map and not a hardcoded constant (AC-4); `CaptureWriter.write` produces image+sidecar in a tmp dir with consistent `image_path` (PCAP-5); the coordinator latch suppresses a duplicate trigger and leaves a skipped visit retryable (AC-6 + §4.2.8 invariant). Suite runs sub-second. **Container build (per-PR, AC-7):** `colcon build` of both packages. **Integration (cross-docset, downstream):** SITL patrol over the M5 world → exactly one `/patrol/checkpoint_capture` per checkpoint with a tag-sourced `checkpoint_id` (AC-2/AC-4/AC-6) and a populated output directory (AC-1); 05 records the topic with the identical compiled type (their AC-7). These test categories match §6.2 milestone testing tables exactly.

### Q6: Security implications and auth interactions
`[OOS for this docset]` — single-host SITL with no users, no network-exposed endpoints, no tenants, no credentials. There is no authN/authZ surface to defend; the message carries no sensitive data (sim imagery + sim pose). The only "trust" assumption is that the consumed topics originate from the same in-container ROS graph (DDS domain), which is the platform baseline owned by 01. Foxglove visualization access control is 05's concern. No privilege-escalation path exists because there are no privileges. (Recorded as deliberate OOS, not an omission.)

### Q7: Technical risks and open questions
Top risks, each tracked as an OQ in §2: **(1) apriltag_ros on Jazzy may not install cleanly** (OQ-6/VP-1, Needs Input) — gates Phase B; fallback is the `apriltag` C lib + thin detector node (still off-the-shelf). **(2) Image representation** (OQ-1, Provisional, settled default `image_path`) — cross-docset with 05's bag budget; wrong call forces a re-record. **(3) checkpoint_id mapping source** (OQ-5, Provisional, settled default = 03's `checkpoints.yaml`) — must reconcile one schema across 02/03/04; a confirmed-schema delta re-touches `CheckpointConfigLoader` (soft gate on T A.2). **(4) Unowned CompressedImage live-frame topic** (OQ-8, Open) — this design refuses to invent it; flagged for human assignment to 03/05. **(5) Trigger contract with 02** (OQ-3, resolved to explicit-signal-gated-by-tag, pending 02 confirmation). Every "Provisional/Open/Needs Input" status here matches an OQ row in §2.

---

## 6. Implementation Plan

### 6.0 Linear Project
**Project:** TBD (no upstream Linear project bound at design time) · **Team:** TBD · **Initiative:** Patrol-Drone Phase 1 · **Created from:** Section 6 of this document.

### 6.1 Milestone Overview

Walking-skeleton decomposition of M6 (mirrors the PRD's Phases A/B/C). Each milestone ends in an observable demo. M6 is a single plan-milestone; "shippable" here means "a reviewer can build/run and see the demo," appropriate to a solo-dev sim phase.

| # | Milestone | Type | Shippable Demo | Scope | Dependencies | Exit Criteria | Linear |
|---|-----------|------|----------------|-------|-------------|---------------|--------|
| M6.A | Message contract first | skeleton | Reviewer runs `colcon build` and `ros2 interface show patrol_interfaces/msg/CheckpointCapture`; runs the unit suite green sub-second | `CheckpointCapture.msg` + the ROS-free core (Builder/Resolver/ConfigLoader stubs) + message-construction unit tests | 01 package shell; 03 `checkpoints.yaml` schema (settled default — OQ-5 confirmation is a soft gate on T A.2, see M6.A note) | AC-3, AC-5, AC-7 (interfaces build); unit suite green sub-second, no ROS spin-up | *post-approval* |
| M6.B | Capture + detect + publish | layer 1: live capture | In a SITL patrol over the M5 world, `ros2 topic echo /patrol/checkpoint_capture` shows one well-formed message per checkpoint with a tag-sourced `checkpoint_id` | PerceptionNode, samplers, apriltag_ros wiring, CaptureCoordinator latch, CapturePublisher | M6.A; VP-1/VP-2 verified; 03 camera+tags (M5); 02 trigger | AC-2, AC-4, AC-6 — one tagged message per checkpoint live | *post-approval* |
| M6.C | Persist + metadata | layer 2: on-disk artifact | After a completed patrol, reviewer browses `<output_root>/<run>/` with one image + one JSON sidecar per checkpoint, consistent with the topic | CaptureWriter + `image_path` wiring + `metadata`/sidecar (PCAP-6); PCAP-7 confidence passthrough | M6.B | AC-1 — directory of images + sidecars matching checkpoints traversed; sidecar↔message consistency | *post-approval* |

### 6.2 Milestone Details

#### M6.A: Message contract first
**Type:** skeleton · **Goal:** lock the durable `CheckpointCapture` schema and the testable ROS-free core before any live wiring. **Shippable demo:** reviewer builds the workspace and runs `ros2 interface show patrol_interfaces/msg/CheckpointCapture` + the green unit suite. **Dependencies:** 01 `patrol_interfaces` shell; 03 `checkpoints.yaml` schema (settled default). **Exit criteria:** AC-3 (message exists with the field set), AC-5 (sub-second, no-ROS unit suite), AC-7 (both packages `colcon build`).

> **OQ-5 soft-gate note (cross-docset):** T A.2 (`CheckpointConfigLoader`) parses 03's `sim/config/checkpoints.yaml` against the *settled-default* schema while OQ-5 is still confirmed at combined review (2026-06-03). M6.A may proceed on the default (the loader and its schema-validation are the cheapest things to re-touch), but if 03's confirmed schema differs from `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`, T A.2's validator and `CheckpointEntry` shape are the first re-work. The message field set (T A.1) is independent of the YAML schema and is not gated by OQ-5. Confirming OQ-5 before M6.B is preferred so the resolver wires against the final map.

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| Live camera subscription / detection wiring | design §4.2.7–4.2.9 (node layer) | M6.B |
| On-disk persistence | design §4.2.6 | M6.C |
| Live-frame CompressedImage topic | OQ-8 (not owned by this docset) | never (this docset) |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Dependencies |
|---|------|---------------|-----------|-------|------|-------------|
| T A.1 | Define `CheckpointCapture.msg` + `package.xml`/`CMakeLists` deps (std/geometry/diagnostic) | `patrol_interfaces/msg/CheckpointCapture.msg` (new); `patrol_interfaces/CMakeLists.txt` (modify); `patrol_interfaces/package.xml` (modify) | CheckpointCapture.msg | Interface/Message | M | — |
| T A.2 | `CheckpointConfigLoader` + schema validation (settled-default schema; re-touch if OQ-5 confirms a delta) | `patrol_perception/checkpoint_config.py` (new) | CheckpointConfigLoader | Domain/Logic | M | T A.1 |
| T A.3 | `CheckpointResolver` (tag→checkpoint_id + confidence) | `patrol_perception/checkpoint_resolver.py` (new) | CheckpointResolver | Domain/Logic | M | T A.2 |
| T A.4 | `CheckpointCaptureBuilder` (build_message + build_sidecar) | `patrol_perception/capture_builder.py` (new) | CheckpointCaptureBuilder | Domain/Logic | M | T A.1 |
| T A.5 | Unit suite (no ROS) for resolver/builder/sidecar consistency | `tests/unit/test_capture_builder.py` (new); `tests/unit/test_resolver.py` (new) | (core) | Domain/Logic | M | T A.3, T A.4 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | core only, no ROS | build_message fields; build_sidecar == message KV set (PCAP-6); resolve sources checkpoint_id from config not constant (AC-4); sub-second (AC-5) |
| Build | both packages | `colcon build` clean in-container (AC-7) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| `patrol_interfaces` README note | downstream docsets (05, Phases 3/4/6) | The `CheckpointCapture` field contract + `image_path` rationale |

#### M6.B: Capture + detect + publish
**Type:** layer 1: live capture · **Goal:** make the node capture-detect-publish in a running patrol. **Shippable demo:** `ros2 topic echo /patrol/checkpoint_capture` during a SITL patrol shows one tagged message per checkpoint. **Dependencies:** M6.A; VP-1/VP-2 verified (OQ-6/OQ-9); 03 M5 world+camera+tags; 02 trigger; OQ-5 confirmation preferred (so the resolver binds the final `tag_id↔checkpoint_id` map). **Exit criteria:** AC-2, AC-4, AC-6.

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| On-disk artifacts + sidecar | design §4.2.6 | M6.C |
| Time-synchronized frame/pose sampling | ADR-B (latest-frame is sufficient Phase 1) | Phase 3+ |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Dependencies |
|---|------|---------------|-----------|-------|------|-------------|
| T B.1 | Verify/install apriltag_ros + cv_bridge in-container; record VP-1/VP-2 result | `docker/dev/` or `ros2_ws` deps (modify); design §3.2 (update) | apriltag_ros (external) | External | M | M6.A |
| T B.2 | `FrameSampler` + `PoseSampler` (latest-frame, world/ENU pose) | `patrol_perception/samplers.py` (new) | FrameSampler, PoseSampler | Node | M | T B.1 |
| T B.3 | `CaptureCoordinator` (trigger handling + per-visit latch + ADR-A gate; latch only on success) | `patrol_perception/coordinator.py` (new) | CaptureCoordinator | Node | L | T B.2; T A.3, T A.4 |
| T B.4 | `PerceptionNode` + `CapturePublisher` + params + launch | `patrol_perception/perception_node.py` (new); `patrol_perception/capture_publisher.py` (new); `patrol_bringup`/launch include (modify) | PerceptionNode, CapturePublisher | Node | L | T B.3 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | coordinator latch logic (fakes) | duplicate trigger suppressed (AC-6); no-tag-in-view skips and stays retryable (ADR-A + §4.2.8 latch-only-on-success) |
| Integration (downstream) | SITL patrol over M5 world | one tagged `/patrol/checkpoint_capture` per checkpoint (AC-2/AC-4); cardinality (AC-6) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| Node param reference | operator / 02 / 03 | the §4.2.9 parameter table + how to bind camera/trigger/config topics |

#### M6.C: Persist + metadata
**Type:** layer 2: on-disk artifact · **Goal:** write the inspectable on-disk artifact and populate metadata. **Shippable demo:** browse `<output_root>/<run>/` after a patrol; one image + one sidecar per checkpoint, consistent with the topic. **Dependencies:** M6.B. **Exit criteria:** AC-1; sidecar↔message consistency (PCAP-5).

##### Out of Scope
| Item | Source | Deferred to |
|------|--------|-------------|
| Recording the artifacts into the MCAP bag | PRD §Out of Scope (owned by 05) | docset 05 (M7) |
| Typed metadata fields (beyond free-form KV) | PRD §Potential challenges (metadata is the extension point) | Phases 3/4/6 |

##### Tasks
| # | Task | Files Touched | Component | Layer | Size | Dependencies |
|---|------|---------------|-----------|-------|------|-------------|
| T C.1 | `CaptureWriter` (image-then-sidecar, run-scoped dir, `image_path` return) | `patrol_perception/capture_writer.py` (new) | CaptureWriter | Persistence | M | M6.B |
| T C.2 | Wire writer into CaptureCoordinator; set `image_path` in the published message | `patrol_perception/coordinator.py` (modify) | CaptureCoordinator | Node | S | T C.1 |
| T C.3 | Populate `metadata` (mission id, waypoint index, detection confidence — PCAP-6/PCAP-7) into message + sidecar | `patrol_perception/capture_builder.py` (modify); `patrol_perception/coordinator.py` (modify) | CheckpointCaptureBuilder | Domain/Logic | M | T C.2 |
| T C.4 | Unit tests: writer round-trip + sidecar↔message consistency | `tests/unit/test_capture_writer.py` (new) | CaptureWriter | Persistence | S | T C.1 |

##### Testing
| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | writer in tmp dir (no ROS) | image+sidecar pair written; `image_path` consistent; metadata round-trips (PCAP-6) |
| Integration (downstream) | full patrol over M5 world | directory of images+sidecars matches checkpoints traversed (AC-1) |

##### Documentation
| Artifact | Audience | Content |
|----------|----------|---------|
| On-disk artifact layout doc | 05 / analysts | `<output_root>/<run>/NNN_<checkpoint_id>.{png,json}` + sidecar schema, alignment with 05 output |

### 6.3 Layered Delivery Sequence

**Skeleton + layering rationale:**
1. **M6.A (skeleton)** is the thinnest end-to-end slice that crosses the Interface/Message + Domain layers: the durable `CheckpointCapture` type compiles and the ROS-free core constructs it, validated by a sub-second unit suite. After M6.A a reviewer can build the workspace and see the contract + green tests. This puts the *highest-risk, hardest-to-reverse* element (the schema) first — the PRD's central tenet.
2. **M6.B (layer 1: live capture)** thickens the skeleton with the Node layer: real camera/detection/trigger wiring publishing live `/patrol/checkpoint_capture` messages in a SITL patrol. After M6.B the demo shows one tagged message per checkpoint live. Why next: it's the first point the contract is exercised against real upstream topics (03/02), surfacing integration risk early.
3. **M6.C (layer 2: on-disk artifact)** adds the Persistence layer: inspectable images + sidecars and full metadata. After M6.C the demo adds a browsable output directory consistent with the topic.

**What gets demoable, when:**
- After M6.A: build the message + run the green unit suite.
- After M6.B: M6.A + live tagged messages during a patrol.
- After M6.C: M6.B + a browsable directory of per-checkpoint images + sidecars.

**Scope-shedding plan:** if schedule slips, shed M6.C (persistence) — the live topic alone (M6.B) still lets 05 record captures and is demoable. M6.A alone (the compiled, tested contract) is still a shippable artifact and unblocks 05's schema binding. **Hard floor:** the `CheckpointCapture` schema (M6.A) cannot be shed — it is the exit-item-11 deliverable everything downstream binds to.

**Parallel work opportunities:** within M6.A, T A.2/T A.3 (config/resolver) and T A.4 (builder) can proceed in parallel once T A.1 (the .msg) lands. M6.B's apriltag_ros verification (T B.1) can run concurrently with M6.A. M6.B and M6.C are sequential (C depends on B's coordinator).

### 6.4 Definition of Done

A milestone is complete when:
- [ ] All tasks implemented and code-reviewed (solo-dev: self-review + PR gate on `main`).
- [ ] Specified tests pass (unit per-PR; integration downstream where noted).
- [ ] **Shippable demo runs** (M6.A: build + `ros2 interface show` + green unit suite; M6.B: live `ros2 topic echo`; M6.C: browsable output dir).
- [ ] Documentation artifacts written (contract note / param reference / artifact-layout doc).
- [ ] No P1 bugs remain.
- [ ] Systemic interfaces integrated per §4.4 (rosout logging on capture/skip/failure; config fail-fast; persistence failure handling; latch-only-on-success invariant per §4.2.8/§4.4.6).

---

## 7. Changelog

### v0.2.0 — 2026-06-03
**Self-review revision (software-design ReviewDesign + ReviseDesign, auto-pilot).** Review scored D1–D13 with no finding at or above the medium floor that regenerates; the design was already Strong across the rubric. Applied two low-cost clarifications surfaced by the D11/D2 traceability and D6 consistency checks:

**Topics:**
- D11/D2 traceability: surface OQ-5 (checkpoint-schema confirmation) as an explicit *soft gate* on M6.A task T A.2 (`CheckpointConfigLoader`), since the loader is built against the still-pending settled-default `checkpoints.yaml` schema.
- D6/§4.2.8 consistency: make the **latch-only-on-success** invariant explicit in `CaptureCoordinator` and reconcile it across the three §4.4.5 skip rows (no-frame / no-tag-in-view / unmapped-tag_id), so AC-6 (≤1 deterministic capture) and AC-4 (no fabricated `checkpoint_id`) are visibly consistent.

**Codebase drift:** None (greenfield; v0.1.0 → v0.2.0 same session).

**Sections modified:**
- §2: OQ-1/OQ-5/OQ-8 rationales tightened with sibling-DoD citations; OQ-5 marked soft-gate on T A.2.
- §3.2: VP-3/VP-4/VP-5/VP-6 citations made specific to the sibling DoD sections / README ownership matrix.
- §4.2.5: noted the config-loader re-check trigger if OQ-5's confirmed schema differs.
- §4.2.8: latch-only-on-success made explicit; reconciled with AC-4.
- §4.2.9: added a one-line note that all six node params are internal config seams tracing to a cross-docset interface or ADR (PRD-trace audit, reverse direction).
- §4.4.5: unmapped-tag_id and no-tag-in-view rows annotated "latch NOT set"; added §4.4.6 completeness note tying the three skip rows to one invariant.
- §4.5 Sequence 1: latch annotated "only after success".
- §5 Q5/Q7: added the coordinator-latch unit scenario and the OQ-5 soft-gate to the risk list.
- §6.1/§6.2: M6.A dependency + OQ-5 soft-gate note; M6.B dependency adds "OQ-5 confirmation preferred"; T B.3 task note "latch only on success".

**Key decisions:** No new design decisions; settled cross-docset defaults (OQ-1 `image_path` + separate CompressedImage live topic; OQ-5 `checkpoint_id` via 03's `sim/config/checkpoints.yaml`) remain Provisional confirmed at combined review (2026-06-03). OQ-8 (CompressedImage live-frame-topic owner) remains an unowned cross-docset item deferred to the human's combined review.

### v0.1.0 — 2026-06-03
**Initial version** — Created via CreateDesign workflow from `04-perception/prd.md` rev 2. Adopts the settled cross-docset contract defaults (OQ-1 `image_path` + separate CompressedImage live topic; OQ-5 `checkpoint_id` via 03's `sim/config/checkpoints.yaml`) flagged confirmed at combined review (2026-06-03). Resolves design-internal OQs (OQ-2 metadata=`diagnostic_msgs/KeyValue[]`; OQ-3 explicit-trigger-gated-by-tag; OQ-4 run-scoped dir + JSON sidecar; OQ-7 latest-frame + explicit world/ENU frame). Records OQ-6/OQ-9 (apriltag_ros + cv_bridge on Jazzy) as Needs-Input verification gates before Phase B, and OQ-8 (CompressedImage live-frame topic owner) as an unowned cross-docset item deferred to human.

---

## Appendix B: User Acceptance Criteria (carried from PRD for traceability)

**UAC-PCAP-1** — GIVEN the M5 world (3+ AprilTags) + a running patrol with exactly one checkpoint in FOV, WHEN the patrol reaches that checkpoint and the trigger fires, THEN exactly one capture is produced (no dup/drop), sampled from the 03 camera topic. *(→ CaptureCoordinator, FrameSampler; AC-6 latch §4.2.8)*

**UAC-PCAP-2** — GIVEN a checkpoint AprilTag in view + the shared 03 config, WHEN the node captures, THEN `checkpoint_id` traces to a detected `tag_id` mapped through the config, not a hardcoded constant. *(→ CheckpointResolver §4.2.5; AC-4)*

**UAC-PCAP-3** — GIVEN a running patrol producing captures, WHEN each checkpoint is reached, THEN a `/patrol/checkpoint_capture` message of type `patrol_interfaces/msg/CheckpointCapture` is published in real time carrying the same `(image-or-path, checkpoint_id, pose, timestamp)` as the on-disk artifact, consumable by 05 via the identical compiled type. *(→ CapturePublisher, CheckpointCaptureBuilder §4.2.4/4.2.9)*

**UAC-PCAP-4** — GIVEN `patrol_interfaces`, WHEN built with `colcon build`, THEN `CheckpointCapture` exists with `header`/`checkpoint_id` (string)/`pose` (PoseStamped)/image field (`string image_path`)/`metadata`; AND the same compiled type is published by 04 and recorded by 05. *(→ CheckpointCapture.msg §4.2.3; AC-3/AC-7)*

**UAC-PCAP-5** — GIVEN the M5 world + a patrol, WHEN the mission completes, THEN the output directory contains one image + one sidecar per checkpoint, and each `image_path` matches the data in its `/patrol/checkpoint_capture` message. *(→ CaptureWriter §4.2.6; AC-1)*
