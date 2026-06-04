# Mission Control — Patrol Mission Orchestration (Phase 1)

> **One-liner:** A Python ROS 2 node and ROS-free state machine that arm a SITL drone, fly a YAML-configured multi-waypoint patrol with per-waypoint dwell over PX4 offboard control, return home, land, and abort safely — the executable heart of Phase 1.

**Date:** 2026-06-03
**Status:** Draft
**Owner:** Project owner (solo dev)
**DRI:** jxstanford@wemodulate.energy
**Docset:** docs/phase1/02-mission-control (2 of 5) · **Milestones:** M3–M4
**Source of truth:** docs/phase1/02-mission-control/dod.md · docs/phase1_simulation_plan.md
**PRD tier:** Standard (Complexity=complex × Risk=medium → Standard; Scope=cross-service pulls in the Cross-Service Impact section)

## Overview

Phase 1 stands up a pre-hardware simulation of an autonomous patrol drone. The platform docset (01) already delivers PX4 SITL in Gazebo Harmonic with live `/fmu/*` telemetry over a native uXRCE-DDS bridge. This docset delivers the mission-orchestration layer on top of that substrate: a `patrol_mission` ROS 2 node that commands PX4 offboard control, driven by a separately-testable `MissionStateMachine` plain-Python class that arms, takes off, flies a configured multi-waypoint patrol with per-waypoint dwell, returns home, lands, and aborts on low-battery or an external signal.

This is the capability the single Phase-1 exit command (`ros2 launch patrol_bringup mission_patrol.launch.py`) drives end-to-end. It owns contracts that three sibling docsets bind to: the mission/route YAML schema and the launch entry-points (consumed by 03 and 05), the mission-orchestration ROS topics (recorded by 05), the checkpoint-arrival capture-trigger semantic (consumed by 04), and the `MissionStateMachine` class contract that Phase 2+ flight bring-up reuses unchanged. Because the abort/fly-away safety logic is the highest-consequence code in Phase 1, it is exercised in SITL precisely so the risk is bought down before it can ever reach hardware.

## Problem Statement

> **When** an operator wants the simulated drone to fly a repeatable patrol over a set of checkpoints,
> **they struggle with** the fact that nothing in the stack commands programmatic flight — there is only manual QGroundControl stick input, no waypoint sequencing, no dwell, no return-to-home, and no abort path,
> **which means** Phase 1's integrative goal (a full patrol with per-checkpoint image capture, recorded to a bag) is unreachable, and the safety-critical abort logic that prevents real-world fly-aways never gets exercised before hardware.

Today the only way to move the SITL drone is by hand in QGC. A hardcoded scripted flight would not represent a reviewable, versioned patrol route, would offer no way to verify a one-line transition change without a slow flaky full-stack run, and would defer the abort path — the classic source of fly-aways — to "later," which the plan explicitly warns against ("if it doesn't work in sim it won't work in flight"). This matters now because M3–M4 gate every downstream docset: the sim world (03), perception capture (04), and the logging pipeline (05) all need a real patrol to fly before they can be exercised end-to-end.

## Goals

### Business goals
- Unblock the Phase 1 integrative exit (full patrol run, item 1) and the M3/M4 milestones so 03/04/05 can be exercised against a real patrol.
- Buy down fly-away safety risk in sim before any hardware exists, so the same state-machine contract carries into Phase 2 flight bring-up unchanged.

### User goals
- The operator flies a takeoff/hover/land and a multi-waypoint patrol from a single launch command, editing routes in a versioned YAML file rather than code.
- The developer validates every mission transition (including all abort paths) in a ROS-free unit suite that runs in under 5 s, and trusts a small strict SITL integration test to catch end-to-end regressions.

### Non-goals
- VIO / SLAM localization — SITL provides ground-truth pose; offboard uses NED from the EKF origin (Phase 3/4).
- Real object/anomaly detection in the mission loop — mission only triggers capture; detection is owned by 04 (Phase 3/6).
- Bag recording mechanics, the world/checkpoints/camera, and the `CheckpointCapture` message — owned by 05, 03, and 04 respectively this Phase.

## Out of Scope

> Items explicitly **not** part of this Phase-1 MVP. Each entry has a status, rationale, and target. This section is the contract-level commitment that prevents scope creep downstream.

| Item | Status | Rationale | Target | Added |
|------|--------|-----------|--------|-------|
| VIO / SLAM localization | Deferred | SITL provides ground-truth pose; offboard uses NED relative to the EKF origin. Would come in when real sensors replace ground truth. | Phase 3/4 | 2026-06-03 |
| Real object/anomaly detection in the mission loop | Out of scope | AprilTag scaffolding is enough; the mission only triggers capture. Detection is owned by 04-perception. | Phase 3/6 (04) | 2026-06-03 |
| EKF2 / VIO parameter tuning; MAVROS-vs-DDS relitigation | Out of scope | uXRCE-DDS is the settled call (ADR-0001); tuning needs real sensors. | Phase 3/4 | 2026-06-03 |
| Manual-takeover abort *trigger* in SITL | Deferred | The state-machine transition exists (MC-11), but the plan only requires low-battery + external-signal to be *triggerable* in sim; manual takeover needs hardware RC. | Phase 2+ (hardware RC) | 2026-06-03 |
| Bag recording mechanics, upload, manifest, Foxglove | Out of scope | The mission launch *invokes* the recorder, but the wrapper/pipeline is owned by 05-logging-replay. | Phase 1 (05) | 2026-06-03 |
| The world, checkpoint models, RGB camera topic | Out of scope | Owned by 03-sim-environment; this docset consumes the checkpoint positions it publishes. | Phase 1 (03) | 2026-06-03 |
| Mission-replan / dynamic re-routing mid-flight [INFERRED] | Out of scope | Phase 1 flies a static YAML route start-to-finish; dynamic replanning is not a Phase 1 capability. Inferred fence (not an explicit DoD deferral). | TBD (later phase) | 2026-06-03 |

## Key Hypotheses

- **H1:** We believe extracting transition logic into a ROS-free `MissionStateMachine` class will let the developer validate every transition (including all abort paths) in a fast unit suite *because* the precondition for London-style TDD is that the logic is decoupled from rclpy/SITL. *Signal: the state-machine unit suite passes at ≥85% coverage in <5 s with no ROS/Gazebo/PX4 process started (AC-4).*
- **H2:** We believe a tolerance-plus-hold-time waypoint-completion criterion will advance the mission reliably *because* exact float/setpoint equality never fires, mirroring how hardware behaves. *Signal: the patrol visits all configured waypoints in order and completes in SITL without stalling at a waypoint (AC-2, AC-9).*
- **H3:** We believe exercising abort transitions in SITL now will surface fly-away bugs before hardware *because* "if it doesn't work in sim it won't work in flight." *Signal: an external-signal abort mid-patrol drives an observable return-home transition in a SITL run (AC-6), and all abort paths are unit-covered (AC-8).*

## Tenets

> Decision tie-breakers when implementation is ambiguous — *unless you know better ones.*

1. **Sim is the proving ground for safety, not a toy.** When a choice trades sim convenience against fidelity to how hardware will behave, lean toward hardware-faithful behavior (tolerance+hold completion, abort paths present from day one).
2. **The state machine is ROS-free.** When tempted to read a topic or call rclpy from transition logic, stop — plumbing lives in the node, decisions live in the class. This is the precondition for fast tests.
3. **Config over code for anything an operator changes.** Routes, tolerances, dwell, and thresholds live in versioned YAML, never hardcoded.
4. **One explicit frame boundary.** All world/YAML→NED conversion happens at exactly one named place a reviewer can point to; silent frame mistakes are "silent and infuriating."
5. **Keep the slow tier small and strict.** SITL integration is the flaky/expensive lane; prefer one canonical strict test over many brittle ones, and never make SITL a per-PR gate.

## Functional Requirements

> The FR table below is the **contract** for this PRD (see Scope Authority). Each P1 FR pairs with a UAC in Appendix B. FR IDs use the `MC-` (mission-control) prefix.

### P1: Critical (must ship)

#### MC-1: Single-command arm + takeoff + hover + land
WHEN the operator runs `ros2 launch patrol_bringup mission_basic.launch.py` against a running SITL drone, the system SHALL arm the vehicle, take off to 5 m AGL, hold for 10 s, and land, via PX4 offboard control over uXRCE-DDS.

**Customer scenario:** The operator validates programmatic flight end-to-end before any patrol logic, watching the SITL drone arm, climb to 5 m, hover 10 s, and land from one command.

**Pain removed:** Without an offboard mission node there is no programmatic flight at all — only manual QGC stick input. This is the thin walking-skeleton flight everything else builds on.

**Acceptance criteria:**
- The launch file `patrol_bringup/mission_basic.launch.py` exists and starts the `patrol_mission` node.
- The drone arms, reaches ~5 m AGL, holds ~10 s, and lands, observable in SITL.
- Offboard setpoints/commands are published on `/fmu/in/*`; no MAVROS/MAVSDK layer is used.

**Trace:** UAC-MC-1 (Appendix B) · AC-1 · M3 Exit

#### MC-2: Multi-waypoint patrol with dwell and return-to-home
WHEN the operator runs `ros2 launch patrol_bringup mission_patrol.launch.py` against SITL with a patrol YAML of 4+ waypoints, the system SHALL fly to each waypoint in configured order, dwell the configured time at each, return home, and land.

**Customer scenario:** The operator defines 4+ waypoints in YAML and the drone visits each in order, dwells, returns home, and lands — the Phase-1 exit command.

**Pain removed:** A fixed/hardcoded path can't represent a real patrol route and can't be changed without a code edit; this is the patrol-flight portion of the integrative exit (item 1).

**Acceptance criteria:**
- The launch file `patrol_bringup/mission_patrol.launch.py` exists and drives the full patrol.
- The drone visits every configured waypoint in YAML order, dwelling the configured time at each.
- After the last waypoint the drone returns home and lands, observable in SITL.

**Trace:** UAC-MC-2 (Appendix B) · AC-2 · M4 Exit; exit-checklist item 1 (mission-flight portion)

#### MC-3: Mission configuration loaded from a repo-checked-in YAML file
The system SHALL load the patrol route and mission parameters from a YAML file checked into the repository; no route data SHALL be hardcoded in source.

**Customer scenario:** The operator edits `patrol_mission.yaml`, commits it, and the same launch command flies the new route.

**Pain removed:** Routes embedded in code are unreviewable and un-versioned; YAML makes a route a reviewable, diffable artifact.

**Acceptance criteria:**
- On run start, the mission config (waypoint list, dwell, completion tolerance+hold, abort thresholds) is read from a checked-in YAML file.
- No waypoint/route data is present in Python source.
- The YAML declares the frame of each waypoint (see MC-7).

**Trace:** UAC-MC-3 (Appendix B) · AC-3 · exit-checklist item 2

#### MC-4: ROS-free, separately-testable `MissionStateMachine`
The system SHALL implement all mission transition logic in a plain-Python `MissionStateMachine` class decoupled from rclpy, exposing a `tick(current_state, telemetry) -> command` contract, covering arm → takeoff → waypoint sequence (with dwell) → return-to-home → land plus abort transitions.

**Customer scenario:** The developer runs `pytest` and validates every mission transition in <5 s without launching ROS, Gazebo, or PX4.

**Pain removed:** Embedding logic in the node would force slow, flaky SITL runs to verify a one-line transition change; the ROS-free class is the precondition for London-style TDD and is reused unchanged in Phase 2+.

**Acceptance criteria:**
- The `MissionStateMachine` class is importable and exercisable without importing rclpy or starting any ROS/Gazebo/PX4 process.
- The unit suite covers every state and transition and passes at ≥85% coverage (CI floor per ADR-0002; DoD AC-4 states >80% with ≥85% as the enforced gate), completing in <5 s.
- The PX4 interface is mocked in unit tests; the simulator is not mocked (settled constraint).

**Trace:** UAC-MC-4 (Appendix B) · AC-4 · M3 Exit; exit-checklist item 3; ADR-0002

#### MC-5: Robust waypoint-completion criterion (tolerance + hold time)
WHILE flying toward a waypoint, the system SHALL mark the waypoint complete and advance only after the vehicle remains within the configured position tolerance for the configured hold time; it SHALL NOT advance on exact position equality.

**Customer scenario:** The drone advances to the next waypoint only after holding within tolerance for a set time, mirroring how hardware behaves.

**Pain removed:** Exact `position == target` checks never fire on floats or real setpoints, stalling the mission indefinitely.

**Acceptance criteria:**
- A waypoint is marked complete iff the vehicle is within the configured tolerance continuously for the configured hold time.
- The criterion is parameterized from YAML (MC-3), not hardcoded.
- A unit test demonstrates the criterion advances on hold-within-tolerance and never on exact equality.

**Trace:** UAC-MC-5 (Appendix B) · AC-9 · M4 waypoint-completion criterion

#### MC-6: Mission abort — low-battery and external-signal
WHEN an abort is requested via the external-signal ROS topic, OR WHEN the battery telemetry crosses the configured low-battery threshold, the system SHALL transition the state machine to abort and return the drone home; the external-signal transition SHALL be observable in a SITL run.

**Customer scenario:** The operator publishes an abort signal (or battery crosses threshold) mid-patrol and the drone returns home; the transition is observable in a SITL run.

**Pain removed:** An abort path bolted on later is the classic source of fly-aways — "if it doesn't work in sim it won't work in flight."

**Acceptance criteria:**
- Publishing the abort signal on the external-signal topic mid-patrol drives an observable abort→return-home transition in SITL.
- A battery telemetry value crossing the configured threshold drives an abort transition (unit-tested per MC-9 / AC-7).
- Abort threshold and the battery telemetry source are configured, not hardcoded (see OQ-6).

**Trace:** UAC-MC-6 (Appendix B) · AC-6, AC-7 · M4 Exit; exit-checklist item 12

#### MC-7: Explicit coordinate-frame discipline
The system SHALL convert world/YAML waypoint coordinates to PX4 NED relative to the EKF origin at exactly one explicit, reviewable boundary; each waypoint SHALL declare its frame in YAML.

**Customer scenario:** A reviewer reads the mission node and can point to exactly where world/YAML coordinates convert to PX4 NED-relative-to-EKF-origin.

**Pain removed:** Silent frame mistakes are "silent and infuriating" and surface only in misflown missions.

**Acceptance criteria:**
- Waypoints declare their frame in YAML (MC-3).
- All frame conversion happens at one named function/boundary, not scattered through the node.
- A unit test exercises the conversion with a known input→NED output.

**Trace:** UAC-MC-7 (Appendix B) · M4 coordinate-frames

#### MC-8: Mission-orchestration ROS topics published for downstream consumers
The system SHALL publish mission state, current/target waypoint, and the checkpoint-arrival capture-trigger semantic, and SHALL subscribe to an abort-signal topic; these topics SHALL be recordable by 05-logging-replay and renderable in Foxglove.

**Customer scenario:** The logging pipeline (05) records `/patrol/mission_state`, `/patrol/current_waypoint`, and `/patrol/abort`, and the perception node (04) keys image capture off the "arrived and dwelling at checkpoint N" signal.

**Pain removed:** Without published mission state, 05 has nothing to record and 04 has no trigger; downstream docsets would each invent an incompatible signal.

**Acceptance criteria:**
- `/patrol/mission_state` and `/patrol/current_waypoint` are published during a run; `/patrol/abort` is subscribed (and drives MC-6).
- A checkpoint-arrival/"dwelling at checkpoint N" signal is emitted that 04 can key capture off of (trigger contract finalized jointly with 04 — see OQ-7).
- Topic types/QoS are chosen so 05 can record them and Foxglove can render them (see OQ-3).

**Trace:** UAC-MC-8 (Appendix B) · exit-checklist items 8, 12; consumed by 03/04/05

#### MC-9: Abort logic unit-covered
WHEN the unit suite runs, the system SHALL have unit tests covering all abort transitions — low-battery, external-signal, and the scaffolded triggers (MC-11).

**Customer scenario:** The developer changes abort logic and the unit suite proves every abort path still transitions correctly, without a SITL run.

**Pain removed:** Abort paths that are only ever exercised in slow flaky SITL runs rot silently; unit coverage makes regressions cheap to catch.

**Acceptance criteria:**
- Low-battery, external-signal, and scaffolded-trigger aborts each have at least one unit test asserting the transition.
- These tests run in the ROS-free suite (MC-4) and contribute to the ≥85% coverage gate.

**Trace:** UAC-MC-9 (Appendix B) · AC-7, AC-8 · M4 Exit; exit-checklist item 12

#### MC-10: SITL integration test in CI
The system SHALL provide at least one integration test that spins up SITL, runs a canonical mission via the launch file, and passes in CI as the slow/flaky tier (not a required per-PR gate).

**Customer scenario:** A change that breaks the end-to-end offboard flight is caught by a canonical SITL mission test rather than only by a manual run.

**Pain removed:** Without an integration test the offboard control loop and launch wiring are only ever validated by hand, so regressions reach downstream docsets silently.

**Acceptance criteria:**
- ≥1 integration test launches SITL, runs a canonical mission via `mission_*.launch.py`, and asserts success.
- It runs in CI's slow tier per ADR-0002 (SITL is not a required per-PR check); the canonical scenario(s) and runtime/flakiness budget are finalized in design (OQ-5).

**Trace:** UAC-MC-10 (Appendix B) · AC-5 · M3/M4 Exit; exit-checklist item 4

### P2: Important (should ship)

#### MC-11: Abort-state scaffolding for triggers not exercisable in SITL
The system SHALL include abort state transitions for triggers that cannot be exercised in SITL (e.g., manual takeover, timeout) so a Phase-2 hardware trigger wires into an existing state rather than requiring a redesign.

**Customer scenario:** The developer adds the hardware-triggered manual-takeover abort in Phase 2 by wiring an existing state, not redesigning the machine.

**Acceptance criteria:**
- The state machine defines abort transitions for the non-SITL triggers (manual takeover, timeout) even though they are not fired in Phase 1 SITL.
- These scaffolded transitions are covered by unit tests (MC-9).

**Trace:** AC-8 (scaffolded-trigger coverage) · plan M4 "even if the conditions can't be triggered in SITL, the state transitions should be there"

## Scope Authority

The FR table above is the **contract** for this PRD. The design document (`docs/phase1/02-mission-control/design.md` — to be added when the design is created) realizes these FRs as components, sequences, and milestone tasks.

**The design must not introduce surface area beyond this PRD's FR table without a corresponding PRD revision.** If the design proposes a new mission topic, YAML field, launch entry-point, or state-machine method not authorized by an FR, the PRD must be updated first — adding the FR through the PRD's revision flow.

Conversely, **this PRD must not specify implementation detail beyond the FR shape.** The choice of state-machine library, concrete topic types/QoS, tolerance/dwell default values, return-to-home mechanism, and the canonical integration scenario belong in the design (and are tracked as Open Questions), not here.

This discipline keeps the design honest and the PRD lean.

## Success Metrics

| Metric | Baseline (current) | Target | How Measured | Timeline |
|--------|-------------------|--------|--------------|----------|
| State-machine unit suite runtime | N/A (new) | <5 s, no ROS/Gazebo/PX4 process | `pytest` wall-clock on the pure-Python runner | M3 |
| Mission state-machine coverage | N/A (new) | ≥85% (CI gate; DoD AC-4 floor >80%) | `coverage` on the unit suite in CI Layer A | M3 |
| Basic mission success (arm→takeoff→hover→land) | N/A (new) | Passes in SITL via `mission_basic.launch.py` | Manual + integration run, observed states | M3 |
| Patrol completion (4+ waypoints, dwell, RTH, land) | N/A (new) | All waypoints visited in order, returns home, lands | SITL run via `mission_patrol.launch.py` | M4 |
| External-signal abort observable in SITL | N/A (new) | Abort→return-home transition observed mid-patrol | SITL run + recorded `/patrol/*` topics | M4 |
| SITL integration test in CI | N/A (new) | ≥1 canonical mission test passes; runtime within agreed budget | CI slow tier per ADR-0002 | M4 |

## Technical Considerations

### Integration points
- **Consumes from 01-platform:** `/fmu/out/*` (notably `vehicle_local_position`, battery state), `/fmu/in/*` offboard setpoint/command topics, `px4_msgs`, ROS 2 Jazzy + uXRCE-DDS agent + container build.
- **Consumes from 03-sim-environment:** checkpoint world + checkpoint positions (the shared checkpoint mapping) + RGB camera topic; the mission flies to those positions.
- **Consumes from 05-logging-replay:** the bag-recorder launch include/wrapper invoked at mission start from `mission_patrol.launch.py`.
- **Provides to 03/04/05:** mission/route YAML schema, `mission_*.launch.py` entry-points, `/patrol/{mission_state,current_waypoint,abort}`, the checkpoint-arrival capture-trigger semantic, and the `MissionStateMachine` contract.

### Checkpoint mapping contract (settled default — confirmed at combined review (2026-06-03))
The mission reads checkpoint positions from a single shared YAML `sim/config/checkpoints.yaml`, **owned by 03-sim-environment**, a list of `{checkpoint_id: string, position: {x,y,z} in the world/ENU frame, tag_family: string, tag_id: int}`. This docset (02) reads the `position` entries to build waypoints (converting world/ENU → PX4 NED at the MC-7 boundary). 03 places the AprilTag models + camera from the same file; 04 maps a detected `tag_id` → `checkpoint_id`. The mission YAML (MC-3) references checkpoint IDs from this shared file rather than duplicating positions. *This default is recorded as OQ-2 / OQ-7 pending the human's combined cross-docset review.*

### Data storage
- Mission/route configuration is a repo-checked-in YAML file (MC-3); no runtime database. No persistent state beyond the bag produced by 05.

### Scalability
- Phase 1 scale: a single drone, a handful of waypoints, one SITL instance. No multi-vehicle or high-rate concerns; offboard setpoints are published at the rate PX4 offboard requires (a design detail).

### Rabbit holes
> Areas that look simple but could explode in scope.

- **PX4 offboard mode entry/keepalive timing.** Offboard requires a continuous setpoint stream before and during the mode switch or PX4 rejects/exits offboard. *Containment:* prove the arm→offboard→takeoff handshake in MC-1's walking skeleton first; treat keepalive as a node-plumbing concern, kept out of the state machine.
- **SITL integration-test flakiness and runtime.** The plan's self-identified least-confident area. *Containment:* keep the canonical mission small and strict, gate it in the slow tier only (ADR-0002), and finalize the runtime/flakiness budget in design (OQ-5) rather than expanding the test set.
- **Return-to-home semantics.** PX4 RTL mode vs an explicit home-waypoint offboard sequence behave differently around mode handoff and landing. *Containment:* decide one mechanism in design (OQ-8) and keep it behind the same RTH state the machine already models.
- **Coordinate-frame conversion sprawl.** Easy to sprinkle NED math through the node. *Containment:* MC-7 forces one named boundary; reject any review that adds a second conversion site.
- **Low-battery trigger fidelity in SITL.** SITL battery modeling may not deplete realistically, so the trigger may not be naturally exercisable. *Containment:* unit-test the threshold transition (MC-9/AC-7) and treat the SITL-observable half as best-effort; pick the telemetry field/threshold in design (OQ-6).

### Potential challenges
- The offboard control loop and abort semantics are the highest-consequence logic in Phase 1; mitigation is to exercise them in SITL now and keep all decision logic in the unit-tested ROS-free class.
- Three sibling docsets and Phase 2+ consume this docset's contracts, so topic-name/YAML-schema/trigger changes cascade; mitigation is to settle the cross-docset contracts (OQ-2, OQ-3, OQ-7) early and treat the FR table as the contract.

## Cross-Service Impact

> Scope is cross-service: this docset owns contracts consumed by three sibling docsets and reused by Phase 2+.

### Affected services (docsets)

| Docset | Impact | Changes required |
|--------|--------|-----------------|
| 01-platform | Consumed only | None — 02 binds to the existing `/fmu/*` surface, `px4_msgs`, and the `patrol_bringup`/`patrol_interfaces` package shells 01 created. |
| 03-sim-environment | Reads waypoints from the shared checkpoint mapping; patrol exercises its world | 03 must place AprilTags/camera from the same `sim/config/checkpoints.yaml` whose `position` entries 02 turns into waypoints. |
| 04-perception | Keys capture off the checkpoint-arrival trigger this docset emits | 02 must emit a "arrived and dwelling at checkpoint N" signal; the explicit-signal-vs-inferred trigger contract is finalized jointly (OQ-7). |
| 05-logging-replay | Records `/patrol/{mission_state,current_waypoint,abort}` and is invoked by `mission_patrol.launch.py` | 02's launch file includes 05's recorder wrapper; topic types must be Foxglove-renderable so 05's bag is usable. |

### Interface changes
- New owned contracts (no prior versions to break): mission/route YAML schema; `/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/abort`; the `MissionStateMachine` `tick(current_state, telemetry) -> command` contract; the checkpoint-arrival capture-trigger semantic; `mission_basic.launch.py` and `mission_patrol.launch.py`.
- Consumed contract default (pending confirmation): checkpoint mapping `sim/config/checkpoints.yaml` owned by 03 (see OQ-2).

### Deployment coordination
- Build/dependency order is `01 → 02 → 03 → 04 → 05`. This docset stands on 01 (must be green first). 03/04/05 stand on the contracts here; the YAML schema, mission topics, and trigger semantic should be settled before 03/04/05 finalize their consuming designs.
- The mission topics and YAML schema must be stable before 05 records them and before 04 binds its capture trigger; changes after that cascade.

### Testing implications
- Contract-level: the mission topics must be recordable by 05 and renderable in Foxglove (item 8); the checkpoint-arrival trigger must satisfy 04's capture cardinality (one capture per checkpoint visit, 04 AC-6).
- End-to-end: the integrative exit (item 1) is a single patrol run that this docset drives and 03/04/05 all participate in; the canonical SITL integration test (MC-10) is the in-CI proxy for that path.

## Alternatives Considered

> The cross-docset contracts and the relitigated stack decisions are settled (DoD §6); this section records the decisive trade-offs so reviewers see the solution space was explored.

### Option 1: ROS-free `MissionStateMachine` class + thin rclpy node (selected)
Transition logic in a plain-Python class (`tick(state, telemetry) -> command`); the node owns only ROS plumbing (pub/sub, offboard keepalive, frame conversion).

**Pros:**
- Every transition (including all abort paths) is validated in <5 s without ROS/Gazebo/PX4 (H1) — the precondition for London-style TDD.
- The same class is reused unchanged in Phase 2+ flight bring-up.

**Cons / Trade-offs accepted:**
- A small plumbing/decision boundary must be designed and respected; accepted because the testability and reuse payoff is exactly the Phase-1 thesis ("buy down risk in sim").

### Option 2: Mission logic embedded in the rclpy node
Put state and transitions directly in the ROS node.

**Pros:**
- Fewer moving parts to wire up initially.

**Cons:**
- Verifying a one-line transition change requires a slow, flaky full-stack SITL run; abort-path coverage becomes impractical.

**Why not chosen:** It violates the settled "state machine as a separate class" constraint (DoD §6) and forecloses fast, reliable abort-path testing — the decisive factor given fly-away safety is the highest-consequence Phase-1 logic.

### Option 3: MAVROS / MAVSDK offboard instead of native uXRCE-DDS
Drive offboard through a MAVLink translation layer.

**Pros:**
- More mature high-level mission abstractions exist in MAVSDK.

**Cons:**
- Adds a translation layer the project has explicitly excluded; diverges from the native `/fmu/*` surface every sibling consumes.

**Why not chosen:** uXRCE-DDS native is settled (ADR-0001, plan "Target stack"); this is a do-not-relitigate constraint.

### Option 4: Do nothing / status quo
Rely on manual QGroundControl stick input.

**Why not acceptable:** There is no programmatic patrol, no dwell, no return-to-home, and no abort path — Phase 1's integrative exit (item 1) and M3/M4 are unreachable, and the safety-critical abort logic never gets exercised before hardware.

## Milestones

### Phase 1: M3 — Python mission node, takeoff and land
- `MissionStateMachine` class with arm→takeoff→hover→land states and the ROS-free unit suite (MC-4) at ≥85% coverage in <5 s.
- `patrol_mission` node + `mission_basic.launch.py` driving offboard arm/takeoff/hover/land in SITL (MC-1).
- One SITL integration test scaffold for the basic mission (MC-10, basic scenario).
- **Validation:** AC-1, AC-4, AC-5 (basic) pass; exit-checklist items 3, 4 satisfied for the basic mission.

### Phase 2: M4 — Multi-waypoint patrol mission
- YAML mission schema (MC-3), waypoint sequencing with dwell, frame conversion at one boundary (MC-7), tolerance+hold completion (MC-5), return-to-home + land (MC-2).
- Abort paths: external-signal + low-battery transitions and scaffolded triggers (MC-6, MC-9, MC-11); `/patrol/*` topics published (MC-8).
- Canonical patrol SITL integration test (MC-10, patrol scenario).
- **Validation:** AC-2, AC-3, AC-6, AC-7, AC-8, AC-9 pass; exit-checklist items 2, 12 satisfied; mission-flight portion of item 1 demonstrated.

## Open Questions

> Decisions handed to design via /drive. The two cross-docset contract defaults (OQ-2, OQ-7) carry the settled run-policy defaults and are flagged **confirmed at combined review (2026-06-03)** for the human's combined review — they are not silently invented.

| # | Question | Status | Decision target | Rationale (why open / what would resolve it) |
|---|----------|--------|-----------------|----------------------------------------------|
| OQ-1 | State-machine library: `transitions` vs `python-statemachine` vs hand-rolled? | Open | Design | Plan explicitly flags this open ("strong preferences welcome"). Resolved by a design spike weighing testability/clarity vs dependency weight. |
| OQ-2 | Checkpoint mapping schema/location shared with 03/04. | Resolved (combined review 2026-06-03) | Design (joint with 03, 04) | **Default:** a single shared YAML `sim/config/checkpoints.yaml` owned by 03, `{checkpoint_id, position {x,y,z} world/ENU, tag_family, tag_id}`; 02 reads `position` to build waypoints. Resolved when the human confirms the shared file in the combined cross-docset review. |
| OQ-3 | Mission topic names, types, and QoS (`patrol_interfaces` custom vs `std_msgs`). | Open | Design | Recorded by 05 and must be Foxglove-renderable (item 8). Resolved by choosing types that round-trip through MCAP and render in Foxglove. |
| OQ-4 | Waypoint-completion tolerance and hold-time defaults (plan illustrates 0.5 m / 2 s). | Open | Design | Illustrative only in the plan; real defaults are a tuning call against SITL behavior. Resolved by tuning in SITL. |
| OQ-5 | Canonical integration-test mission(s) + CI runtime/flakiness budget. | Open | Design | Plan's self-identified least-confident area. Resolved by selecting one small strict scenario and a measured runtime budget in the slow tier (ADR-0002). |
| OQ-6 | Low-battery threshold + which `/fmu/out/*` battery field drives abort. | Open | Design | SITL battery-modeling fidelity affects whether the trigger is naturally exercisable. Resolved by picking the field/threshold and unit-testing the transition (SITL-observable half best-effort). |
| OQ-7 | Capture-trigger contract with 04 (explicit "capture now" signal vs 04 infers from AprilTag-in-view + dwell). | Resolved (combined review 2026-06-03) | Design (joint with 04) | **Default direction:** 02 emits an observable "arrived and dwelling at checkpoint N" mission signal that 04 keys capture off of (the checkpoint-arrival semantic this docset owns). Final explicit-vs-inferred shape resolved jointly with 04 §7 and confirmed in the combined review. |
| OQ-8 | Return-to-home semantics: PX4 RTL mode vs explicit home-waypoint offboard sequence. | Open | Design | Plan says "return home" without prescribing mechanism. Resolved by choosing one mechanism behind the RTH state, weighing mode-handoff/landing behavior. |

## Appendix B: User Acceptance Criteria

> Every P1 FR has a corresponding UAC in Given/When/Then form. UAC IDs match their FR.

### UAC-MC-1: Single-command arm + takeoff + hover + land
**GIVEN** a running SITL drone (01-platform up, `/fmu/*` publishing)
**WHEN** the operator runs `ros2 launch patrol_bringup mission_basic.launch.py`
**THEN** the drone arms, takes off to ~5 m AGL, hovers ~10 s, and lands, via offboard control over uXRCE-DDS (no MAVROS/MAVSDK).

### UAC-MC-2: Multi-waypoint patrol with dwell and return-to-home
**GIVEN** a patrol mission YAML with 4+ waypoints and a running SITL drone
**WHEN** the operator runs `ros2 launch patrol_bringup mission_patrol.launch.py`
**THEN** the drone visits each waypoint in configured order, dwells the configured time at each, returns home, and lands.

### UAC-MC-3: Mission configuration loaded from a repo-checked-in YAML file
**GIVEN** the patrol route is defined in a YAML file checked into the repo
**WHEN** a run starts
**THEN** the mission config is read from that YAML file and no route/waypoint data is hardcoded in source.

### UAC-MC-4: ROS-free, separately-testable `MissionStateMachine`
**GIVEN** the `MissionStateMachine` class
**WHEN** the unit suite runs
**THEN** it passes at ≥85% coverage in <5 s without importing rclpy or starting ROS/Gazebo/PX4, with the PX4 interface mocked and the simulator not mocked.

### UAC-MC-5: Robust waypoint-completion criterion
**GIVEN** a waypoint target with a configured tolerance and hold time
**WHEN** the vehicle remains within tolerance continuously for the hold time
**THEN** the state machine marks the waypoint complete and advances — and never advances on exact position equality.

### UAC-MC-6: Mission abort — low-battery and external-signal
**GIVEN** a patrol in flight in SITL
**WHEN** an abort is published on the external-signal topic (or battery telemetry crosses the configured threshold)
**THEN** the state machine transitions to abort and the drone returns home; the external-signal transition is observable in the SITL run.

### UAC-MC-7: Explicit coordinate-frame discipline
**GIVEN** waypoints declaring their frame in YAML
**WHEN** the node converts a waypoint to PX4 NED relative to the EKF origin
**THEN** the conversion happens at exactly one named, reviewable boundary and a unit test confirms a known input→NED output.

### UAC-MC-8: Mission-orchestration ROS topics published for downstream consumers
**GIVEN** a mission running in SITL
**WHEN** ROS 2 inspects topics
**THEN** `/patrol/mission_state` and `/patrol/current_waypoint` are published, `/patrol/abort` is subscribed and drives abort, a checkpoint-arrival/dwelling signal is emitted for 04, and the types/QoS allow 05 to record and Foxglove to render them.

### UAC-MC-9: Abort logic unit-covered
**GIVEN** the abort logic in `MissionStateMachine`
**WHEN** the unit suite runs
**THEN** low-battery, external-signal, and scaffolded-trigger aborts each have a unit test asserting the transition, contributing to the ≥85% coverage gate.

### UAC-MC-10: SITL integration test in CI
**GIVEN** CI's slow tier (per ADR-0002)
**WHEN** the integration suite runs
**THEN** ≥1 test spins up SITL, runs a canonical mission via `mission_*.launch.py`, and passes — and SITL is not a required per-PR check.

## Quality Gate Notes

- **UAC bodies:** All P1 FRs (MC-1…MC-10) have completed (non-stub) UAC bodies in Appendix B. MC-11 is P2 and is covered by AC-8 via MC-9's coverage; no separate UAC required.
- **Coverage figure reconciliation:** DoD AC-4 states ">80% on the mission state machine"; ADR-0002 enforces ≥85% as the CI floor. The PRD uses ≥85% as the gate (the stricter, enforced number) and notes the DoD floor — flagged so the design uses the enforced value, not the looser DoD prose.
- **Deferred cross-docset confirmations:** OQ-2 (checkpoint mapping, default `sim/config/checkpoints.yaml` owned by 03) and OQ-7 (capture-trigger contract with 04) carry settled run-policy defaults marked *confirmed at combined review (2026-06-03)* for the human's combined review of all five docset pairs. They are not invented answers.
- **Out-of-scope completeness:** All six DoD §2 deferrals are carried into Out of Scope, plus one inferred entry (mission-replan / dynamic re-routing [INFERRED] — not in the DoD but a plausible scope-creep vector worth fencing).
