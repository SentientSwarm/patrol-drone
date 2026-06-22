# Definition of Done — Mission Control

**Phase 1 docset:** 2 of 5 · **Milestones:** M3–M4
**Lifecycle status:** DoD ✅ · PRD ✅ · Design ✅
**Source:** docs/phase1_simulation_plan.md — [M3 "Python mission node, takeoff and land"](../../phase1_simulation_plan.md#m3--python-mission-node-takeoff-and-land), [M4 "Multi-waypoint patrol mission"](../../phase1_simulation_plan.md#m4--multi-waypoint-patrol-mission); cross-cutting "Test strategy", "What's explicitly NOT in Phase 1", "Phase 1 exit checklist"
**Stakeholders:** Project owner (solo dev) — operator who runs patrols and maintainer of the mission stack; reviewers — small-PR trunk-based reviewers per "Working agreement"; downstream — Phase 2 (outdoor first flights) reuses the same launch files and state machine; docsets 03/04/05 consume the mission topics and launch entry-point this docset defines.
**Depends on:** 01-platform (ROS 2 Jazzy + uXRCE-DDS bridge, container build, `px4_msgs`, `/fmu/*` telemetry)
**Consumed by:** 03-sim-environment (patrol drives the checkpoint world), 04-perception (capture triggered at waypoint arrival), 05-logging-replay (bag wrapper invoked by the mission launch; mission/abort topics recorded); Phase 2+ flight bring-up.

## 1. Intent

Deliver the mission orchestration layer: a Python ROS 2 node driving PX4 offboard control through a separately-testable state machine that arms, takes off, flies a YAML-configured multi-waypoint patrol with per-waypoint dwell, returns home, lands, and aborts safely. This is the executable heart of Phase 1 — the capability the single exit command (item 1) drives end-to-end.

## 2. Scope

**In scope:**
- A ROS 2 mission node (`patrol_mission`) that commands PX4 offboard control via uXRCE-DDS topics.
- A `MissionStateMachine` plain-Python class, decoupled from rclpy, covering arm → takeoff → waypoint sequence (with dwell) → return-to-home → land, plus abort transitions.
- Waypoint/mission configuration loaded from a repo-checked-in YAML file.
- Waypoint-completion criterion (tolerance + hold time), explicit NED-vs-frame handling at the conversion boundary.
- Mission abort paths: low-battery and external-signal (ROS topic), implemented as state transitions and observable in SITL.
- Launch entry-points `mission_basic.launch.py` (M3) and `mission_patrol.launch.py` (M4→full Phase 1) in `patrol_bringup`.
- Unit tests for the state machine and abort logic; ≥1 SITL integration test in CI running a canonical mission.

**Out of scope (explicit deferrals — item · rationale · target):**
- VIO / SLAM localization · SITL provides ground-truth pose; offboard uses NED from the EKF origin · Phase 3/4.
- Real object/anomaly detection in the mission loop · AprilTag scaffolding is enough; mission only triggers capture · Phase 3/6 (owned by 04-perception in Phase 1).
- EKF2 / VIO parameter tuning, MAVROS-vs-DDS relitigation · uXRCE-DDS is the settled call; tuning needs real sensors · Phase 3/4.
- Manual-takeover abort *trigger* in SITL · transition exists in the state machine, but the plan only requires low-battery + external-signal to be *triggerable* in sim · Phase 2+ (hardware RC).
- Bag recording mechanics, upload, manifest, Foxglove · the mission launch *invokes* the recorder, but the wrapper/pipeline is owned by 05-logging-replay · this Phase, docset 05.
- The world, checkpoint models, and camera topic · owned by 03-sim-environment · this Phase, docset 03.

## 3. Capabilities (must-do — seeds the PRD's functional requirements)

1. **(P1) Arm + takeoff + hover + land via a single launch command.** Customer scenario: the operator runs `ros2 launch patrol_bringup mission_basic.launch.py` and watches the SITL drone arm, climb to 5 m AGL, hold 10 s, and land. Pain removed: without an offboard mission node there is no programmatic flight at all — only manual QGC stick input.
2. **(P1) State machine as a separate, ROS-free testable class.** Customer scenario: the developer runs `pytest` and validates every mission transition in <5 s without launching ROS, Gazebo, or PX4. Pain removed: embedding logic in the node would force slow, flaky SITL runs to verify a one-line transition change.
3. **(P1) Multi-waypoint patrol with dwell and return-to-home.** Customer scenario: the operator defines 4+ waypoints in YAML and the drone visits each in NED order, dwells the configured time, returns home, and lands. Pain removed: a fixed/hardcoded path can't represent a real patrol route and can't be changed without a code edit.
4. **(P1) Mission config loaded from a repo-checked-in YAML file.** Customer scenario: the operator edits `patrol_mission.yaml`, commits it, and the same launch command flies the new route. Pain removed: routes embedded in code are unreviewable and un-versioned.
5. **(P1) Robust waypoint-completion criterion.** Customer scenario: the drone advances to the next waypoint only after holding within tolerance for a set time, mirroring how hardware behaves. Pain removed: exact `position == target` checks never fire on floats or real setpoints, stalling the mission.
6. **(P1) Mission abort — low-battery and external-signal.** Customer scenario: the operator publishes an abort signal (or battery crosses threshold) mid-patrol and the drone returns home; the transition is observable in a SITL run. Pain removed: an abort path bolted on later is the classic source of fly-aways — "if it doesn't work in sim it won't work in flight."
7. **(P1) Explicit coordinate-frame discipline.** Customer scenario: a reviewer reads the mission node and can point to exactly where world/YAML coordinates convert to PX4 NED-relative-to-EKF-origin. Pain removed: silent frame mistakes are "silent and infuriating" and surface only in misflown missions.
8. **(P2) Abort-state scaffolding for triggers not exercisable in SITL** (e.g., manual takeover, timeout). Customer scenario: the developer adds the hardware-triggered abort in Phase 2 by wiring an existing state, not redesigning the machine. (Plan: "Even if the conditions can't be triggered in SITL, the state transitions should be there.")

## 4. Acceptance criteria / Definition of Done (falsifiable — seeds the PRD's UACs)

- [x] **AC-1** — GIVEN a running SITL drone, WHEN `ros2 launch patrol_bringup mission_basic.launch.py` is invoked, THEN the drone arms, takes off to 5 m AGL, hovers 10 s, and lands. *(M3 Exit)*
- [x] **AC-2** — GIVEN a patrol mission YAML with 4+ waypoints, WHEN `ros2 launch patrol_bringup mission_patrol.launch.py` is invoked against SITL, THEN the drone visits each waypoint in order, dwells the configured time at each, returns home, and lands. *(M4 Exit; exit-checklist item 1 — mission-flight portion, integrative)*
- [x] **AC-3** — GIVEN the patrol route definition, WHEN a run starts, THEN the mission config is read from a YAML file checked into the repo (no route data hardcoded in source). *(exit-checklist item 2)*
- [x] **AC-4** — GIVEN the `MissionStateMachine` class, WHEN the unit suite runs, THEN it passes with ≥85% coverage on the mission state machine and completes in <5 s without ROS/Gazebo/PX4. *(M3 Exit; exit-checklist item 3; ADR-0002 enforces ≥85% as the CI floor)*
- [x] **AC-5** — GIVEN CI, WHEN the integration suite runs, THEN ≥1 integration test spins up SITL, runs a canonical mission via the launch file, and passes. *(M3/M4 Exit; exit-checklist item 4)*
- [x] **AC-6** — GIVEN a patrol in flight, WHEN an abort is requested via an external-signal ROS topic, THEN the state machine transitions to abort and the drone returns home; the transition is observable in the SITL run. *(M4 Exit; exit-checklist item 12 — external-signal half)*
- [x] **AC-7** — GIVEN a low-battery condition crossing the configured threshold, WHEN the mission is running, THEN the state machine transitions to abort; this transition is covered by a unit test. *(exit-checklist item 12 — low-battery half)*
- [x] **AC-8** — GIVEN the abort logic, WHEN the unit suite runs, THEN abort transitions (low-battery, external-signal, and scaffolded triggers) are covered by unit tests. *(M4 Exit; exit-checklist item 12)*
- [x] **AC-9** — GIVEN a waypoint target, WHEN the drone is within the configured tolerance for the configured hold time, THEN the state machine marks the waypoint complete and advances — never on exact position equality. *(M4 — waypoint completion criterion)*

## 5. Interfaces

**Owns (contracts this docset defines that others depend on):**
- Launch entry-points: `patrol_bringup` `mission_basic.launch.py` (M3) and `mission_patrol.launch.py` (the Phase-1 exit command).
- Mission/route YAML schema (e.g., `patrol_bringup`/`patrol_mission` config: waypoint list in NED, per-waypoint dwell, completion tolerance + hold time, abort thresholds) — the file referenced by exit-checklist item 2.
- Mission-orchestration ROS topics: published mission state, current/target waypoint, and the abort signal topic consumed to trigger abort (e.g., `/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/abort`). Exact names/types are an open decision (§7); 05-logging-replay records these.
- The `MissionStateMachine` class contract (`patrol_mission` package): `tick(current_state, telemetry) -> command`, exercised by unit tests and reused in Phase 2+.
- The capture-trigger semantic ("drone has arrived and is dwelling at checkpoint N") that 04-perception keys image capture off of.

**Consumes (from other docsets / PX4):**
- `/fmu/out/*` PX4 telemetry (notably `vehicle_local_position`, battery state) and `/fmu/in/*` offboard setpoint/command topics, plus `px4_msgs` — from 01-platform.
- ROS 2 Jazzy + uXRCE-DDS agent + container build environment — from 01-platform.
- Checkpoint world, checkpoint positions, and the simulated RGB camera image topic — from 03-sim-environment (mission flies to those positions).
- Bag-recorder launch include / wrapper invoked at mission start — from 05-logging-replay.

## 6. Settled constraints (do NOT relitigate — cite the source)

- **uXRCE-DDS native, not MAVROS or MAVSDK.** Offboard control goes through PX4's uXRCE-DDS topics directly; MAVSDK/MAVLink translation layers are excluded. (plan "Target stack"; M3 design call; ADR-0001 "Neutral".)
- **State machine as a separate class, not embedded in the node.** ROS plumbing in the node; transition logic in plain Python — the precondition for London-style TDD. (plan M3; "Test strategy".)
- **PX4 offboard uses NED relative to the EKF origin.** Waypoints declare their frame; conversion happens at one explicit boundary. (plan M4 "Coordinate frames".)
- **ROS 2 Jazzy on Ubuntu 24.04, Python 3.12, PX4 v1.17.0.** (plan "Target stack"; ADR-0001.)
- **Don't mock the simulator.** Mock the PX4 interface in unit tests; use real SITL for anything needing flight dynamics. (plan "Test strategy".)
- **CI tiering:** mission-core unit tests + ≥85% coverage gate run per-PR on a pure-Python runner; SITL integration is the slow/flaky tier kept small and strict (nightly SITL scaffold is not a required per-PR check). (ADR-0002.)
- **Abort transitions exist from day one even if not all triggerable in SITL.** (plan M4 "Mission abort paths".)

## 7. Open decisions (handed to /drive — each: question · decision target · why open)

- **State-machine library** · choose `transitions` vs `python-statemachine` vs hand-rolled · Design · plan explicitly flags this as open: "the choice of state machine library … is open. Strong preferences welcome."
- **Mission/route YAML schema shape** · field set, units, frame declaration, abort-threshold encoding · PRD/Design · the plan specifies *that* it's YAML and *what* it must express, not the concrete schema; the `CheckpointCapture` schema (04) and this must stay coherent.
- **Mission topic names, types, and QoS** · e.g., `/patrol/mission_state`, `/patrol/abort` (custom in `patrol_interfaces` vs `std_msgs`) · Design · these are recorded by 05 and must be Foxglove-renderable (exit item 8).
- **Waypoint-completion tolerance and hold-time defaults** · concrete values (plan illustrates "within 0.5 m for 2 s") · Design · illustrative only in the plan; the real defaults are a tuning call.
- **Canonical integration-test mission(s)** · which scenario(s) constitute the CI integration test, and the CI runtime/flakiness budget · Design · plan's self-identified least-confident area: "I want [strong opinions on] integration test orchestration … CI runtime budgets and flakiness."
- **Low-battery threshold + battery telemetry source** · which `/fmu/out/*` field and threshold drive the abort · Design · SITL battery modeling fidelity affects whether the trigger is exercisable.
- **Return-to-home semantics** · PX4 RTL mode vs an explicit home-waypoint offboard sequence · Design · plan says "return home" without prescribing mechanism.

## 8. Assessment signals (so prd-engine right-sizes the PRD)

| Dimension | Value | One-line justification |
|---|---|---|
| Nature | greenfield | First mission-orchestration code in the repo; no prior implementation to enhance. |
| Complexity | complex | Stateful flight logic, offboard control, abort safety, plus a SITL integration harness across multiple consumers. |
| Urgency | standard | Sequenced Phase-1 milestone work; no emergency, no pure exploration. |
| Risk | medium | Abort/fly-away safety semantics and the offboard control loop are the highest-consequence logic in Phase 1, but the consequence is contained to sim — pre-hardware, no airframe and no data at stake, fully recoverable before Phase 2. (The flight-safety logic is *exercised* in SITL precisely so the risk is bought down before it can ever reach hardware.) |
| Reversibility | mostly-reversible | Pre-hardware and pure software, but mission topics, YAML schema, and the state-machine contract are consumed by 03/04/05 and Phase 2+, so changes cascade. |
| Scope | cross-service | Owns contracts consumed by three sibling docsets and reused by later phases. |
| Audience | developer | Solo-dev / small-PR-reviewer audience per the working agreement. |

**Suggested PRD tier:** Standard (Complexity=complex × Risk=medium → Standard per prd-engine's Complexity×Risk matrix, which lands Complex×Low-Medium squarely on Standard with no conflict-rule bump needed; the flight-safety consequence is real but sim-only and fully recoverable pre-hardware, so it sits in the Low-Medium column rather than High-Critical, and the cross-service scope pulls in conditional Cross-Service-Impact / Operational-Readiness sections rather than a tier jump to Comprehensive).

## 9. Traceability

- **Milestones:**
  - M3 — Python mission node, takeoff and land (state machine + offboard via uXRCE-DDS) — docs/phase1_simulation_plan.md#m3--python-mission-node-takeoff-and-land
  - M4 — Multi-waypoint patrol mission (YAML waypoints, dwell, RTH, abort) — docs/phase1_simulation_plan.md#m4--multi-waypoint-patrol-mission
- **Exit-checklist items owned:** 2, 3, 4, 12 (primary). Shared/integrative: 1 (mission-flight behavior; this docset owns the patrol logic and the launch entry-point, but the full end-to-end claim depends on 01/03/04/05).
- **Packages / dirs:** `ros2_ws/src/patrol_mission/`, `ros2_ws/src/patrol_bringup/` (launch files, configs, params); consumes `ros2_ws/src/external/px4_msgs/`; mission-state types may live in `ros2_ws/src/patrol_interfaces/` (owned by 04). Tests in `tests/unit/` (state machine) and `tests/integration/` (SITL mission).
- **Lifecycle:** dod.md (this) → prd.md (via /drive) → design.md (via /drive)
