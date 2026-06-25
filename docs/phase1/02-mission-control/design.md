# Mission Control — System Design Working Document

**Status:** Approved (combined review 2026-06-03; bootstrapped to Linear) · **Version:** 0.3.0 · **Date:** 2026-06-03
**Projects:** Patrol Drone Phase 1 (pre-hardware simulation) — Mission Control (docset 02 of 5)
**Authors:** jxstanford@wemodulate.energy (solo dev / DRI)
**Requirements source (sole):** `docs/phase1/02-mission-control/prd.md`
**Upstream:** `docs/phase1/02-mission-control/dod.md` · `docs/phase1_simulation_plan.md` (M3–M4) · ADR-0001 (distro/OS, uXRCE-DDS) · ADR-0002 (two-layer CI)

---

## 1. Introduction

This design realizes the Mission Control docset: the mission-orchestration layer that sits on top of the PX4 SITL substrate that docset 01 delivers. It is built from two cooperating artifacts and the glue around them:

- **`MissionStateMachine`** — a plain-Python, rclpy-free class that owns every mission *decision*: arm, takeoff to altitude, hover, fly a waypoint sequence with per-waypoint dwell, return home, land, and abort on low-battery or an external signal (plus scaffolded triggers that cannot be fired in SITL). Its only public surface is `tick(current_state, telemetry) -> (next_state, command)`.
- **`PatrolMissionNode`** — a thin rclpy node that owns every *mechanism*: subscribe `/fmu/out/*`, publish `/fmu/in/*`, run the offboard keepalive heartbeat, convert world/ENU → PX4 NED at exactly one boundary, publish `/patrol/*` for downstream consumers, and drive `tick()` from a fixed-rate timer.

Two launch files (`mission_basic.launch.py`, `mission_patrol.launch.py`) wire the node into the running simulation; a versioned YAML file makes the patrol route a reviewable, diffable artifact rather than code. The decision/mechanism split is not stylistic — it is the precondition for the testing posture the PRD demands (MC-4): with all decision logic in a pure-Python class, every transition (including all four abort paths) is validated in a sub-5-second unit suite at ≥85% coverage with **no** ROS/Gazebo/PX4 process started, and the same `MissionStateMachine` carries unchanged into Phase 2 hardware bring-up.

This docset owns five contracts consumed by three sibling docsets and reused by Phase 2+: the mission/route YAML schema (consumed by 03/05), the `mission_*.launch.py` entry-points (consumed by 03/05), the `/patrol/{mission_state,current_waypoint,abort}` topics (recorded by 05), the checkpoint-arrival capture-trigger semantic (consumed by 04), and the `MissionStateMachine` class contract (reused by Phase 2+). It consumes `/fmu/*` + `px4_msgs` from 01, the shared checkpoint mapping from 03, and the bag-recorder launch include from 05. The two consumed cross-docset defaults still pending the combined human review (checkpoint mapping with 03; capture trigger with 04) are carried as Open Questions with their settled defaults applied, not silently invented — both were confirmed at the combined review (2026-06-03).

### Source Projects (Linear)

| # | Project | Est. | Wave |
|---|---------|------|------|
| 1 | [Patrol Drone 02 Mission Control](https://linear.app/wemodulate/project/patrol-drone-02-mission-control-539c82e00210) | ~22 ew | 2 |

### Related Projects

| Project | Relevance |
|---------|-----------|
| 01-platform — Platform & Simulation Foundation | Provides the ROS 2 Jazzy + uXRCE-DDS bridge, `px4_msgs`, the `/fmu/*` telemetry surface, and the `patrol_bringup`/`patrol_interfaces` package shells this docset builds on. Must be green first (build/dependency order `01 → 02 → 03 → 04 → 05`). |
| 03-sim-environment — Simulation Environment & Assets | Owns the checkpoint world + the shared `sim/config/checkpoints.yaml`; this docset reads its `position` entries to build waypoints and exercises its world during patrols. |
| 04-perception — Perception & Checkpoint Capture | Keys image capture off the atomic `/patrol/dwell` capture event this docset emits (one `std_msgs/Int32` per checkpoint visit; OQ-7). |
| 05-logging-replay — Logging & Replay Pipeline | Records `/patrol/{mission_state,current_waypoint,dwell,abort}`; its recorder launch include is invoked by `mission_patrol.launch.py`. |

---

## 2. Open Questions & Assumptions

All eight PRD Open Questions are carried forward. Six were Resolved in design; the two cross-docset contract defaults (OQ-2, OQ-7) are **Resolved (combined review 2026-06-03)** — confirmed at the combined human review (COMBINED-REVIEW decisions 5 and 2 respectively). No settled item is reopened. Two genuinely-deferred measurement items remain Deferred with rationale (OQ-5 re-measure → MZ; the §3.5 pre-M3 precondition probes → run before M1). Two design assumptions (A-1, A-2) are recorded.

| # | Item | Source | Status |
|---|------|--------|--------|
| OQ-1 | State-machine library: `transitions` vs `python-statemachine` vs hand-rolled | PRD OQ-1; plan M3 | **Resolved (hand-rolled, ratified 2026-06-03)** |
| OQ-2 | Checkpoint mapping schema + file location shared with 03/04 | PRD OQ-2; COMBINED-REVIEW #5 | **Resolved (combined review 2026-06-03)** — schema confirmed; the file path is isolated behind a parameter |
| OQ-3 | Mission topic names, types, QoS | PRD OQ-3; COMBINED-REVIEW #4 | **Resolved (`std_msgs`, ratified 2026-06-03)** |
| OQ-4 | Waypoint-completion tolerance + hold-time defaults | PRD OQ-4 | **Resolved (`0.5 m` / `2.0 s`, ratified 2026-06-03)** |
| OQ-5 | Canonical integration-test mission(s) + CI runtime/flakiness budget | PRD OQ-5 | **Resolved (provisional budget)** — runtime figure re-measured in MZ |
| OQ-6 | Low-battery threshold + battery telemetry field | PRD OQ-6 | **Resolved (`battery_status.remaining < 0.20`, ratified 2026-06-03)** |
| OQ-7 | Capture-trigger contract with 04 | PRD OQ-7; COMBINED-REVIEW #2 | **Resolved (combined review 2026-06-03; superseded 2026-06-21)** — atomic `/patrol/dwell` (`std_msgs/Int32`, dwelled index) event, one per DWELL entry; 04 captures once per event (was: `DWELL` + `current_waypoint` correlation) |
| OQ-8 | Return-to-home semantics: PX4 RTL vs explicit home-waypoint offboard sequence | PRD OQ-8 | **Resolved (explicit home-waypoint sequence, ratified 2026-06-03)** |
| A-1 | Assumption: 01's `patrol_bringup`/`patrol_interfaces` shells exist + `colcon build` green before M1 | Design | **Assumption — verified §3.5 (pre-M1 gate)** |
| A-2 | Assumption [INFERRED]: PX4 offboard needs a continuous setpoint stream (~10 Hz) before the mode switch | Design (rabbit hole) | **Assumption — verified §3.5; node-plumbing only** |

**Resolutions in full:**

- **OQ-1 — Resolved (hand-rolled state machine).** A hand-rolled `enum` + `tick()` dispatch is chosen over `transitions` / `python-statemachine`. With ~10 stable states (IDLE, ARMING, TAKEOFF, HOVER, WAYPOINT, DWELL, RTH, LANDING, ABORT, DONE) the dependency weight of a library buys nothing the project needs, and a hand-rolled machine has zero runtime dependency on the Layer-A pure-Python runner, is import-clean and pickle-safe, reads top-to-bottom in one file (KISS), and gives direct branch coverage of every transition. See §3.4, §4.2.3. *Human-ratified (COMBINED-REVIEW "01 & 02 design-decision ratifications").*
- **OQ-2 — Resolved (combined review 2026-06-03): checkpoint mapping schema AND file location.** A single shared YAML at `sim/config/checkpoints.yaml`, owned by 03, list of `{checkpoint_id, position{x,y,z} world/ENU meters, tag_family, tag_id}`. This docset (02) reads the `position` entries to build waypoints (converting ENU → NED at the MC-7 boundary) and references `checkpoint_id` from the mission YAML rather than duplicating positions. The **schema (field set)** matches 03 DoD §5 and is confirmed (COMBINED-REVIEW #5). The **physical file location/name** is still flagged open in 03 DoD §7; consuming code is isolated from that decision because the path is a single `MissionConfig` loader parameter / launch argument, so an agreed-different location is a one-line config change, not a code edit. Joint with 03 §7 + 04.
- **OQ-3 — Resolved (`std_msgs`).** `/patrol/mission_state` = `std_msgs/String`, `/patrol/current_waypoint` = `std_msgs/Int32`, `/patrol/abort` = `std_msgs/Bool` (subscribed). These round-trip through MCAP and render in Foxglove with **no** custom-type plugin (exit item 8). Typed mission states live in the state-machine code, not in a custom message. The capture trigger (OQ-7) is the atomic `/patrol/dwell` topic — also a plain `std_msgs/Int32`, so still **no custom message type** is owned here. QoS: reliable / transient-local depth-1 for `mission_state` and `current_waypoint` (a late subscriber sees the latest value); `abort` reliable + volatile (the latch lives in the state machine's `_NON_ABORTABLE` set, not topic durability, so a plain `ros2 topic pub` is QoS-compatible); `/patrol/dwell` reliable + volatile, keep-last route-covering depth (a discrete live event, delivered once, never coalesced). *Human-ratified (COMBINED-REVIEW #4 + "02 OQ-3").*
- **OQ-4 — Resolved.** `tolerance_m: 0.5`, `hold_time_s: 2.0` — the plan's illustratives adopted as overridable YAML defaults, SITL-tunable without a code change. *Human-ratified ("02 OQ-4").*
- **OQ-5 — Resolved (provisional budget).** Two canonical SITL scenarios: one basic mission and one reduced 2-waypoint patrol, run in the nightly SITL tier. Provisional budget ≤8 min/scenario including spin-up; quarantine-not-expand on a >1-in-5 flake rate or a >2× budget overrun; never a required per-PR check. The runtime figure is **provisional until measured against 01's landed SITL** — re-measure is seeded into MZ (§6.5). *Bulk-accepted ("02 OQ-5"); measurement remains deferred.*
- **OQ-6 — Resolved.** Abort drives off `/fmu/out/battery_status_v1.remaining` (a normalized 0.0–1.0 fraction), default threshold `0.20`, configured in YAML. The transition is unit-tested (AC-7); SITL-observable depletion is best-effort because SITL battery modeling may not deplete realistically. *Human-ratified ("02 OQ-6").*
- **OQ-7 — Resolved (combined review 2026-06-03); superseded 2026-06-21: capture-trigger contract with 04.** 02 emits an **atomic** `/patrol/dwell` capture event — `std_msgs/Int32` carrying the dwelled waypoint index, owned by 02, one event published on the rising edge into `DWELL` (QoS: reliable + volatile, keep-last with a route-covering depth so every checkpoint event is delivered once and never coalesced to "latest"). The index maps 1:1 to a `checkpoint_id` via the mission YAML. 04 subscribes to this single topic and captures exactly once per event (satisfying 04 AC-6, one capture per checkpoint visit); it does **not** correlate the two separate, non-atomic `mission_state` + `current_waypoint` topics. `mission_state` / `current_waypoint` remain the observable/Foxglove surface (OQ-3), not the capture trigger. Joint with 04. **Supersedes** the originally-ratified "no new topic; `DWELL` + `current_waypoint` is the trigger" default: that default carried a cross-topic delivery race (DDS guarantees per-topic ordering, not an atomic cross-topic snapshot, so a `current_waypoint=i+1` sample could arrive before a still-pending `DWELL(i)` and mis-attribute a capture one leg early) surfaced across Hermes PR #8 reviews R6–R11; the atomic event removes the race by construction. *Confirmed (COMBINED-REVIEW #2); revised 2026-06-21 per PR #8 review (this repo is the owner-of-record).*
- **OQ-8 — Resolved.** Return-to-home is an **explicit home-waypoint offboard sequence** behind the single `RTH` state — not a handoff to PX4 RTL mode. This keeps control authority inside the state machine (no offboard → RTL mode handoff to reason about) and keeps RTH unit-testable. See §4.2.6, §4.5 Sequence 3. *Human-ratified (COMBINED-REVIEW "02 OQ-8").*
- **A-1 (assumption).** 01's `patrol_bringup`/`patrol_interfaces` package shells exist and `colcon build` is green before M1 begins. Verified in §3.5 row 1 (README docset matrix); recorded as a pre-M1 gate because the repo is a single-commit skeleton and 01 has not yet landed.
- **A-2 (assumption, INFERRED).** PX4 offboard requires a continuous setpoint stream (~10 Hz) published *before* and *during* the offboard-mode switch, or PX4 rejects/exits offboard. This drives the node keepalive heartbeat. Verified in §3.5 row 2 against `offboard_control.py` (plan:215); it is strictly node-plumbing and never enters the state machine.

---

## 3. Existing Foundation

This is a greenfield docset landing inside 01-platform's ROS 2 / PX4 SITL substrate and the two-layer CI ADR-0002 already establishes. There is no prior mission code to enhance; the "existing foundation" is the substrate 02 consumes and the CI it plugs into.

### 3.1 Runtime Architecture (substrate from 01)

```
                  ros2 launch patrol_bringup mission_patrol.launch.py
                                     │
                                     │ (mission_patrol also includes 05's record.launch.py)
                                     ▼
        ┌───────────────────────────────────────────────────────────┐
        │                  PatrolMissionNode (rclpy)                  │
        │  ┌─────────────────────────────────────────────────────┐  │
        │  │  10 Hz timer:                                        │  │
        │  │   1. publish offboard keepalive heartbeat            │  │
        │  │   2. build Telemetry from latest /fmu/out/* cache    │  │
        │  │   3. (next_state, command) = sm.tick(state, telem)   │  │
        │  │   4. translate Command → /fmu/in/* setpoints/cmds    │  │
        │  │   5. publish /patrol/{mission_state,current_waypoint} │  │
        │  └─────────────────────────────────────────────────────┘  │
        │      │ drives                  ▲ ENU→NED once at startup    │
        │      ▼                         │ (FrameConversion boundary) │
        │  MissionStateMachine      MissionConfig (YAML loader)       │
        │  (pure Python, decisions)  (pure Python, schema)            │
        └───────────────────────────────────────────────────────────┘
             │ sub /fmu/out/*          │ pub /fmu/in/*        │ pub /patrol/*  / sub /patrol/abort
             ▼                         ▼                      ▼
        ┌──────────────────────────────────────────┐    ┌──────────────────────────┐
        │   PX4 SITL  (Gazebo Harmonic)             │    │   04 (capture trigger)   │
        │   via uXRCE-DDS agent  (px4_msgs)         │    │   05 (bag recorder)      │
        └──────────────────────────────────────────┘    └──────────────────────────┘
```

| Layer | Owner | Contents |
|-------|-------|----------|
| Config | 02 | `patrol_mission.yaml`, `mission_basic.yaml`; consumes 03's `checkpoints.yaml`; launch files |
| Decision (ROS-free) | 02 | `MissionStateMachine`, `FrameConversion` |
| Plumbing | 02 | `PatrolMissionNode` (rclpy, keepalive, pub/sub) |
| Flight substrate | 01 (consumed) | PX4 SITL, Gazebo Harmonic, uXRCE-DDS agent, `/fmu/*`, `px4_msgs` |
| CI | 01 + ADR-0002 (consumed/extended) | Layer A (pure-Python per-PR) + Layer B (Jazzy container) + nightly SITL |

### 3.2 Consumed `/fmu/*` surface (px4_msgs, PX4 v1.16.x)

| Direction | Topic | px4_msgs type | Fields this docset uses |
|-----------|-------|---------------|--------------------------|
| sub | `/fmu/out/vehicle_local_position_v1` | `VehicleLocalPosition` | `x, y, z` (NED, m), `heading` — current position for tolerance+hold |
| sub | `/fmu/out/battery_status_v1` | `BatteryStatus` | `remaining` (0.0–1.0) — low-battery abort (OQ-6) |
| sub | `/fmu/out/vehicle_status_v1` | `VehicleStatus` | `arming_state`, `nav_state` — arm/offboard confirmation |
| pub | `/fmu/in/offboard_control_mode` | `OffboardControlMode` | `position=True` — keepalive heartbeat (A-2) |
| pub | `/fmu/in/trajectory_setpoint` | `TrajectorySetpoint` | `position[3]` (NED), `yaw` — the active setpoint |
| pub | `/fmu/in/vehicle_command` | `VehicleCommand` | arm / set-offboard-mode / land commands |

### 3.3 Existing CI (ADR-0002)

- **Layer A (pure-Python, per-PR):** this docset adds the `MissionStateMachine` + `FrameConversion` + `MissionConfig` unit suite. Hard gates: ≥85% coverage (the enforced ADR-0002 floor, governing over the DoD's earlier `>80%` prose — COMBINED-REVIEW #8), plus `xenon` (complexity), `ruff` (lint), `mypy` (types). No ROS toolchain required (the mission core is rclpy-free; `qos.py` imports rclpy but runs under the unit suite's rclpy stub).
- **Layer B (Jazzy container):** `colcon build` + `colcon test` of the `patrol_mission` / `patrol_bringup` packages.
- **SITL nightly:** the canonical mission integration tests (MC-10, OQ-5). Never a required per-PR check.

### 3.4 Architectural Decision: hand-rolled state machine over a library (OQ-1)

**Decision:** Implement `MissionStateMachine` as a hand-rolled `enum` + `tick()` dispatch, not a state-machine library.
**Rationale:** ~10 stable states; the four abort guards are evaluated first every tick (highest precedence) before normal dispatch; transitions read top-to-bottom in one file; coverage tooling sees plain branches. A library adds a runtime dependency to the Layer-A runner and an indirection layer for no behavioral gain at this size.
**Implication:** `tick(current_state, telemetry) -> (next_state, command)` is the only public surface. The machine holds no rclpy import and no I/O; the clock is injected via `telemetry.now_s` so tests are deterministic. *Human-ratified.*

### 3.5 Verified Preconditions

The repo is a single-commit skeleton, so the producer docsets (01/03/05) and the vendored `px4_msgs` have not landed. Each row below is either verified now against the plan / docset matrix, or recorded as a pre-M1 verification gate to run before the milestone starts. The Result column quotes the verified-or-expected shape; the Citation column points at the source that proves it.

| # | Claim | Verification | Result | Citation |
|---|-------|--------------|--------|----------|
| 1 | 01 provides `patrol_bringup` + `patrol_interfaces` package shells and a green `colcon build` | `colcon list \| grep -E 'patrol_bringup\|patrol_interfaces'` in the Jazzy container | pending (01 not yet landed) — gates M1 start | plan repo-structure (plan:139–164); 01 DoD §5 |
| 2 | PX4 offboard needs a continuous setpoint stream before the mode switch (A-2) | Read `offboard_control.py` keepalive pattern; reproduce in the M1 skeleton | verified (pattern documented) — keepalive is node plumbing | plan M3 "offboard example" (plan:215) |
| 3 | `px4_msgs` provides the six message types in §3.2, vendored/pinned to PX4 v1.16.x | `ros2 interface show px4_msgs/msg/VehicleLocalPosition` (and the other five) in the container | pending (px4_msgs vendored by 01) | plan "px4_msgs vendored" (plan:170); 01 OQ-3 |
| 4 | Checkpoint-positions YAML schema matches `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}` | `yq '.[0]' sim/config/checkpoints.yaml` once 03 lands | schema confirmed (03 DoD §5); **file location open per 03 DoD §7 — path parameterized** | COMBINED-REVIEW #5; 03 DoD §5/§7 |
| 5 | 05 provides a recorder launch include that `mission_patrol.launch.py` can `IncludeLaunchDescription` | `ros2 launch patrol_bringup mission_patrol.launch.py --show-args` lists the recorder include | pending (05 not yet landed) — non-critical to flight | 05 DoD §5 |
| 6 | PyYAML importable via the ROS 2 Jazzy base (`python3-yaml`) | `python3 -c "import yaml"` in the Jazzy container | pending — pre-M1 gate; fallback `<exec_depend>python3-yaml</exec_depend>` in `package.xml` | ROS 2 Jazzy base dep |

---

## 4. Detailed Design

### 4.1 UC Traceability Matrix

| Design Component | Covers FRs | Covers UACs | Inferred | Milestone(s) |
|------------------|------------|-------------|----------|--------------|
| **MissionStateMachine** | MC-4, MC-5, MC-6, MC-9, MC-11 | UAC-MC-4/5/6/9 | INF-M1 | M1 + M2 |
| **FrameConversion** | MC-7 | UAC-MC-7 | — | M2 (boundary stubbed M1) |
| **MissionConfig** | MC-3, MC-5, MC-6, MC-7 | UAC-MC-3 | INF-M3 | M2 (minimal in M1) |
| **PatrolMissionNode** | MC-1, MC-2, MC-6, MC-8 | UAC-MC-1/2/6/8 | INF-M2 | M1 + M2 |
| **Launch entry-points** | MC-1, MC-2, MC-8 | UAC-MC-1/2 | — | M1 + M2 |
| **Test suites** | MC-4, MC-9, MC-10 | UAC-MC-4/9/10 | INF-M1 | M1 + M2 |

Every PRD FR (MC-1…MC-11) and every UAC (UAC-MC-1…10) is covered. PRD Appendix B is the authoritative UAC set; this matrix maps components to it without duplicating the UAC bodies.

### 4.2 Component Architecture

#### 4.2.1 Component Inventory

| Component | Type | Boundary | Responsibility | Dependencies |
|-----------|------|----------|----------------|--------------|
| MissionStateMachine | module (pure Python) | Owns every mission *decision*; no rclpy, no I/O | `tick()` dispatch over the state enum; abort guards; tolerance+hold completion | `FrameConversion` (NED waypoints), `MissionConfig` (params) — both passed in as plain data |
| FrameConversion | module (pure Python) | Owns the single ENU↔NED conversion site | `to_ned_from_origin(point, frame, ekf_origin_ned)` | none |
| MissionConfig | module (pure Python) | Owns YAML parse + schema + validation + `checkpoint_id` resolution | Load mission YAML, resolve checkpoint refs, fail loud on bad config | `FrameConversion` (validates frames), checkpoints YAML (consumed read-only) |
| CommandBuilder (`commands.py`) | module (pure Python) | Owns the PX4 `VehicleCommand` sequence; no rclpy | `build_vehicle_commands(...)` → ordered arm / set-offboard / land commands with A-2 warmup gating | none |
| TopicNames (`topics.py`) | module (pure Python) | Single source of truth for `/fmu/*` + `/patrol/*` topic names (incl. the PX4 v1.17 `_v1` output-suffix rule) | name constants + `named_topic()` lookup | none |
| QoSProfiles (`qos.py`) | module (rclpy) | Single source of truth for the per-surface QoS profiles | `px4_qos()`, `patrol_state_qos()`, `patrol_event_qos()`, `patrol_abort_qos()` | `rclpy.qos` (Layer-B; measured via the unit rclpy stub) |
| PatrolMissionNode | module (rclpy node) | Owns every *mechanism*: pub/sub, keepalive, timer, frame boundary call | Build Telemetry, drive `tick()`, translate Command → `/fmu/in/*`, publish `/patrol/*` | `MissionStateMachine`, `FrameConversion`, `MissionConfig`, `px4_msgs`, `std_msgs`, rclpy |
| Launch entry-points | config | Owns wiring only; no logic | Start the node with the right YAML; `mission_patrol` includes 05's recorder | `PatrolMissionNode`, 05 recorder include |
| Test suites | tests | Owns unit (ROS-free) + integration (SITL) coverage | Drive all transitions/aborts/frames/config in <5 s; drive `mission_*.launch.py` against real SITL | all components; SITL (integration only) |

#### 4.2.2 Component Dependency Diagram

```
   Launch entry-points
   (mission_basic / mission_patrol)
            │ starts
            ▼
   PatrolMissionNode (rclpy) ──────────────► px4_msgs / /fmu/* ◄──► PX4 SITL
        │        │        │                         (offboard control loop)
        │        │        └── pub /patrol/* ──────────────────────► 04 / 05
        │        └── FrameConversion (ENU→NED, single site)
        ▼
   MissionStateMachine ◄── MissionConfig (Waypoint[], AbortConfig)
        │                        ▲
        │                        └── reads sim/config/checkpoints.yaml (03, read-only)
        ▼
   tick(state, telemetry) -> (next_state, command)

   tests/unit/  ──► MissionStateMachine, FrameConversion, MissionConfig   (PX4 mocked, sim NOT mocked)
   tests/integration/  ──► Launch → Node → real SITL                       (nightly)
```

#### 4.2.3 MissionStateMachine

**Type:** module (pure Python)
**Location:** `ros2_ws/src/patrol_mission/patrol_mission/state_machine.py`
**Boundary:** Owns every decision; raises on no I/O, holds no rclpy import. The node injects telemetry and consumes the returned command.
**Dependencies:** `MissionConfig` (params, passed at construction as plain data), NED waypoints (pre-resolved by the node via `FrameConversion`).

State enum and the abort precedence are the heart of the machine. Abort guards are evaluated **first every tick** (highest precedence), so an abort condition pre-empts any normal transition.

```python
from __future__ import annotations
from dataclasses import dataclass, field
from enum import Enum, auto


class MissionState(Enum):
    IDLE = auto()       # pre-arm; publishing keepalive only
    ARMING = auto()     # arm command sent; waiting for arming_state
    TAKEOFF = auto()    # climbing to takeoff_alt_m
    HOVER = auto()      # holding at takeoff point for hover_time_s
    WAYPOINT = auto()   # flying toward waypoints[i]
    DWELL = auto()      # arrived at waypoints[i]; holding for dwell_s (capture trigger for 04)
    RTH = auto()        # explicit home-waypoint offboard sequence (OQ-8)
    LANDING = auto()    # land command sent; descending
    ABORT = auto()      # abort latched; routes to RTH
    DONE = auto()       # disarmed on the ground; terminal


class AbortReason(Enum):
    NONE = auto()
    EXTERNAL_SIGNAL = auto()   # /patrol/abort True (MC-6)
    LOW_BATTERY = auto()       # battery_status.remaining < threshold (MC-6/OQ-6)
    MANUAL_TAKEOVER = auto()   # scaffolded — not fired in SITL (MC-11)
    TIMEOUT = auto()           # scaffolded — not fired in SITL (MC-11)


@dataclass(frozen=True)
class Telemetry:
    now_s: float                 # injected clock — deterministic tests (INF-M1)
    position_ned: tuple[float, float, float]
    armed: bool
    offboard_active: bool
    battery_remaining: float     # 0.0..1.0 (BatteryStatus.remaining)
    abort_requested: bool        # latched value from /patrol/abort
    manual_takeover: bool = False  # always False in SITL (scaffold)
    timed_out: bool = False        # always False in SITL (scaffold)


@dataclass(frozen=True)
class Command:
    arm: bool = False
    set_offboard: bool = False
    land: bool = False
    setpoint_ned: tuple[float, float, float] | None = None
    yaw: float = 0.0
    current_waypoint: int = -1   # published to /patrol/current_waypoint (-1 = none)
    # the mission-state STRING is NOT carried here — the node publishes next_state.name
    # (tick() returns (next_state, command); the enum is the single source of truth)


@dataclass
class _Progress:
    """Mutable per-run bookkeeping the machine carries between ticks."""
    waypoint_index: int = 0
    inside_since_s: float | None = None   # first time inside tolerance (tolerance+hold, MC-5)
    state_entered_s: float = 0.0          # timestamp the current state was entered
    last_state: MissionState | None = None  # previous tick's state — drives _enter_if_new/reset_timing
    abort_reason: AbortReason = AbortReason.NONE
```

The dispatch contract and the two completion helpers (no equality test, ever — MC-5):

```python
class MissionStateMachine:
    def __init__(self, config: "MissionConfig", waypoints_ned: list[tuple[float, float, float]],
                 home_ned: tuple[float, float, float]):
        self._cfg = config
        self._wps = waypoints_ned          # already ENU→NED at the single boundary (MC-7)
        self._home = home_ned
        self._p = _Progress()

    def tick(self, state: MissionState, telem: Telemetry) -> tuple[MissionState, Command]:
        """
        Guards: abort guards evaluated FIRST (highest precedence); a latched abort
                routes ABORT -> RTH regardless of the normal transition.
        Effect: returns the next state + the command the node should issue this tick.
        Side effects: none (pure) — bookkeeping lives in self._p only.
        """
        reason = self._abort_reason(telem)
        if reason is not AbortReason.NONE and state not in (MissionState.ABORT,
                                                            MissionState.RTH,
                                                            MissionState.LANDING,
                                                            MissionState.DONE):
            self._p.abort_reason = reason
            return self._enter(MissionState.ABORT, telem)
        return self._dispatch[state](self, telem)

    def _abort_reason(self, telem: Telemetry) -> AbortReason:
        if telem.abort_requested:
            return AbortReason.EXTERNAL_SIGNAL
        if telem.battery_remaining < self._cfg.abort.low_battery_threshold:
            return AbortReason.LOW_BATTERY
        if telem.manual_takeover:                 # scaffold (MC-11) — never True in SITL
            return AbortReason.MANUAL_TAKEOVER
        if telem.timed_out:                       # scaffold (MC-11) — never True in SITL
            return AbortReason.TIMEOUT
        return AbortReason.NONE

    def _within_tolerance_for_hold(self, telem: Telemetry, target_ned: tuple[float, float, float]) -> bool:
        """MC-5: complete iff continuously within tolerance for hold_time_s. Never tests equality."""
        if _distance(telem.position_ned, target_ned) <= self._cfg.completion.tolerance_m:
            if self._p.inside_since_s is None:
                self._p.inside_since_s = telem.now_s
            return (telem.now_s - self._p.inside_since_s) >= self._cfg.completion.hold_time_s
        self._p.inside_since_s = None             # left the tolerance ball; reset the hold clock
        return False
```

The shipped class adds small pure-Python guard helpers during M3/M4 hardening (not shown): `telemetry_fresh` / `local_position_usable` (stale-telemetry + EKF gating), `battery_low` (unknown-battery `-1`/NaN handling), `_enter_if_new` / `reset_timing` (state-entry detection via `_Progress.last_state`), and a `waypoints_ned`-vs-`config.waypoints` length guard. `FrameConversion` (§4.2.4) likewise adds `takeoff_target_ned` (the AGL-above-home setpoint). All are Layer-A tested.

Per-state behavior (entered / tick / next):

| State | On entry | Tick behavior | Transitions to |
|-------|----------|---------------|----------------|
| IDLE | — | emit keepalive; issue `arm` | ARMING |
| ARMING | record `state_entered_s` | wait for `telem.armed`; issue `set_offboard` | TAKEOFF when armed + offboard_active |
| TAKEOFF | reset hold clock | setpoint = takeoff point at `takeoff_alt_m` | HOVER when `_within_tolerance_for_hold` at takeoff point |
| HOVER | reset hold clock | hold takeoff point | WAYPOINT (or RTH if no waypoints) after `hover_time_s` |
| WAYPOINT | reset hold clock | setpoint = `waypoints_ned[i]` | DWELL when `_within_tolerance_for_hold` at `waypoints_ned[i]` |
| DWELL | reset hold clock; publish index | hold `waypoints_ned[i]` (04 captures once here) | next WAYPOINT, or RTH after last, when `dwell_s` elapsed |
| RTH | reset hold clock | setpoint = `home_ned` (explicit home-waypoint sequence, OQ-8) | LANDING when `_within_tolerance_for_hold` at home |
| LANDING | issue `land` | wait for ground | DONE when `not telem.armed` (disarmed) |
| ABORT | latch `abort_reason` | (pass-through) | RTH (next tick) |
| DONE | — | terminal; keepalive only | — (terminal) |

*Traces to: MC-4, MC-5, MC-6, MC-9, MC-11 · UAC-MC-4/5/6/9 · INF-M1.*

#### 4.2.4 FrameConversion

**Type:** module (pure Python)
**Location:** `ros2_ws/src/patrol_mission/patrol_mission/frames.py`
**Boundary:** The single ENU↔NED conversion site (Tenet 4 — a second site is review-rejectable).
**Dependencies:** none.

```python
from __future__ import annotations

Point = tuple[float, float, float]


def to_ned_from_origin(point: Point, frame: str, ekf_origin_ned: Point) -> Point:
    """
    Convert a world/YAML waypoint to PX4 NED relative to the EKF origin (MC-7).
    The ONLY place this conversion happens. Fail loud on an unknown frame.

      frame == "ned":  passthrough (already NED relative to origin)
      frame == "enu":  ENU (x_e, y_n, z_u) -> NED (x_n=y_e? ) handled below, then offset

    ENU->NED axis map: (x_enu, y_enu, z_enu) -> (y_enu, x_enu, -z_enu)
    Then add the EKF-origin NED offset so the result is origin-relative.
    """
    ox, oy, oz = ekf_origin_ned
    if frame == "ned":
        x, y, z = point
        return (x + ox, y + oy, z + oz)
    if frame == "enu":
        xe, ye, ze = point
        return (ye + ox, xe + oy, -ze + oz)
    raise ValueError(f"unknown frame {frame!r}: expected 'ned' or 'enu'")
```

*Traces to: MC-7 · UAC-MC-7.*

#### 4.2.5 MissionConfig

**Type:** module (pure Python)
**Location:** `ros2_ws/src/patrol_mission/patrol_mission/config.py`
**Boundary:** Owns YAML parse + schema + validation + `checkpoint_id` resolution; no rclpy.
**Dependencies:** `FrameConversion` (validates frame strings), the consumed checkpoints YAML (read-only).

```python
from __future__ import annotations
from dataclasses import dataclass
import yaml


@dataclass(frozen=True)
class Completion:
    tolerance_m: float = 0.5     # OQ-4 default
    hold_time_s: float = 2.0     # OQ-4 default


@dataclass(frozen=True)
class AbortConfig:
    low_battery_threshold: float = 0.20   # OQ-6 default (battery_status.remaining)


@dataclass(frozen=True)
class Waypoint:
    position_enu: tuple[float, float, float]  # resolved to ENU (from inline or checkpoint_id)
    frame: str                                # "enu" | "ned"
    dwell_s: float
    checkpoint_id: str | None = None          # set when resolved from checkpoints.yaml


@dataclass(frozen=True)
class MissionConfig:
    takeoff_alt_m: float
    hover_time_s: float
    completion: Completion
    abort: AbortConfig
    home_position: tuple[float, float, float]
    home_frame: str
    waypoints: tuple[Waypoint, ...]


def load_mission_config(mission_yaml_path: str,
                        checkpoints_yaml_path: str = "") -> MissionConfig:
    """
    Parse + validate the mission YAML (MC-3). Resolve each waypoint that references a
    checkpoint_id against checkpoints_yaml_path (03-owned; NO in-package default — the caller
    passes an explicit absolute path, so the OQ-2 file-location decision is a one-line config
    change and a CWD-relative default can never resolve inconsistently).
    Fail loud (raise) on: missing required field, unknown frame, unresolvable checkpoint_id.
    No route/waypoint data is hardcoded in source (MC-3 / AC-3).
    """
    with open(mission_yaml_path) as fh:
        raw = yaml.safe_load(fh)
    checkpoints = _load_checkpoints(checkpoints_yaml_path)  # {checkpoint_id: position_enu}

    waypoints: list[Waypoint] = []
    for w in raw["waypoints"]:
        if "checkpoint_id" in w:
            cid = w["checkpoint_id"]
            if cid not in checkpoints:
                raise ValueError(f"waypoint references unknown checkpoint_id {cid!r}")
            waypoints.append(Waypoint(position_enu=checkpoints[cid], frame="enu",
                                      dwell_s=float(w["dwell_s"]), checkpoint_id=cid))
        else:
            p = w["position"]
            frame = w["frame"]
            if frame not in ("enu", "ned"):
                raise ValueError(f"waypoint declares unknown frame {frame!r}")
            waypoints.append(Waypoint(position_enu=(p["x"], p["y"], p["z"]), frame=frame,
                                      dwell_s=float(w["dwell_s"])))

    home = raw["home"]
    return MissionConfig(
        takeoff_alt_m=float(raw["takeoff_alt_m"]),
        hover_time_s=float(raw["hover_time_s"]),
        completion=Completion(**raw.get("completion", {})),
        abort=AbortConfig(**raw.get("abort", {})),
        home_position=(home["position"]["x"], home["position"]["y"], home["position"]["z"]),
        home_frame=home["frame"],
        waypoints=tuple(waypoints),
    )
```

*Traces to: MC-3, MC-5, MC-6, MC-7 · UAC-MC-3 · INF-M3.*

#### 4.2.6 PatrolMissionNode

**Type:** module (rclpy node)
**Location:** `ros2_ws/src/patrol_mission/patrol_mission/node.py`
**Boundary:** Owns every mechanism — pub/sub, the offboard keepalive heartbeat, the fixed-rate timer, the single frame-boundary call at startup. Holds no decision logic (a node that branched on mission state is a review-rejectable layer violation, §4.3).
**Dependencies:** `MissionStateMachine`, `FrameConversion`, `MissionConfig`, `px4_msgs`, `std_msgs`, rclpy.

```python
class PatrolMissionNode(Node):
    def __init__(self):
        super().__init__("patrol_mission")
        mission_yaml = self.declare_parameter("mission_yaml", "").value
        checkpoints_yaml = self.declare_parameter(
            "checkpoints_yaml", "").value     # OQ-2: no default; launch passes an explicit abs path
        self._cfg = load_mission_config(mission_yaml, checkpoints_yaml)

        # /fmu/* (px4_msgs) — §3.2. ALL use px4_qos() (best-effort + transient-local + depth-1) to
        # match PX4's uXRCE-DDS bridge; a reliable/depth-10 endpoint would not connect (see qos.py).
        self._sub_pos = self.create_subscription(VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1", self._on_pos, px4_qos())
        self._sub_bat = self.create_subscription(BatteryStatus,
            "/fmu/out/battery_status_v1", self._on_bat, px4_qos())
        self._sub_status = self.create_subscription(VehicleStatus,
            "/fmu/out/vehicle_status_v1", self._on_status, px4_qos())
        self._pub_ctrl = self.create_publisher(OffboardControlMode,
            "/fmu/in/offboard_control_mode", px4_qos())
        self._pub_sp = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", px4_qos())
        self._pub_cmd = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", px4_qos())

        # /patrol/* (std_msgs) — OQ-3 / OQ-7; profiles from qos.py
        self._pub_state = self.create_publisher(String, "/patrol/mission_state", patrol_state_qos())
        self._pub_wp = self.create_publisher(Int32, "/patrol/current_waypoint", patrol_state_qos())
        self._pub_dwell = self.create_publisher(Int32, "/patrol/dwell", patrol_event_qos())  # atomic OQ-7 trigger
        self._sub_abort = self.create_subscription(Bool, "/patrol/abort", self._on_abort, patrol_abort_qos())

        self._ekf_origin_ned = (0.0, 0.0, 0.0)   # captured once at arm
        self._state = MissionState.IDLE
        self._sm: MissionStateMachine | None = None   # built at arm, after origin is known
        self._timer = self.create_timer(0.1, self._on_tick)   # 10 Hz (A-2 keepalive rate)

    def _build_state_machine(self) -> None:
        """Resolve every waypoint ENU->NED at the SINGLE boundary (MC-7), then build the SM."""
        wps_ned = [to_ned_from_origin(w.position_enu, w.frame, self._ekf_origin_ned)
                   for w in self._cfg.waypoints]
        home_ned = to_ned_from_origin(self._cfg.home_position, self._cfg.home_frame,
                                      self._ekf_origin_ned)
        self._sm = MissionStateMachine(self._cfg, wps_ned, home_ned)

    def _on_tick(self) -> None:
        self._publish_keepalive()                       # A-2: heartbeat before/during offboard
        telem = self._build_telemetry()
        if self._sm is None and telem.armed:
            self._capture_origin(); self._build_state_machine()
        if self._sm is None:
            self._issue(Command(arm=True)); return  # state string is next_state.name, not a Command field
        prev = self._state
        self._state, cmd = self._sm.tick(self._state, telem)
        self._issue(cmd)                                # translate Command -> /fmu/in/*
        self._publish_patrol(cmd)                       # /patrol/mission_state + current_waypoint
        self._publish_dwell_event(prev, cmd)            # atomic /patrol/dwell on the DWELL rising edge
```

Topic table:

| Direction | Topic | Type | QoS | FR |
|-----------|-------|------|-----|----|
| pub | `/patrol/mission_state` | `std_msgs/String` | reliable, transient-local, depth 1 | MC-8 |
| pub | `/patrol/current_waypoint` | `std_msgs/Int32` | reliable, transient-local, depth 1 | MC-8 |
| pub | `/patrol/dwell` | `std_msgs/Int32` | reliable, volatile, keep-last route-covering depth | MC-8, OQ-7 |
| sub | `/patrol/abort` | `std_msgs/Bool` | `patrol_abort_qos` (reliable, volatile; latch in the state machine) | MC-6, MC-8 |
| sub | `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | `px4_qos` (best-effort, transient-local, depth 1) | MC-1/2/5 |
| sub | `/fmu/out/battery_status_v1` | `px4_msgs/BatteryStatus` | `px4_qos` (best-effort, transient-local, depth 1) | MC-6 |
| sub | `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | `px4_qos` (best-effort, transient-local, depth 1) | MC-1 |
| pub | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | `px4_qos` (best-effort, transient-local, depth 1) | MC-1 (keepalive) |
| pub | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | `px4_qos` (best-effort, transient-local, depth 1) | MC-1/2 |
| pub | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | `px4_qos` (best-effort, transient-local, depth 1) | MC-1/6 |

The arrival/capture trigger (OQ-7) is the atomic `/patrol/dwell` event — one `std_msgs/Int32` (the dwelled waypoint index) published on the rising edge into `DWELL`, so 04 never correlates the two separate, non-atomic `mission_state` + `current_waypoint` topics (which can interleave across DDS topics; see OQ-7). `mission_state` / `current_waypoint` stay the observable surface.

*Traces to: MC-1, MC-2, MC-6, MC-8 · UAC-MC-1/2/6/8 · INF-M2.*

#### 4.2.7 Launch entry-points

**Type:** config
**Location:** `ros2_ws/src/patrol_bringup/launch/mission_basic.launch.py`, `ros2_ws/src/patrol_bringup/launch/mission_patrol.launch.py`
**Boundary:** Wiring only — start the node with the right YAML parameter; `mission_patrol` additionally `IncludeLaunchDescription`s 05's `record.launch.py`. No logic.
**Dependencies:** `PatrolMissionNode`, 05 recorder include.

```python
# mission_basic.launch.py  (MC-1)
def generate_launch_description():
    return LaunchDescription([
        Node(package="patrol_mission", executable="patrol_mission", name="patrol_mission",
             parameters=[{"mission_yaml": PathJoinSubstitution(
                 [FindPackageShare("patrol_bringup"), "config", "mission_basic.yaml"])}]),
    ])

# mission_patrol.launch.py  (MC-2; resiliently includes 05 recorder — exit item 1)
def generate_launch_description():
    return LaunchDescription([
        # checkpoints_yaml default "" (OQ-2; 03's deliverable). It is OPTIONAL at the launch layer —
        # the fail-loud is downstream in load_mission_config (ValueError on an unresolved checkpoint_id),
        # not a required launch arg. patrol_mission.yaml uses checkpoint_id waypoints, so in practice an
        # absolute path is supplied: checkpoints_yaml:=/abs/path.
        DeclareLaunchArgument("checkpoints_yaml", default_value=""),
        # record defaults FALSE until 05 (patrol_logging) lands in-tree; pass record:=true to attach it.
        DeclareLaunchArgument("record", default_value="false"),
        Node(package="patrol_mission", executable="patrol_mission", name="patrol_mission",
             parameters=[{"mission_yaml": PathJoinSubstitution(
                 [FindPackageShare("patrol_bringup"), "config", "patrol_mission.yaml"]),
                 "checkpoints_yaml": LaunchConfiguration("checkpoints_yaml")}]),
        # Resilient include: OpaqueFunction resolves at launch time so an absent 05 is skip-with-warning,
        # not a hard PackageNotFoundError that would ground the patrol (Hermes PR #8; design §4.4.5).
        OpaqueFunction(function=_maybe_record),  # includes patrol_logging/record.launch.py iff record:=true AND installed
    ])
```

*Traces to: MC-1, MC-2, MC-8 · UAC-MC-1/2.*

#### 4.2.8 Test suites

**Type:** tests
**Location:** `tests/unit/` (ROS-free), `tests/integration/` (SITL)
**Boundary:** Layer-A unit imports the rclpy-free mission core directly (`qos.py` via the rclpy stub) with the PX4 interface mocked (<5 s, ≥85% — the simulator is **not** mocked because no flight dynamics are involved in unit logic); integration drives `mission_*.launch.py` against real SITL nightly (don't mock the simulator).
**Dependencies:** all components; SITL (integration only).

*Traces to: MC-4, MC-9, MC-10 · UAC-MC-4/9/10 · INF-M1.*

#### 4.2.9 Inventory Triangle Check

The component inventory (§4.2.1) lists six **core** components — `MissionStateMachine`, `FrameConversion`, `MissionConfig`, `PatrolMissionNode`, Launch entry-points, Test suites — plus three supporting single-source-of-truth modules (`commands.py`, `topics.py`, `qos.py`) the node composes but that are not architectural nodes in the §4.2.2 dependency diagram. The six core components, the dependency diagram, and the consumer surface (`tick()` + `/patrol/*` + `mission_*.launch.py` + the mission YAML schema + the `MissionStateMachine` contract) all enumerate the same six. No drift.

### 4.3 Layer View

#### 4.3.1 Layer Mapping

| Layer | Components | Key Responsibilities |
|-------|-----------|----------------------|
| Config | MissionConfig, Launch entry-points | Versioned route/params; wiring |
| Decision (ROS-free) | MissionStateMachine, FrameConversion | Every mission decision + the single frame conversion; pure, deterministic, no rclpy |
| Plumbing | PatrolMissionNode | pub/sub, keepalive heartbeat, timer, frame-boundary call |
| Flight substrate (consumed) | PX4 via `/fmu/*` (01) | Offboard control loop |
| Test | Test suites | Unit (Layer A) + integration (SITL nightly) |

#### 4.3.2 Decision layer — Design notes

**Conventions:** pure functions / frozen dataclasses; clock injected via `telem.now_s`; raises on bad input (fail loud). **New in this design:** the `tick()` contract + the single frame-conversion site. **Integration points:** the node injects `Telemetry` and consumes `Command`; nothing in this layer imports rclpy.

#### 4.3.3 Plumbing layer — Design notes

**Conventions:** follows the `px4_ros_com` `offboard_control.py` keepalive pattern (A-2). **New in this design:** the 10 Hz timer that drives `tick()` and translates `Command` → `/fmu/in/*`. **Integration points:** a node that branched on mission state would be a layer violation — all branching lives in the decision layer.

### 4.4 Systemic / Platform Interfaces

#### 4.4.1 Interface Integration Summary

| Interface | Current State (§3) | Design Changes | Priority |
|-----------|--------------------|----------------|----------|
| Messaging | `/fmu/*` exists (01); no `/patrol/*` | Add 4 `/patrol/*` `std_msgs` topics (incl. the atomic `/patrol/dwell` OQ-7 trigger) | P1 |
| Observability | None for missions | `mission_state` + `current_waypoint` ARE the observable surface (Foxglove-renderable); `/patrol/dwell` is the discrete capture event | P1 |
| Configuration | None for missions | New mission YAML; consume `checkpoints.yaml` (03) | P1 |
| CI / Test | ADR-0002 two-layer + nightly SITL | Add the unit suite + 85% gate to Layer A; add SITL scenarios nightly | P1 |
| Security | N/A | `[OOS]` — single-host pre-hardware sim | — |

#### 4.4.2 Messaging

`std_msgs` was chosen (OQ-3) so MCAP records and Foxglove renders the topics with no custom-type plugin. QoS: `/patrol/mission_state` and `/patrol/current_waypoint` reliable + transient-local depth-1 (a late subscriber, e.g. 04 or 05 starting after the node, sees the latest value); `/patrol/dwell` (the atomic OQ-7 capture event) reliable + volatile, keep-last with a route-covering depth so each checkpoint event is delivered once and never coalesced to "latest"; `/patrol/abort` reliable + volatile (so a plain `ros2 topic pub` is QoS-compatible — the abort sticks via the state machine's `_NON_ABORTABLE` latch, not topic durability). An absent subscriber is harmless. If `/patrol/abort` is never published, the low-battery abort still works (the two abort paths are independent in `_abort_reason`).

**Failure mode:** a subscriber that never connects causes no node error — publishers fire regardless; the mission flies whether or not anyone is listening.

#### 4.4.3 Security

`[OOS: single-host pre-hardware sim — no network exposure, no users, no tenancy, no credentials; the PRD declares no security FR.]` `/patrol/abort` is deliberately unauthenticated in sim (empty threat model) and is the wiring point for an authenticated hardware abort trigger later (MC-11); there are no privilege boundaries to escalate across.

**Failure mode:** N/A by scope.

#### 4.4.4 CI / Test infra

Layer-A coverage `source` is the whole `patrol_mission` package (path-based), with **only** `node.py` omitted; the aggregate ≥85% floor therefore measures every other module — the rclpy-free core `{state_machine, frames, config, commands, topics}` plus `qos.py` (which imports rclpy but is exercised through `tests/unit/test_node_glue.py`'s rclpy stub). The rclpy-free modules import directly so Layer A needs no ROS toolchain (this resolves the ADR-0002 M3 coverage-source spike, T1.7). SITL flake never blocks a PR; quarantine-not-expand on flake (Tenet 5).

**Failure mode:** if the SITL tier flakes above budget, the scenario is quarantined (not expanded) and the per-PR gate is unaffected.

#### 4.4.5 Cross-cutting Failure Modes

| Category | Failure mode | Detection | Degraded behavior | Recovery |
|----------|--------------|-----------|-------------------|----------|
| Persistent state | Invalid / missing mission YAML | `load_mission_config` raises at startup | Node refuses to start — never flies a bad config | Fix the YAML; restart launch |
| Network dependency | Stale / lost `vehicle_local_position` | Timestamp age on the cached message | Hold the last setpoint + keep keepalive alive; do **not** advance the waypoint; log | Resume on fresh telemetry, or operator publishes `/patrol/abort` |
| Network dependency | SITL / uXRCE-DDS agent unreachable | No `vehicle_status` (never `armed`) | Stays in IDLE/ARMING; never arms | Bring the agent/SITL up; node arms on first `armed` |
| Network dependency | Offboard exits mid-flight (keepalive gap) | PX4 nav_state leaves offboard | PX4 failsafe takes over; surfaced on `/patrol/mission_state` | PX4-side recovery; mission does not silently continue |
| Plugin / extension | 05 recorder include absent | Launch include missing | Mission flies, no bag produced (non-critical to flight) | Land 05; re-run with `record:=true` |
| Identity provider | (none) | — | `[OOS: no identity provider in single-host sim]` | — |
| Mesh / cross-cluster | (none) | — | `[OOS: single host, no mesh]` | — |

Note: a low-battery crossing is the **intended** `ABORT → RTH`, not a fault — it is listed under §4.5 Sequence 4, not here.

#### 4.4.6 Consumed cross-docset contracts (provisional)

| Contract | Owner | Default | Status |
|----------|-------|---------|--------|
| Checkpoint mapping `sim/config/checkpoints.yaml` | 03 | `{checkpoint_id, position{x,y,z} ENU, tag_family, tag_id}`; 02 reads `position` → NED at §4.2.4, references `checkpoint_id` | Schema confirmed (COMBINED-REVIEW #5); file location open per 03 DoD §7 — path parameterized |
| Capture trigger | 02↔04 | 02 emits the atomic `/patrol/dwell` event (`std_msgs/Int32` index) once per DWELL entry; 04 captures once per event | Confirmed (COMBINED-REVIEW #2); superseded 2026-06-21 per PR #8 (atomic event replaces the `DWELL` + `current_waypoint` correlation) |

### 4.5 Key Interaction Sequences

#### Sequence 1: Basic mission (arm → takeoff 5 m → hover 10 s → land) — happy path (MC-1)

```
Operator        Launch          PatrolMissionNode        MissionStateMachine      PX4 SITL (/fmu/*)
  │               │                    │                        │                      │
  ├─ ros2 launch ─►│                    │                        │                      │
  │  mission_basic │── start node ─────►│                        │                      │
  │               │                    ├─ 10Hz: keepalive ──────────────────────────────►│
  │               │                    ├─ arm cmd ───────────────────────────────────────►│
  │               │                    │◄────────────────── armed + offboard_active ─────┤
  │               │                    ├─ tick(IDLE/ARMING) ───►│ (build SM @ origin)     │
  │               │                    │◄── (TAKEOFF, sp=5m) ───┤                         │
  │               │                    ├─ trajectory_setpoint(NED, -5) ──────────────────►│
  │               │                    │   ... within 0.5 m for 2 s (MC-5) ...            │
  │               │                    │◄── (HOVER) ────────────┤                         │
  │               │                    │   ... hold takeoff pt for hover_time_s=10 s ...  │
  │               │                    │◄── (LANDING, land) ────┤  (no waypoints)         │
  │               │                    ├─ land cmd ──────────────────────────────────────►│
  │               │                    │◄────────────────── disarmed ────────────────────┤
  │               │                    │◄── (DONE) ─────────────┤                         │
```

#### Sequence 2: Patrol waypoint → tolerance+hold → DWELL (arrival signal → 04) → next → RTH (MC-2/MC-5/OQ-7)

```
PatrolMissionNode        MissionStateMachine       /patrol/*           04 (perception)
        │                        │                    │                     │
        ├─ tick(WAYPOINT i) ────►│                    │                     │
        │◄── (WAYPOINT, sp=wp_i)─┤                    │                     │
        │   ... _within_tolerance_for_hold(wp_i) true (0.5 m / 2 s) ...     │
        │◄── (DWELL, wp=i) ──────┤                    │                     │
        ├─ pub mission_state="DWELL", current_waypoint=i (observable) ─────►│ │
        ├─ pub /patrol/dwell = i  (atomic capture event, rising edge) ─────►│ (capture once)
        │   ... hold wp_i for dwell_s ...             │                     │
        │◄── (WAYPOINT i+1) ─────┤                    │                     │
        │   ... last waypoint done ...                │                     │
        │◄── (RTH) ──────────────┤                    │                     │
```

#### Sequence 3: RTH via explicit home-waypoint offboard sequence then land (OQ-8)

```
PatrolMissionNode        MissionStateMachine        PX4 SITL
        │                        │                      │
        │◄── (RTH, sp=home_ned)──┤                      │
        ├─ trajectory_setpoint(home_ned) ─────────────►│   (NO offboard->RTL handoff —
        │   ... _within_tolerance_for_hold(home) ...   │    control authority stays in SM)
        │◄── (LANDING, land) ────┤                      │
        ├─ land cmd ───────────────────────────────────►│
        │◄── (DONE) ─────────────┤◄──── disarmed ───────┤
```

PX4 RTL mode was rejected: it removes control authority from the state machine at the mode handoff and is not unit-testable. The explicit home-waypoint sequence keeps both.

#### Sequence 4: External-signal abort mid-patrol (AC-6); low-battery path identical

```
Operator        /patrol/abort      PatrolMissionNode      MissionStateMachine     PX4 SITL
  │                  │                    │                       │                   │
  ├─ pub abort=True ►│── (latched) ──────►│                       │                   │
  │                  │                    ├─ tick(WAYPOINT) ──────►│ (guards FIRST)    │
  │                  │                    │◄── (ABORT) ───────────┤ EXTERNAL_SIGNAL   │
  │                  │                    ├─ pub mission_state="ABORT" (observable, recorded by 05)
  │                  │                    ├─ tick(ABORT) ─────────►│                   │
  │                  │                    │◄── (RTH, sp=home) ────┤                   │
  │                  │                    ├─ trajectory_setpoint(home) ──────────────►│
  │                  │                    │   ... → LANDING → DONE (Sequence 3) ...    │
```

Low-battery abort follows the identical ABORT → RTH path, driven by `battery_remaining < 0.20`. The SITL-observable half of low-battery is best-effort (SITL battery modeling); the transition is always unit-tested (AC-7).

### 4.6 Data Model Changes (Consolidated)

No database. New checked-in YAML config:

| File | Change | Detail |
|------|--------|--------|
| `patrol_bringup/config/patrol_mission.yaml` | **New** | Full patrol route (4+ waypoints, dwell, completion, abort, home) — MC-3 |
| `patrol_bringup/config/mission_basic.yaml` | **New** | Minimal basic-mission params (takeoff_alt, hover_time, no waypoints) — MC-1 |
| `sim/config/checkpoints.yaml` (03-owned) | **Consumed read-only** | `{checkpoint_id, position ENU, tag_family, tag_id}` (provisional OQ-2) |

New in-memory dataclasses (runtime only): `Telemetry`, `Command`, `_Progress`, `Waypoint`, `Completion`, `AbortConfig`, `MissionConfig`.

### 4.7 UX Mocks

`[OOS: no graphical UI; the operator interface is the CLI launch command; the observable surface is `/patrol/*` rendered in Foxglove (owned by 05).]` The operator state surface for `ros2 topic echo /patrol/mission_state` over a patrol run:

| Order | `mission_state` | `current_waypoint` | Meaning |
|-------|-----------------|--------------------|---------|
| 1 | `IDLE` → `ARMING` | -1 | Pre-arm / arming |
| 2 | `TAKEOFF` | -1 | Climbing to `takeoff_alt_m` |
| 3 | `HOVER` | -1 | Holding takeoff point for `hover_time_s` |
| 4 | `WAYPOINT` | 0 | Flying toward waypoint 0 |
| 5 | `DWELL` | 0 | Arrived + dwelling at waypoint 0 (04 captures) |
| … | `WAYPOINT`/`DWELL` | 1, 2, 3 | Each subsequent waypoint |
| n-2 | `RTH` | -1 | Returning to home waypoint |
| n-1 | `LANDING` | -1 | Descending |
| n | `DONE` | -1 | Disarmed on the ground |

---

## 5. Design Questions FAQ

### Q1: Main components and interactions

Six components (§4.2): `MissionStateMachine` (decisions), `FrameConversion` (single ENU↔NED site), `MissionConfig` (YAML loader/schema), `PatrolMissionNode` (rclpy plumbing), Launch entry-points (wiring), Test suites. The node drives the state machine at 10 Hz: build `Telemetry` from cached `/fmu/out/*`, call `tick()`, translate the returned `Command` to `/fmu/in/*`, publish `/patrol/*`. Build order: state-machine + frames + config + unit suite (M1) → node + `mission_basic` + basic SITL (M1) → waypoint/dwell/abort/RTH + `mission_patrol` + patrol SITL (M2).

### Q2: Core API contracts and data models

The single public class contract is `tick(current_state, telemetry) -> (next_state, command)` (§4.2.3). Topic contracts: 3 `/patrol/*` `std_msgs` topics + 6 `/fmu/*` `px4_msgs` topics (paths/types in §4.2.6, identical to §3.2). Data models are the frozen dataclasses in §4.2.3 / §4.2.5. The owned YAML schema is in Appendix C.2; the consumed checkpoints schema is in Appendix C.1. No REST, no DB.

### Q3: Deployment and infrastructure dependencies

Runs inside 01's containers and `ros2_ws`. New `patrol_mission` colcon package + `patrol_bringup` launch/config. Coverage `source` = the whole `patrol_mission` package minus `node.py` (§4.4.4). SITL scenarios added to the nightly job, never per-PR. No new runtime dependency (hand-rolled state machine, OQ-1). PyYAML is expected transitively via the ROS 2 Jazzy base (`python3-yaml`), verified pre-M1 in §3.5 row 6, with a `package.xml` `<exec_depend>` fallback.

### Q4: External components and interfaces

PX4 / `px4_msgs` / `/fmu/*` from 01; the checkpoint mapping + world + camera from 03; the recorder include from 05 (non-critical to flight). Each dependency is captured in §3.5 (verified preconditions) or §4.4.6 (consumed contracts).

### Q5: Testing strategy (unit, integration, E2E)

Unit (Layer A, ROS-free): all `tick()` transitions, all four abort guards, `_within_tolerance_for_hold` (advances on hold, never on equality), `FrameConversion` known input→NED, `MissionConfig` parse + fail-loud + `checkpoint_id` resolution — <5 s, ≥85%, PX4 mocked, simulator **not** mocked. SITL (nightly): the basic scenario + the reduced 2-waypoint patrol + the external-abort assertion. These map to §6.2 task tables exactly.

### Q6: Security implications and auth interactions

OOS by design (§4.4.3) — single-host pre-hardware sim, no auth surface, no privilege boundaries. `/patrol/abort` is intentionally unauthenticated and is the wiring point for a later authenticated hardware trigger (MC-11).

### Q7: Technical risks and open questions

1. **Offboard keepalive (A-2)** — proven in the M1 skeleton first, so the highest-uncertainty plumbing risk surfaces day one, not late.
2. **SITL flakiness (OQ-5)** — small/strict/nightly/quarantine-not-expand; the runtime budget is provisional until measured (re-measure → MZ).
3. **Frame-conversion sprawl (MC-7)** — the single §4.2.4 boundary; a second site is review-rejectable.

Open: OQ-2 and OQ-7 remain provisional pending the combined human review and match §2 exactly (no §2/Q7 drift). They were confirmed at the combined review (2026-06-03).

---

## 6. Implementation Plan

### 6.0 Linear Project

**Project:** [Patrol Drone 02 Mission Control](https://linear.app/wemodulate/project/patrol-drone-02-mission-control-539c82e00210) (Swarm; id `7dd1c39c-718e-4655-a8e4-046a57080a87`)
**Team:** Swarm
**Initiative:** Patrol Drone — Phase 1 simulation
**Created from:** Section 6 of this document.

Per the Linear materialization conventions (COMBINED-REVIEW), this is a **per-docset-local** Linear project with local milestone numbering. The PRD/DoD references to plan milestones **M3–M4** are master-plan traceability links, not Linear milestones. The local milestones are **M1, M2, MZ**:

- **M1** = plan M3, skeleton (basic mission node, takeoff and land).
- **M2** = plan M4, layer 1 (multi-waypoint patrol + safety + config + topics).
- **MZ** = terminal catch-all (deferred OQs/spikes, e2e/integration test expansion, final documentation + test consolidation).

Each milestone's Definition of Done includes a lightweight documentation true-up (§6.4); MZ holds the comprehensive final documentation + test consolidation (§6.5). The §6.2 task list below maps 1:1 to the already-created Linear issues.

### 6.1 Milestone Overview (walking-skeleton)

| # | Milestone | Type | Shippable Demo | Scope | Dependencies | Exit Criteria | Linear |
|---|-----------|------|----------------|-------|-------------|---------------|--------|
| M1 | Basic mission node — takeoff and land | skeleton | Stakeholder runs `ros2 launch patrol_bringup mission_basic.launch.py` and watches the SITL drone arm, climb to 5 m, hover 10 s, and land | Full stack crossed at minimum thickness: `MissionStateMachine` basic states, the single frame boundary, minimal config, the unit suite (≥85%, <5 s), the node + offboard keepalive + `/fmu/*` wiring, `mission_basic.launch.py`, one basic SITL test | 01 green (A-1) | AC-1, AC-4, AC-5 (basic) pass; the demo runs end-to-end; offboard-keepalive risk surfaced and retired | *bootstrapped* |
| M2 | Multi-waypoint patrol mission | layer 1: patrol + safety + config + topics | Stakeholder runs `ros2 launch patrol_bringup mission_patrol.launch.py` and watches a 4+ waypoint patrol with dwell, return-to-home, land, and a mid-patrol external abort | WAYPOINT/DWELL/ABORT states + all 4 abort guards, full YAML schema + `checkpoint_id` resolution, RTH offboard sequence, `/patrol/*` topics, abort/waypoint/RTH unit tests, `mission_patrol.launch.py` + 05 include, patrol SITL test + external-abort assertion | M1 | AC-2, AC-3, AC-6, AC-7, AC-8, AC-9 pass; the patrol demo runs; mission-flight portion of exit item 1 demonstrated | *bootstrapped* |
| MZ | Consolidation & deferred backlog | terminal | No new flight capability; the project's deferred items are cleared or explicitly punted to Phase 2 | OQ-5 SITL runtime re-measure, e2e/integration test expansion, final documentation + test consolidation | M1, M2 | MZ reviewed and either cleared or items explicitly punted; "project done" requires this | *bootstrapped* |

### 6.2 Milestone Details

#### M1: Basic mission node — takeoff and land

**Type:** skeleton
**Goal:** The thinnest end-to-end offboard flight — arm, takeoff to 5 m, hover 10 s, land — crossing every layer at minimum thickness. The offboard-keepalive risk (A-2) is surfaced and retired here, not late.
**Shippable demo:** A stakeholder runs `ros2 launch patrol_bringup mission_basic.launch.py` against a running SITL drone and watches it arm, climb to 5 m AGL, hold ~10 s, and land.
**Dependencies:** 01 green (A-1); `px4_msgs` vendored (§3.5 row 3); PyYAML importable (§3.5 row 6).
**Exit criteria:** AC-1 (basic flight observable in SITL), AC-4 (unit suite ≥85% in <5 s, no ROS), AC-5 (basic SITL integration test) pass; the demo runs end-to-end.

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| Multi-waypoint sequencing, dwell, RTH | design §4.2.3 (WAYPOINT/DWELL/RTH states) | M2 |
| Abort guards (external / low-battery / scaffolded) | design §4.2.3 `_abort_reason`; MC-6/MC-9/MC-11 | M2 |
| `/patrol/*` topics | design §4.2.6; MC-8 | M2 |
| Full mission YAML schema + `checkpoint_id` resolution | design §4.2.5; MC-3 | M2 |
| SITL runtime/flakiness budget measurement | design OQ-5 (provisional) | MZ |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T1.1 | `MissionStateMachine` — basic states arm/takeoff/hover/land (hand-rolled per OQ-1; MC-4) | `patrol_mission/state_machine.py` (new) | MissionStateMachine | Decision | M | — | *bootstrapped* |
| T1.2 | Coordinate-frame conversion boundary world/ENU → PX4 NED (MC-7) | `patrol_mission/frames.py` (new) | FrameConversion | Decision | S | — | *bootstrapped* |
| T1.3 | Minimal mission config parse/validate (`mission_basic.yaml`; MC-3) | `patrol_mission/config.py` (new); `patrol_bringup/config/mission_basic.yaml` (new) | MissionConfig | Config | M | — | *bootstrapped* |
| T1.4 | State-machine unit suite (≥85%, <5 s, no ROS; AC-4) | `tests/unit/test_state_machine.py` (new); `tests/unit/test_frames.py` (new); `tests/unit/test_config.py` (new) | Test suites | Test | M | T1.1, T1.2, T1.3 | *bootstrapped* |
| T1.5 | `PatrolMissionNode` + offboard keepalive + `/fmu/*` wiring (MC-1) | `patrol_mission/node.py` (new); `patrol_mission/setup.py` (modify) | PatrolMissionNode | Plumbing | L | T1.1, T1.2, T1.3 | *bootstrapped* |
| T1.6 | `mission_basic.launch.py` (MC-1/AC-1) | `patrol_bringup/launch/mission_basic.launch.py` (new) | Launch entry-points | Config | S | T1.5 | *bootstrapped* |
| T1.7 | Basic SITL integration test + coverage-source resolution (AC-5) | `tests/integration/test_mission_basic.py` (new); CI `coverage` config (modify); `.github/workflows/sitl-nightly.yml` (modify) | Test suites | Test | M | T1.5, T1.6 | *bootstrapped* |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | `state_machine`, `frames`, `config`, `commands` | arm→takeoff→hover→land transitions; tolerance+hold advances on hold and never on equality (MC-5); ENU→NED known input→output; minimal config parse + fail-loud; `VehicleCommand` ordering + warmup gating |
| Integration | Launch → Node → SITL | `mission_basic.launch.py` drives arm→takeoff 5 m→hover 10 s→land; assert state progression to DONE |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| README docset-02 section | developers | How to run `mission_basic.launch.py`; what the basic mission demonstrates |
| Changelog / Design true-up | future devs | M1 reconciliation: PRD/Design/DoD/README vs. what was built |

#### M2: Multi-waypoint patrol mission

**Type:** layer 1: patrol + safety + config + topics
**Goal:** Thicken the skeleton to a real patrol — visit 4+ waypoints in order with dwell, return home, land — plus the abort safety floor and the downstream `/patrol/*` contracts.
**Shippable demo:** A stakeholder runs `ros2 launch patrol_bringup mission_patrol.launch.py` and watches a 4+ waypoint patrol with dwell at each, return-to-home, land, and a mid-patrol external abort that drives an observable return-home.
**Dependencies:** M1.
**Exit criteria:** AC-2 (patrol visits all waypoints, RTH, land), AC-3 (config from checked-in YAML), AC-6 (external-signal abort observable in SITL), AC-7 (low-battery abort unit-tested), AC-8 (all abort transitions unit-covered), AC-9 (tolerance+hold completion) pass; the patrol demo runs; exit item 1 mission-flight portion demonstrated.

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| SITL runtime/flakiness budget re-measure | design OQ-5 (provisional ≤8 min/scenario) | MZ |
| Manual-takeover abort *trigger* in SITL | design §4.2.3 (scaffold only); PRD Out-of-Scope | Phase 2+ (hardware RC) |
| Timeout abort *trigger* in SITL | design §4.2.3 (scaffold only); PRD Out-of-Scope | Phase 2+ (hardware RC) |
| Bag recording mechanics / manifest / Foxglove | design §4.7; owned by 05 | Phase 1 (05) |
| The world / checkpoint models / camera topic | design §4.4.6; owned by 03 | Phase 1 (03) |
| e2e/integration test expansion beyond the two canonical scenarios | design §6.5 | MZ |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T2.1 | WAYPOINT/DWELL/ABORT states + all 4 abort guards (MC-6/MC-9) | `patrol_mission/state_machine.py` (modify) | MissionStateMachine | Decision | L | M1 | *bootstrapped* |
| T2.2 | Full mission YAML schema + `checkpoint_id` resolution vs 03's `sim/config/checkpoints.yaml` (MC-3/MC-5) | `patrol_mission/config.py` (modify); `patrol_bringup/config/patrol_mission.yaml` (new) | MissionConfig | Config | M | M1 | *bootstrapped* |
| T2.3 | RTH — explicit home-waypoint offboard sequence (no PX4 RTL; OQ-8) | `patrol_mission/state_machine.py` (modify); `patrol_mission/node.py` (modify) | MissionStateMachine, PatrolMissionNode | Decision/Plumbing | M | T2.1 | *bootstrapped* |
| T2.4 | `/patrol/*` topics `std_msgs`: `mission_state`=String, `current_waypoint`=Int32, `dwell`=Int32, `abort`=Bool (OQ-3); the atomic `/patrol/dwell` event drives 04 (OQ-7) | `patrol_mission/node.py` (modify) | PatrolMissionNode | Plumbing | M | T2.1 | *bootstrapped* |
| T2.5 | Abort / waypoint / RTH unit tests (AC-7, AC-8) | `tests/unit/test_state_machine.py` (modify); `tests/unit/test_abort.py` (new) | Test suites | Test | M | T2.1, T2.3 | *bootstrapped* |
| T2.6 | `mission_patrol.launch.py` + 05 recorder include (MC-2/AC-2, exit item 1) | `patrol_bringup/launch/mission_patrol.launch.py` (new) | Launch entry-points | Config | S | T2.2, T2.4 | *bootstrapped* |
| T2.7 | Patrol SITL test + external-abort assertion (AC-5/AC-6) | `tests/integration/test_mission_patrol.py` (new); `.github/workflows/sitl-nightly.yml` (modify) | Test suites | Test | M | T2.6 | *bootstrapped* |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | `state_machine`, `config` | all waypoint transitions + dwell; all four abort guards (external, low-battery, manual-takeover scaffold, timeout scaffold); RTH sequence; full YAML parse + `checkpoint_id` resolution + fail-loud |
| Integration | Launch → Node → SITL | `mission_patrol.launch.py` visits all waypoints in order, dwells, RTH, lands; external-abort published mid-patrol drives observable RTH (AC-6) |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| Mission YAML schema doc | developers / operators | The owned `patrol_mission.yaml` schema (Appendix C.2); how to author a route |
| `/patrol/*` contract note | 04 / 05 maintainers | Topic names/types/QoS; the atomic `/patrol/dwell` capture-event trigger semantic (OQ-7) |
| Design true-up | future devs | M2 reconciliation: PRD/Design/DoD/README vs. what was built |

#### MZ: Consolidation & deferred backlog (terminal)

**Type:** terminal
**Goal:** Clear (or explicitly punt to Phase 2) the items that surfaced during M1–M2 but were not blocking. No new flight capability.
**Shippable demo:** N/A (terminal consolidation milestone) — the demo is "the M2 patrol still runs end-to-end after consolidation."
**Dependencies:** M1, M2.
**Exit criteria:** MZ reviewed and either cleared or items explicitly punted to Phase 2; "project done" requires this.

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| New mission capability of any kind | design §6.5 (MZ is consolidation-only) | Phase 2 |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| MZ.1 | SITL runtime/flakiness budget re-measure (OQ-5) — replace the provisional ≤8 min/scenario with a measured wall-clock figure once 01's SITL has landed | `.github/workflows/sitl-nightly.yml` (modify); design §2/OQ-5 (modify) | Test suites | Test | M | M2 | *bootstrapped* |
| MZ.2 | e2e/integration test-suite expansion beyond the two canonical scenarios | `tests/integration/**` (new) | Test suites | Test | M | M2 | *bootstrapped* |
| MZ.3 | Documentation + test consolidation (final true-up) — comprehensive PRD/Design/DoD/README reconciliation + test cleanup | `docs/phase1/02-mission-control/**` (modify); `tests/**` (modify) | — | — | M | M2 | *bootstrapped* |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Integration | Expanded SITL set | Re-measured runtime budget holds; expanded scenarios pass nightly without raising the flake rate above budget |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| Final documentation consolidation | all | Comprehensive PRD/Design/DoD/README true-up; deferred-item disposition (cleared vs. punted to Phase 2) |

### 6.3 Layered Delivery Sequence

**Skeleton + layering rationale:**

1. **M1 (skeleton, basic mission node)** is the thinnest end-to-end slice: arm → takeoff → hover → land driven by one launch command. The path crosses every layer at minimum thickness — config (`mission_basic.yaml`), decision (`MissionStateMachine` basic states + the frame boundary), plumbing (`PatrolMissionNode` + keepalive + `/fmu/*`), and test (unit + one basic SITL). After M1 a stakeholder can demonstrate programmatic offboard flight. The riskiest plumbing — offboard keepalive (A-2) — is surfaced here, day one, not deferred.
2. **M2 (layer 1: patrol + safety + config + topics)** thickens the skeleton with the real patrol: waypoint sequencing + dwell, the full YAML schema, RTH, all four abort guards, and the `/patrol/*` downstream contracts. After M2 the demo shows a full 4+ waypoint patrol with a mid-patrol abort returning home. Why this layer next: it is exactly the integrative-exit mission-flight behavior (exit item 1) and the safety floor the project most needs bought down in sim.
3. **MZ (terminal, consolidation)** adds no flight capability; it clears the deferred backlog (OQ-5 re-measure, test expansion, final docs).

**What gets demoable, when:**
- After M1: programmatic arm→takeoff→hover→land from one command.
- After M2: M1 demo + a full multi-waypoint patrol with dwell, RTH, land, and observable mid-patrol abort.
- After MZ: full design with deferred items cleared/punted.

**Scope-shedding plan:**
- If schedule slips, shed M2 — M1 alone (basic offboard flight) is still a shippable demo.
- The **abort paths are a hard floor**: even if SITL observability slips, the abort transitions remain unit-tested (AC-7, AC-8) regardless. They are never shed.

**Parallel work opportunities:**
- Within M1, T1.2 (frames) and T1.3 (minimal config) are independent of T1.1 (state machine) and can proceed concurrently; T1.4 (unit suite) depends on all three.
- Within M2, T2.2 (YAML schema) is largely independent of T2.1 (states) and can proceed alongside it until they meet at T2.5.

### 6.4 Definition of Done (every milestone)

A milestone is complete when:
- [ ] All tasks are implemented and code-reviewed.
- [ ] Unit suite passes (≥85% coverage, <5 s, no ROS/Gazebo/PX4) and the milestone's SITL scenario(s) pass.
- [ ] **Shippable demo runs end-to-end** (M1: thin path across every layer; M2: skeleton + the patrol/safety/topics layer).
- [ ] **Documentation trued up** — PRD/Design/DoD/README reconciled against what was actually built this milestone.
- [ ] No P1 bugs remain.
- [ ] Systemic interfaces (§4.4 messaging, observability, CI) are integrated per Section 4.4.

### 6.5 MZ (terminal milestone)

MZ is the catch-all for work that surfaces during M1–M2 but is not blocking. It is seeded with:

- **OQ-5 SITL-runtime budget re-measure** — replace the provisional `≤8 min/scenario` with a measured wall-clock figure once 01's SITL has landed (MZ.1).
- **e2e / integration test-suite expansion** beyond the two canonical scenarios (MZ.2).
- **A comprehensive documentation + test consolidation pass** — the final true-up across PRD/Design/DoD/README plus test cleanup (MZ.3).
- Any tech-debt / hardening items logged during the M1–M2 build.

**Exit:** MZ is reviewed and either cleared or its items are explicitly punted to Phase 2 — "project done" requires this.

---

## 7. Changelog

### v0.3.0 — 2026-06-03

**Full-depth regeneration from the PRD (sole source).** Replaces the condensed v0.2.0 body with the complete design written inline — real ASCII component + sequence diagrams, full Python dataclass / function-signature code blocks, and complete traceability / interface / milestone-task tables. **No scope or decision change:** every FR (MC-1…MC-11), every OQ resolution, the §6 milestone plan (M1/M2/MZ), and the §6.0 Linear link/id are preserved exactly as ratified at the combined review (2026-06-03). The header keeps **Status: Approved (combined review 2026-06-03; bootstrapped to Linear)** and the Requirements-source / Upstream lines.

**Sections modified:** all (summaries expanded to full content); decisions unchanged.

### v0.2.0 — 2026-06-03

**Self-review revision (ReviewDesign + ReviseDesign, auto-pilot).** Two findings ≥ medium resolved, no scope change. (D13) Added a §3.5 Verified-Precondition row for PyYAML with command + pre-M3 gate + fallback; softened Q3 prose. (D2/D4 cross-docset) OQ-2 now states explicitly that *two* parts are pending — schema (matches 03 DoD §5) and file location (open per 03 DoD §7) — and the path is parameterized in `MissionConfig`; cascaded through §3.5, §4.2.5, §4.4.6, Appendix C.1.

### v0.1.0 — 2026-06-03

**Initial version** via SoftwareDesign/CreateDesign from the PRD (sole source). Resolved OQ-1/3/4/5/6/8; carried OQ-2 + OQ-7 as Resolved (combined review 2026-06-03). Walking-skeleton plan. No scope beyond the PRD FR table (MC-1…MC-11).

---

## Appendix A: Workstream Overview

### A1. Mission Control

**Priority:** P1 | **Wave:** 2 | **Estimate:** ~22 ew | **Plan milestones:** M3–M4 (local M1–M2 + MZ)

The ROS-free `MissionStateMachine` + thin `PatrolMissionNode` + `patrol_bringup` launch/config. Owns the mission/route YAML schema, `mission_*.launch.py`, `/patrol/{mission_state,current_waypoint,abort}`, the `MissionStateMachine` `tick()` contract, and the checkpoint-arrival capture-trigger semantic. Depends on 01; consumes 03/05; consumed by 03/04/05 + Phase 2+.

**Key Issues:** Linear project [Patrol Drone 02 Mission Control](https://linear.app/wemodulate/project/patrol-drone-02-mission-control-539c82e00210); milestones M1, M2, MZ; tasks T1.1–T1.7, T2.1–T2.7, MZ.1–MZ.3.

**Dependencies:**
- 01-platform green (`/fmu/*`, `px4_msgs`, package shells, container build).
- 03 `sim/config/checkpoints.yaml` (consumed, OQ-2).
- 05 recorder include (consumed by `mission_patrol.launch.py`).

---

## Appendix B: User Acceptance Criteria

PRD Appendix B (UAC-MC-1…10) is the authoritative UAC set and is not duplicated here; it is mapped to components in §4.1 and exercised in §6.2. The inferred requirements this design adds:

**INF-M1: Deterministic clock-injected state machine** *(ref: UAC-MC-4)*
GIVEN the `MissionStateMachine` with the clock injected via `telemetry.now_s`
WHEN the unit suite advances time by passing successive `Telemetry` values
THEN tolerance+hold and dwell timing are exercised deterministically with no real wall-clock dependency.

**INF-M2: Offboard keepalive is node-only plumbing** *(ref: UAC-MC-1)*
GIVEN the offboard control mode requires a continuous setpoint stream (A-2)
WHEN the node runs its 10 Hz timer
THEN the keepalive heartbeat is published from the node, never from the state machine, preserving the ROS-free decision layer.

**INF-M3: Fail-loud config validation** *(ref: UAC-MC-3)*
GIVEN a mission YAML with a missing field, an unknown frame, or an unresolvable `checkpoint_id`
WHEN `load_mission_config` runs at startup
THEN it raises and the node refuses to start, so a bad config never flies.

---

## Appendix C: Consumed + owned schemas

### C.1 Consumed checkpoint-positions YAML (03, OQ-2, provisional — schema + location)

Owned by 03-sim-environment; consumed read-only by 02. 02 reads `position` (→ NED at §4.2.4) and references `checkpoint_id` from the mission YAML. The physical file location is open per 03 DoD §7; the path is a `MissionConfig` loader parameter so an agreed-different location is a one-line config change.

```yaml
# sim/config/checkpoints.yaml  (03-owned)
- checkpoint_id: "cp_north"
  position: { x: 10.0, y: 0.0, z: 2.0 }   # world/ENU meters
  tag_family: "tag36h11"
  tag_id: 0
- checkpoint_id: "cp_east"
  position: { x: 0.0, y: 10.0, z: 2.0 }
  tag_family: "tag36h11"
  tag_id: 1
```

### C.2 Owned `patrol_mission.yaml` (MC-3)

Every waypoint carries a frame (directly via `frame`, or implicitly ENU via a `checkpoint_id` reference) so the MC-7 conversion boundary always knows the source frame. No route data is hardcoded in source.

```yaml
# patrol_bringup/config/patrol_mission.yaml  (02-owned)
takeoff_alt_m: 5.0
hover_time_s: 10.0
completion:
  tolerance_m: 0.5      # OQ-4 default (overridable)
  hold_time_s: 2.0      # OQ-4 default (overridable)
abort:
  low_battery_threshold: 0.20   # OQ-6 (battery_status.remaining fraction)
home:
  position: { x: 0.0, y: 0.0, z: 2.0 }
  frame: "enu"
waypoints:
  - checkpoint_id: "cp_north"   # resolved from sim/config/checkpoints.yaml (ENU)
    dwell_s: 3.0
  - checkpoint_id: "cp_east"
    dwell_s: 3.0
  - position: { x: -10.0, y: 0.0, z: 2.0 }   # inline waypoint
    frame: "enu"
    dwell_s: 3.0
  - position: { x: 0.0, y: -10.0, z: 2.0 }
    frame: "enu"
    dwell_s: 3.0
```

The minimal `mission_basic.yaml` (MC-1) carries only `takeoff_alt_m`, `hover_time_s`, `completion`, `abort`, and `home` (no waypoints):

```yaml
# patrol_bringup/config/mission_basic.yaml  (02-owned)
takeoff_alt_m: 5.0
hover_time_s: 10.0
completion: { tolerance_m: 0.5, hold_time_s: 2.0 }
abort: { low_battery_threshold: 0.20 }
home: { position: { x: 0.0, y: 0.0, z: 2.0 }, frame: "enu" }
waypoints: []
```
