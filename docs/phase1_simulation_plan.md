# Autonomous Drone Patrol — Phase 1 Simulation Plan

## Project context

We're building an autonomous drone system for patrolling private property — indoor spaces, outdoor perimeter, and forested trails on a wooded property. The capability set we're working toward: repeatable waypoint patrols with checkpoint inspection (image capture at known positions), anomaly detection from captured imagery, and longer-horizon experimentation with embodied AI and learned navigation policies. The system has two airframes — a 250mm indoor quad and a Holybro X500-class outdoor platform — that share a single flight stack, so software, models, and tooling transfer between them rather than fragmenting.

The architectural commitments worth knowing upfront:

- **PX4 + ROS 2 Jazzy + Pixhawk + Jetson on both platforms.** One stack, two airframes. (See "Distro and OS decision" in the master plan for why we're on Jazzy/24.04 rather than the older Humble/22.04 that some references use.)
- **Visual-inertial odometry as the primary localizer everywhere**, with GPS as augmentation when available. The forest-patrol use case forced this — GPS is unreliable under canopy, so we treat outdoor as "VIO-always, GPS-when-available" rather than GPS-first.
- **Safety-critical loop is always onboard.** VIO, obstacle avoidance, failsafes — none of these depend on the WiFi link.
- **Offboard compute (DGX-class cluster) is for training and heavy/offline inference only.** Never in the safety loop.
- **Simulation-first development.** This doc is that simulation work — Phase 1 of an 8-phase plan, and the only phase that's hardware-free.
- **Logging and replay infrastructure from day one.** Every flight (real or simulated) produces a rosbag. The bag is the regression test, the training corpus, and the debugger.

The full architecture, BOMs, phase plan, and reference implementations live in `autonomous_drone_patrol_project_plan_v2.md`. Skim that doc first if you're new to the project — it's where the hardware and architectural decisions are reasoned through. This doc is the executable plan for Phase 1 only.

## Where Phase 1 sits

Eight phases total:

1. **Phase 1 — Simulation foundation and logging pipeline (this doc).** No hardware. Software stack, mission logic, rosbag pipeline, replay infrastructure. ~8 weeks.
2. Phase 2 — Outdoor first flights (X500 + GPS, no Jetson yet)
3. Phase 3 — Onboard perception outdoor (add Jetson + RealSense + VIO)
4. Phase 4 — Indoor VIO patrol (250mm + AprilTag relocalization)
5. Phase 5 — Forest and trail navigation
6. Phase 6 — Anomaly detection
7. Phase 7 — Operational concept (charging dock, scheduling)
8. Phase 8 — Learned navigation and world models

Hardware doesn't get purchased until Phase 1's exit criteria are green. The discipline is deliberate: every Phase 2+ issue we can shake out in sim is one we don't debug on a vibrating airframe.

## How to engage with this doc

This is a working plan, not a spec. The technical stack is opinionated and the rationale for each call is in the body of the doc. The places I'd actively welcome pushback or contribution:

- **Test strategy and CI design.** The "Test strategy" section is where I'm least confident. If you have strong opinions on integration test orchestration for ROS 2 + SITL, particularly around CI runtime budgets and flakiness, I want them.
- **Mission state machine implementation.** I've specified the class shape and testability requirement; the choice of state machine library (`transitions`, `python-statemachine`, hand-rolled, or something else) is open. Strong preferences welcome.
- **Container layout.** Two-container split (`sim`, `dev`) is a defensible default, not the only answer.
- **Bag schema design.** The `CheckpointCapture` message in M6 is a first cut. Worth designing carefully because every later phase consumes it.

What I'd ask not to relitigate without strong evidence (these calls have reasoning in the body):

- ROS 2 Jazzy on Ubuntu 24.04 (Humble + 22.04 was our earlier call, deliberately reversed — see "Distro and OS decision" in the master plan)
- uXRCE-DDS native rather than MAVROS — known tradeoff, deliberate call
- MCAP bag format (sqlite is legacy)
- Gazebo Harmonic (Gazebo Classic is deprecated)
- JetPack 7.2 path for Jetson (not the older 6.x)

If you disagree with any of these, the reasoning in the doc body is the thing to engage with.

## Quick stack reference

| Layer | Choice |
|---|---|
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Flight stack | PX4 v1.16.x (or latest stable) |
| Simulator | Gazebo Harmonic |
| Bridge | uXRCE-DDS (native) |
| Mission orchestration | Python 3.12 |
| Bags | rosbag2 + MCAP |
| Visualization | Foxglove Studio |
| Containers | Docker + Docker Compose |

Full version pinning and rationale in the "Target stack" section below.

## Working agreement

A few things worth being explicit about for collaboration:

- **Branch and PR everything.** Even solo, the discipline of small PRs against a `main` that's always working-in-sim is what makes this iterable. Trunk-based, short-lived branches.
- **Tests before merge.** Unit tests for any new state machine logic; the integration test suite has to pass; new bag-producing changes get a replay regression added.
- **Decisions get written down.** If we make a non-obvious technical call, it goes in `docs/decisions/` as a short ADR (Architecture Decision Record) — context, decision, consequences. Future-us will thank us.
- **One milestone at a time.** The M1–M8 sequence in this doc is deliberately ordered. Don't skip ahead; each milestone produces an artifact the next one needs.

---

# Phase 1 — Pre-Hardware Simulation Plan

## What this is for

Get the full software stack — flight control, middleware, mission logic, perception scaffolding, logging, replay, visualization — working end-to-end against a simulated drone, before any hardware purchase. The exit state is a single, repeatable command that takes off a simulated quadrotor, flies a multi-checkpoint patrol, captures an image at each checkpoint, returns, lands, and produces a rosbag that round-trips through the DGX ingestion pipeline and renders correctly in Foxglove.

That's the bar. Hit it and you've validated the architecture and the tooling. Miss it and every problem you find on real hardware will be tangled with problems you should have found in sim.

The deliverable is software, not flight time. The next phase (outdoor first flights) starts from a working repo, not a working drone.

---

## Why Phase 1 matters more than it looks

Two reasons.

First, the test suite gets built once. Every subsequent phase — outdoor first flight, indoor VIO, forest navigation, anomaly detection — will introduce regressions. If you don't have a SITL-based test harness now, you're debugging those regressions on a vibrating airframe over your house. The simulator is the regression test environment.

Second, the logging and replay pipeline must exist before the first real flight, not after. The teams that iterate fastest on autonomy are the ones who never have to re-fly to debug. Building this in Phase 1 — when the only thing logging is a simulator and there's no field-data anxiety — is the right time.

---

## Target stack (pinned)

I'd pin versions explicitly. Sliding off "latest" mid-project costs days.

| Layer | Choice | Version |
|---|---|---|
| Host OS | Ubuntu LTS | 24.04 (Noble Numbat) |
| Robotics middleware | ROS 2 | Jazzy Jalisco |
| Flight stack | PX4-Autopilot | v1.16.x or latest stable |
| Simulator | Gazebo | Harmonic (gz-sim 8) |
| PX4 ↔ ROS 2 bridge | uXRCE-DDS | bundled with PX4 v1.14+ |
| ROS 2 PX4 messages | px4_msgs | matching PX4 branch (v1.16 adds message versioning) |
| Mission orchestration | Python | 3.12 (Ubuntu 24.04 default) |
| Bag format | rosbag2 + MCAP | mcap storage plugin |
| Visualization | Foxglove Studio | latest |
| Container runtime | Docker + Docker Compose | latest |
| Build tooling | colcon | latest |

**Ubuntu 24.04 and ROS 2 Jazzy.** This is deliberate and worth being explicit about. PX4's official docs (as of mid-2026) still recommend Humble + 22.04, but two things changed the calculus: Isaac ROS moved its recommended platform to Jazzy on 24.04 with the 4.0 release, and JetPack 7.2 (released June 2026) brought Jetson Orin NX/AGX onto 24.04. Humble EOLs May 2027 — a project of this scope will outlive it, and migrating mid-project after we have working VIO and trained models is far more expensive than absorbing the friction now. The full reasoning is in the master plan under "Distro and OS decision."

Expect ~1 week of integration friction in M1–M2 that you wouldn't have on Humble. PX4 builds on 24.04, Gazebo Harmonic is native to it, and the community has demonstrated the full PX4 + Jazzy + Gazebo Harmonic combination working end-to-end. We're early-adopter, not pioneer.

**PX4 v1.16.x or later.** v1.16 introduced message versioning, which means our ROS 2 workspace can use different `px4_msgs` definitions from the firmware without breaking. Useful insurance against version drift. Track the latest stable release.

**Gazebo Harmonic, not Gazebo Classic.** Classic is deprecated; PX4's modern integration targets Harmonic and so does NVIDIA's tooling. Harmonic also has better Vulkan support, which matters on 24.04 with modern GPUs.

**uXRCE-DDS native, not MAVROS.** The Bernas reference uses MAVROS — fine for their needs, but it's the legacy translation layer and bridge-shaped abstraction we'd have to swap out later. uXRCE-DDS gives us native ROS 2 topics from PX4 with no translation in between.

**ROS 2 Lyrical Luth note.** Lyrical Luth (May 2026 LTS, supported until 2031) is the newest LTS, but ecosystem support (Isaac ROS, third-party packages, documentation) hasn't caught up yet. Jazzy is the "modern but supported" sweet spot; revisit Lyrical Luth in 2027 as a future migration.

---

## Repo structure

Single monorepo. Easier to keep ROS 2, simulation assets, and analysis scripts in lockstep than to coordinate three repos.

```
patrol-drone/
├── ros2_ws/
│   └── src/
│       ├── patrol_mission/         # mission state machine, waypoint nav
│       ├── patrol_perception/      # perception nodes (stubs in Phase 1)
│       ├── patrol_interfaces/      # custom msgs/srvs/actions
│       ├── patrol_bringup/         # launch files, configs, params
│       └── external/
│           ├── px4_msgs/           # vendored, version-pinned to PX4
│           └── px4_ros_com/        # examples + helpers
├── sim/
│   ├── worlds/                     # custom Gazebo worlds
│   ├── models/                     # AprilTags, checkpoints, props
│   └── px4_sitl_overrides/         # airframe params, startup scripts
├── analysis/                       # bag analysis (Python, jupyter)
├── docker/
│   ├── sim/                        # PX4 SITL + Gazebo + ROS 2 container
│   ├── dev/                        # interactive dev container
│   └── ingest/                     # DGX-side bag ingestion service
├── tests/
│   ├── unit/                       # pytest, mock PX4 interface
│   ├── integration/                # spin up SITL, drive mission
│   └── replay/                     # bag-replay regression tests
├── scripts/                        # one-off utilities, setup helpers
├── docs/
└── README.md
```

Notes on the choices:

`patrol_interfaces` exists from day one even if it's empty. Custom messages always show up eventually (mission state, checkpoint metadata, anomaly events) and putting them in their own package avoids the rebuild-the-world problem later.

`px4_msgs` is vendored, not pulled at build time. PX4 message definitions drift between versions and an unpinned dependency will break a known-good build for no good reason.

`tests/` is split into three. London-style TDD wants fast unit tests on the mission state machine that don't touch ROS at all — those go in `tests/unit/` with a mock PX4 interface. Integration tests that spin up SITL go in `tests/integration/`. Replay-based regression tests (run a recorded bag through the perception stack, assert the same detections come out) go in `tests/replay/`.

---

## Milestone breakdown

Eight milestones, roughly a week each, sequenced so each one builds on a verifiable previous state. Don't skip ahead — every milestone produces an artifact (working command, passing test, recorded bag) that you'll use in the next one.

### M1 — Toolchain installed, vanilla SITL flying

**Goal:** PX4 SITL launches Gazebo Harmonic, you can arm and takeoff from QGroundControl, drone holds altitude.

The point is to validate the install, not the architecture. No ROS 2 yet.

```bash
# PX4 from source
git clone --recurse-submodules https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500
```

If the `gz_x500` target launches a drone in Gazebo Harmonic and QGroundControl (run separately) sees it, M1 is done. If the make target fails on missing Gazebo plugins or Python deps, fix that here — it will only get worse later.

**Exit:** `make px4_sitl gz_x500` cleanly launches, drone arms and takes off via QGC, hovers stably for 60 seconds.

### M2 — ROS 2 Jazzy + uXRCE-DDS bridge

**Goal:** ROS 2 sees PX4 topics. You can `ros2 topic echo /fmu/out/vehicle_local_position` and see live telemetry from the simulator.

Install ROS 2 Jazzy per the official instructions for Noble (24.04). Build `px4_msgs` in your `ros2_ws`, branched to match your PX4 version. Run the Micro XRCE-DDS Agent (typically as a separate process or container) talking to PX4 on UDP localhost.

The agent-to-PX4 connection is the single most common failure point at this stage. PX4 needs `uxrce_dds_client` running with the right transport config. In SITL it's automatic; on real hardware it's a parameter. Verify in SITL now so you know what "working" looks like.

**Exit:** with SITL running, `ros2 topic list | grep fmu` returns PX4 topics, and `ros2 topic hz /fmu/out/vehicle_local_position` shows a steady rate (typically 50 Hz).

### M3 — Python mission node, takeoff and land

**Goal:** a Python ROS 2 node that arms the drone, commands takeoff to 5m, hovers 10 seconds, lands. Triggered via `ros2 launch`.

This is where the architecture starts. Two design calls to make now:

**Mission state machine as a separate class, not embedded in the node.** The ROS plumbing (publishers, subscribers, timers) goes in the node. The state machine — what to do given current state and inputs — is plain Python in its own module. This is what makes it testable without spinning up ROS, which is the whole point of London-style TDD here.

**Use PX4's offboard control mode via the uXRCE-DDS topics directly, not MAVSDK.** MAVSDK is tempting because the Python API is ergonomic, but it's MAVLink under the hood and you'd be running a translation layer in parallel with uXRCE-DDS for the rest of the project. Skip it. The PX4 ROS 2 offboard example (`offboard_control.py` in px4_ros_com) is the right starting point.

Write unit tests for the state machine *before* the node works end-to-end. A `MissionStateMachine` class with `tick(current_state, telemetry) -> command` is straightforward to test with pytest and a mock telemetry source. Get the transitions right in unit tests, then wire it into the node.

**Exit:** `ros2 launch patrol_bringup mission_basic.launch.py` arms the SITL drone, takes off to 5m AGL, hovers 10s, lands. Unit test suite for the state machine passes. Integration test that spins up SITL and runs the launch file passes (slow, but it runs in CI).

### M4 — Multi-waypoint patrol mission

**Goal:** drone flies a sequence of waypoints in NED coordinates, hovers at each for a defined dwell time, returns home, lands. Waypoints loaded from a YAML config.

This is the first non-trivial mission logic. Important things to get right:

**Waypoint completion criterion.** "Within 0.5m of target for 2 seconds" is the right kind of test — never just "position == target." Floats don't work that way, and on hardware nothing reaches a setpoint exactly.

**Mission abort paths.** The state machine should handle low battery, manual takeover, and timeout from day one. They're trivial to add now (a few extra states) and a nightmare to bolt on later. Even if the conditions can't be triggered in SITL, the state transitions should be there.

**Coordinate frames.** PX4's offboard control uses NED relative to the EKF origin. Be explicit about which frame each waypoint is in and where conversion happens. Mistakes here are silent and infuriating.

**Exit:** patrol mission YAML with 4+ waypoints executes correctly in SITL, with dwell time and return-to-home. Mission can be aborted mid-flight via a ROS topic and drone returns home. Unit tests cover the abort logic.

### M5 — Custom world with checkpoints and AprilTags

**Goal:** a Gazebo world that represents a meaningful patrol environment, with visual checkpoint markers (AprilTags) at known positions. Drone visits them and "looks at" them (captures a camera frame).

Two pieces of work:

**The world.** Start crude. A flat plane, some building-like boxes, a few trees as obstacles. The point isn't photorealism; it's having known checkpoint positions you can patrol between. Later (Phase 5) we'll need higher-fidelity environments for trail navigation, but Isaac Sim is where that lives.

**AprilTag markers.** Tags placed as textured Gazebo models at known world positions. In sim, you don't need them for relocalization — the simulator gives you ground-truth pose. But putting them in the world now means the perception pipeline you'll build in M6 can be exercised end-to-end, and the AprilTag detection node you'll write *also* runs unmodified on real hardware in Phase 4.

The fact that AprilTag detection works the same in sim and on hardware is the kind of architectural alignment that pays for itself constantly.

**Exit:** custom world loads with 3+ checkpoint AprilTags at YAML-configured positions. Patrol mission from M4 visits each in turn. Simulated RGB camera attached to the drone publishes a ROS 2 image topic.

### M6 — Perception scaffolding, image capture at checkpoint

**Goal:** at each checkpoint, the drone captures the current camera frame, the AprilTag detection node identifies which checkpoint it is, and the (image, checkpoint_id, pose, timestamp) tuple is published as a structured message and written to disk.

The capture pipeline is the scaffold for everything that comes later. YOLO detection (Phase 3), anomaly detection (Phase 6), and any learned perception all hang off this same pattern: at a known location, with known pose, capture and tag an image.

The AprilTag detection node should be `apriltag_ros` or equivalent — don't roll your own. It's a solved problem and the ROS 2 package is good.

Define the `CheckpointCapture` message in `patrol_interfaces` now. It'll have:
- `header` (standard with frame_id and stamp)
- `checkpoint_id` (string, from AprilTag)
- `pose` (geometry_msgs/PoseStamped)
- `image` (sensor_msgs/Image, or a reference to a stored path for large images)
- `metadata` (key-value pairs, free-form)

Get the schema right now and you won't be migrating bag formats two phases from now.

**Exit:** patrol mission produces a directory of captured images with per-image metadata files, and a `/patrol/checkpoint_capture` topic shows the same data in real time. Unit tests cover the capture node's message construction.

### M7 — rosbag2 logging, MCAP format, full sensor capture

**Goal:** every mission run produces a rosbag containing all relevant topics, in MCAP format, with metadata sufficient to identify and replay it.

Critical detail: use the MCAP storage plugin, not the default sqlite3. MCAP is the format Foxglove and modern tooling target; sqlite is legacy. Install `ros-humble-rosbag2-storage-mcap`.

Topics to record (start broad, prune later):
- All `/fmu/out/*` PX4 telemetry topics
- `/patrol/*` mission and perception topics
- Camera image topic (consider compressed image to keep bag size manageable)
- TF tree
- Mission state, current waypoint, abort signals

Bag naming convention: `patrol_<missionId>_<timestamp>.mcap`. Mission ID lets you correlate bags with mission configs; timestamp prevents collisions.

Build a small Python wrapper around `ros2 bag record` that the mission launch file invokes automatically. Every mission run produces a bag, no exceptions. This is the discipline.

**Exit:** `ros2 launch patrol_bringup mission_patrol.launch.py` runs the M6 patrol and produces an MCAP bag in a known output directory. `ros2 bag info <bag>` shows expected topics and message counts. Bag size is reasonable (under a few hundred MB for a 5-minute mission).

### M8 — Replay pipeline: bag → DGX → Foxglove

**Goal:** bags produced on the dev host upload to the DGX automatically, get indexed, and can be loaded into Foxglove (running locally or remote) for inspection. A separate machine can replay the bag and get the same topic stream.

Three components:

**Upload.** A small daemon (Python, watches the output directory, syncs new bags to DGX over SSH/rsync or to S3-compatible storage). Keep it dumb. The complexity goes in the ingestion side, not the producer.

**Ingestion.** On the DGX, bags get indexed into a simple manifest — what mission, when, how long, what topics, what's in the metadata sidecar. A SQLite or DuckDB table is enough at this scale; don't reach for Postgres yet.

**Replay verification.** A test that takes a bag, replays it through `ros2 bag play`, subscribes to expected topics, and asserts they appear with expected rates. This is the foundation of replay-based regression testing for later phases.

Foxglove is the visualizer. It opens MCAP files natively and renders the camera feed, TF tree, mission state, and 3D pose history. No setup beyond the desktop install.

**Exit:** a recorded mission bag is automatically uploaded to DGX within 30s of mission end, appears in the manifest, and opens in Foxglove with all expected panels populated. The replay regression test runs in CI against a checked-in reference bag.

---

## Phase 1 docsets (PRD/Design breakdown)

This plan is the source of truth for *what* Phase 1 delivers. To take it through the prd-engine `/drive` lifecycle (PRD → Design), the eight milestones (M1–M8) are decomposed into **five PRD/Design docsets** under [`docs/phase1/`](docs/phase1/). Each docset begins with a formal Definition-of-Done brief (`dod.md`) — explicit acceptance criteria, named stakeholders, scope boundaries, and the open decisions handed to `/drive` — and then gains a `prd.md` and `design.md` as the lifecycle runs. The cross-docset traceability matrix (which docset owns which exit-checklist item, package, and interface) lives in [`docs/phase1/README.md`](docs/phase1/README.md).

**Build / dependency order:** `01 → 02 → 03 → 04 → 05`, mirroring milestone order M1→M8. Docset 01 is the foundation (everything stands on it); 05 is the terminal consumer of the others' contracts.

**Integrative exit-checklist items** are not owned solely by one docset: **item 1** (the single end-to-end patrol command) is primary to 02 but depends on 01/03/04/05; **item 10** (README setup-to-running-mission in <20 commands) is primary to 01 but spans all docsets.

| Docset | Milestones | Owns (Phase 1 exit-checklist items) | Folder |
|---|---|---|---|
| **01 — Platform & Simulation Foundation** | M1, M2 | 9; 10 (integrative) | [`docs/phase1/01-platform/`](docs/phase1/01-platform/) — [dod](docs/phase1/01-platform/dod.md) · prd · design |
| **02 — Mission Control** | M3, M4 | 2, 3, 4, 12; 1 (integrative) | [`docs/phase1/02-mission-control/`](docs/phase1/02-mission-control/) — [dod](docs/phase1/02-mission-control/dod.md) · prd · design |
| **03 — Simulation Environment & Assets** | M5 | (none solo — enables 1, 8, 11) | [`docs/phase1/03-sim-environment/`](docs/phase1/03-sim-environment/) — [dod](docs/phase1/03-sim-environment/dod.md) · prd · design |
| **04 — Perception & Checkpoint Capture** | M6 | 11 (`CheckpointCapture` message) | [`docs/phase1/04-perception/`](docs/phase1/04-perception/) — [dod](docs/phase1/04-perception/dod.md) · prd · design |
| **05 — Logging & Replay Pipeline** | M7, M8 | 5, 6, 7, 8 | [`docs/phase1/05-logging-replay/`](docs/phase1/05-logging-replay/) — [dod](docs/phase1/05-logging-replay/dod.md) · prd · design |

To trace **from a milestone to its docset**: find the milestone row above (e.g. M6 → docset 04) and open that folder. To trace **from a docset back to the plan**: each `dod.md` cites its milestone section(s) under `## 9. Traceability` and links back into this document. The 12-item [Phase 1 exit checklist](#phase-1-exit-checklist) is fully partitioned across the five docsets (every item has exactly one primary owner; items 1 and 10 are integrative), as enumerated in the table above and detailed in [`docs/phase1/README.md`](docs/phase1/README.md).

---

## Test strategy

The London-style TDD approach maps cleanly onto this stack if you're disciplined about the boundaries.

**Unit tests (fast, mock-everything).** State machine transitions, message construction, config parsing, waypoint sequencing logic, abort handling. These should run in well under a second total and don't depend on ROS, Gazebo, or PX4. Run on every commit.

**Integration tests (slow, real SITL).** Launch PX4 SITL + ROS 2 nodes via a test harness, run a mission, assert on the resulting state and bag contents. These are slow (minutes per test) and flaky if you let them be — keep the count small and the assertions strict. A handful of canonical mission scenarios is enough.

**Replay tests (medium, deterministic).** Take a known-good bag, play it into your perception nodes, assert the outputs match a saved reference. These catch regressions in detection or mission logic without needing the simulator at all. Build this as soon as you have M7 producing bags.

The boundary I'd push back on: don't try to mock the simulator. Mock the PX4 interface in unit tests (the state machine doesn't need real telemetry to test transitions), but for anything that needs the flight dynamics, use real SITL. Simulator-of-a-simulator is a bad trade.

---

## Containerization

Two containers, both based on the same Ubuntu 24.04 + ROS 2 Jazzy base:

**`sim` container.** PX4 SITL + Gazebo Harmonic + uXRCE-DDS agent + your ROS 2 workspace. This is what runs locally for development and in CI for integration tests. Should `docker compose up` and produce a working simulation environment.

**`dev` container.** Same base, plus your editor, debugger, Python tooling. Mount the source tree as a volume. This is where day-to-day code work happens.

The container discipline matters because the Jetson environment in Phase 3+ will be Ubuntu 24.04 + ROS 2 Jazzy (via JetPack 7.2), and aligning the dev environment with the deployment environment from day one avoids the "works on my machine" class of bug. Same container definitions should largely work on Jetson with the NVIDIA Container Runtime swapped in.

Don't containerize Foxglove or QGroundControl. They're desktop apps; let them be desktop apps.

---

## Dev hardware requirements

**A DGX is not required for Phase 1.** The DGX-class compute the master plan references is for Phase 6 (anomaly model training) and Phase 8 (world models / RL training), neither of which runs in Phase 1. A collaborator with a modest dev workstation or capable laptop can contribute fully.

**What Phase 1 actually demands:**

| Component | Minimum | Comfortable | Notes |
|---|---|---|---|
| CPU | Modern 6-core | 8+ cores | Gazebo physics + PX4 SITL + ROS 2 nodes is multi-process and parallelism helps |
| RAM | 16 GB | 32 GB | Gazebo + PX4 + ROS 2 + browser + IDE adds up. 16 GB is workable; 32 GB removes friction |
| GPU | 4 GB VRAM discrete | 8 GB VRAM | Gazebo Harmonic uses Vulkan rendering; a single drone in a simple world is light. RTX 3060 / 4060 is plenty |
| Disk | 256 GB free, NVMe | 1 TB NVMe | Bags grow fast; NVMe matters for rosbag write throughput, not just speed-of-life |
| OS | Ubuntu 24.04 | Ubuntu 24.04 | Native install or dual-boot. WSL2 is *not* recommended (more on this below) |

A 12 GB or 16 GB GPU is overkill for Phase 1 — useful insurance for later phases, but Gazebo will leave most of it idle. If a collaborator only has a 6 GB or 8 GB card, they're fine.

**Where this changes:** Phase 3 onward, when we start running perception models locally for development before deploying to Jetson, a beefier GPU (12 GB+) becomes useful for fitting YOLO models and Isaac Sim training environments. Phase 5+ Isaac Sim trail-environment work wants 16+ GB VRAM and benefits from an RTX 4080-class GPU upward. Phase 6 anomaly model training and Phase 8 RL is where DGX-class compute actually earns its keep.

**Native Linux vs WSL2.** Native Ubuntu 24.04 strongly preferred. WSL2 can run ROS 2 and Gazebo Harmonic, but GPU passthrough, USB device handoff (relevant in later phases for hardware bring-up), and real-time-ish kernel behavior all get awkward. If a collaborator is on Windows, dual-boot is the right answer; if on macOS, a Linux VM works for Phase 1 but won't carry forward to hardware-in-the-loop testing.

**Headless or with display.** Most of Phase 1 can run headless (CI, integration tests). For interactive development you want a display for Gazebo and Foxglove. Both work fine on a typical desktop or laptop screen.

---

## What's explicitly NOT in Phase 1

Drawing the boundary matters. The following are deferred:

- **VIO / SLAM.** Not needed in SITL — the simulator provides ground-truth pose. VIO comes in Phase 3 (outdoor, optional fusion with GPS) and is critical in Phase 4 (indoor, VIO-only).
- **Real object detection.** AprilTag identification is enough scaffolding. YOLO / TensorRT integration is Phase 3.
- **Anomaly detection models.** Phase 6.
- **Isaac Sim.** Gazebo is enough for now. Isaac Sim shows up in Phase 5+ for trail navigation training data and Phase 8 for RL.
- **Multi-drone coordination.** Not in scope at all in the current plan.
- **EKF2 tuning, VIO parameter sets, MAVROS-vs-DDS final decision.** All Phase 3/4 work. The Phase 1 commitment to uXRCE-DDS is the architectural call; the EKF2 vision parameter work happens when you have real sensors.

If you find yourself reaching for any of these in Phase 1, stop. The discipline of "make the basic patrol work end-to-end in sim first" is what makes the later phases tractable.

---

## Phase 1 exit checklist

Concrete and falsifiable. You're done with Phase 1 when all of these are true:

- [ ] `ros2 launch patrol_bringup mission_patrol.launch.py` launches SITL, runs a multi-checkpoint patrol with image capture at each, returns and lands.
- [ ] Mission config is loaded from a YAML file checked into the repo.
- [ ] Unit test suite for mission state machine passes with >80% coverage and runs in <5 seconds.
- [ ] At least one integration test that spins up SITL and runs a canonical mission passes in CI.
- [ ] Every mission run produces an MCAP rosbag in a known output location.
- [ ] Bags upload automatically to the DGX and appear in the manifest.
- [ ] A replay regression test runs a reference bag and asserts on the outputs in CI.
- [ ] Foxglove loads a recorded bag and renders the camera feed, mission state, and 3D pose history.
- [ ] Containerized dev and sim environments build from `docker compose` and a single `colcon build` succeeds inside.
- [ ] README documents the setup-to-running-mission path end-to-end in <20 commands.
- [ ] Custom `CheckpointCapture` message defined in `patrol_interfaces` and used by both the perception node and the bag pipeline.
- [ ] Mission abort (low-battery and external-signal) is implemented in the state machine, covered by unit tests, and observable in a SITL run.

That last one is the discipline check. If the abort path doesn't work in sim, it definitely won't work in flight.

---

## So what next

Phase 1 ends with a working repo and zero hardware. The buying decisions in Phase 2 (outdoor X500/S500 + Pixhawk 6X baseboard + Jetson Orin NX + RealSense D456) should not be made until the Phase 1 exit checklist is green. The cost of buying early and discovering an architectural mistake while waiting for a part to ship is high; the cost of buying late after the software stack is proven is negligible.

When Phase 1 is done, the Phase 2 work is mostly: assemble the drone, flash PX4, connect QGC, fly manually, then run the same launch files that worked in sim — first with GPS waypoints, then with the Jetson and perception stack added in Phase 3.

The simulator doesn't go away when hardware arrives. It becomes the regression environment forever.
