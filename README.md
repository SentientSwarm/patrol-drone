# patrol-drone

Autonomous drone system for indoor and outdoor patrol on private property — including forested terrain — built on PX4 + ROS 2 + Jetson, with a long-horizon track toward embodied AI and learned navigation.

## Where to start

If you're new to this project, read in this order:

1. **[Project plan](docs/autonomous_drone_patrol_project_plan_v2.md)** — the master architecture, hardware BOMs, phase plan (8 phases), reference implementations, and the "Distro and OS decision" rationale. ~30 minutes.
2. **[Phase 1 simulation plan](docs/phase1_simulation_plan.md)** — the executable plan for Phase 1 (pre-hardware simulation work). This is where active development starts. ~20 minutes.
3. **[ADRs](docs/decisions/)** — captured non-obvious technical decisions. Read these for context on *why* things are the way they are.

## Project status

**Phase 1 — Pre-hardware simulation foundation.** No hardware purchased yet; that decision is gated on Phase 1 exit criteria (see Phase 1 plan).

## Quickstart — M1 bring-up (flying SITL)

The fastest path from a clean machine to a drone hovering in simulation. This is the **M1** slice:
vanilla PX4 SITL flying in Gazebo Harmonic, flown manually from QGroundControl. ROS 2, the live
`/fmu/*` telemetry bridge, and the containerized build land in **M2** — see the [Phase 1 plan](docs/phase1_simulation_plan.md).

**Prerequisites:** native Ubuntu 24.04 (not WSL2), an **Xorg/X11** login session (Gazebo Harmonic
rendering is unreliable under Wayland — verify with `echo $XDG_SESSION_TYPE`, expect `x11`), and a
discrete GPU recommended. Versions are pinned in the [stack manifest](stack-manifest.toml) (`stack-manifest.toml`).

```bash
# 1. Clone the repo
git clone https://github.com/<owner>/patrol-drone.git
cd patrol-drone

# 2. Install the full Phase 1 toolchain (idempotent; also fetches & preps PX4-Autopilot).
#    See --help for opt-outs; --disable-wayland forces Xorg at the GDM level.
scripts/setup_phase1.sh

# 3. Build and launch PX4 SITL with the x500 quad in Gazebo Harmonic.
#    First run builds PX4 from source — do NOT interrupt it (several minutes).
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 4. In a second terminal, launch the ground station.
~/Apps/QGroundControl-x86_64.AppImage
```

Then, in QGroundControl (AutoConnect → UDP must be on; it listens on UDP 14550): **Arm**, hit
**Takeoff**, and watch the drone hold altitude for **60 continuous seconds**, then **Land**. That is
the M1 exit criterion (AC-1 / PLAT-1).

## Quickstart — M2 bring-up (live `/fmu/*` bridge)

**M2** adds ROS 2 Jazzy + the native uXRCE-DDS bridge: PX4 telemetry shows up as live ROS 2
topics. The containerized path below reaches a running sim and proves the bridge in a handful of
commands (continues the M1 spine, within the ≤20-command Phase 1 budget). Versions all resolve
from the [stack manifest](stack-manifest.toml) — `gen_build_args.py` turns it into the compose
build args (written to `.env.build`, kept separate from the secret-bearing `./.env`).

```bash
# (from a clean checkout; Docker came from scripts/setup_phase1.sh)
git clone https://github.com/<owner>/patrol-drone.git && cd patrol-drone   #  1–2
scripts/gen_build_args.py --env > .env.build                               #  3  manifest → compose ARGs
docker compose --env-file .env.build build sim dev                         #  4  sim + dev from one base
docker compose --env-file .env.build up -d sim                             #  5  PX4 SITL + Gazebo + agent
# `exec` starts a fresh shell that does NOT run the entrypoint, so source ROS + the workspace
# overlay first (the bridge process itself is already sourced by the entrypoint):
docker compose --env-file .env.build exec sim bash -c \
    'source /opt/ros/jazzy/setup.bash && source /opt/ros2_ws/install/setup.bash \
     && ros2 topic list | grep fmu'                                        #  6  bridge up (PLAT-2)
docker compose --env-file .env.build exec sim bash -c \
    'source /opt/ros/jazzy/setup.bash && source /opt/ros2_ws/install/setup.bash \
     && ros2 topic hz /fmu/out/vehicle_local_position_v1'                  #  7  steady ~50 Hz over 60 s
```

The workspace builds inside the image at build time (a single green `colcon build` over the
vendored `px4_msgs`/`px4_ros_com` and the `patrol_*` package shells). For day-to-day work, edit on
the host and rebuild in the `dev` container (source is bind-mounted):

```bash
docker compose --env-file .env.build run --rm dev colcon build             #  8  in-container build
```

> **Note (ADR-0007):** the Micro XRCE-DDS Agent is **built from source** at a pinned eProsima tag
> — there is no `ros-jazzy-micro-xrce-dds-agent` apt package in the ROS 2 Jazzy repo. Both the sim
> container and `setup_phase1.sh` build the same pinned version, so the host and container agree.

## Quickstart — M3 bring-up (basic mission: takeoff & land)

**M3** lands the first mission node. With a SITL drone + the M2 bridge up, one launch command
arms the drone, climbs to 5 m AGL, hovers 10 s, and lands:

```bash
# with `docker compose ... up -d sim` running (M2 bridge live), in a sourced shell:
ros2 launch patrol_bringup mission_basic.launch.py                         # arm → 5 m → hover 10 s → land
```

The mission *decisions* live in a ROS-free `MissionStateMachine` (plain Python, hand-rolled), so
every transition is unit-tested in <5 s with no ROS/Gazebo/PX4 — run the Layer-A suite directly:

```bash
uv run pytest tests/unit                                                   # mission core, ≥85% coverage
```

The thin `PatrolMissionNode` owns the ROS plumbing (the 10 Hz offboard keepalive + `/fmu/*`
pub/sub) and drives `tick()`; the route/params come from the checked-in
[`mission_basic.yaml`](ros2_ws/src/patrol_bringup/config/mission_basic.yaml) (no route data in
source). The full arm→takeoff→hover→land run against live SITL is exercised by the nightly SITL
job (never a per-PR gate).

## Stack at a glance

| Layer | Choice |
|---|---|
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Flight stack | PX4 — pinned in [stack-manifest.toml](stack-manifest.toml) |
| Simulator | Gazebo Harmonic |
| Bridge | uXRCE-DDS (native) |
| Bags | rosbag2 + MCAP |
| Visualization | Foxglove Studio |

Exact versions live in [stack-manifest.toml](stack-manifest.toml) — the canonical pinned-stack manifest
([ADR-0004](docs/decisions/0004-stack-manifest-location.md)); this table avoids duplicating them.
See [ADR-0001](docs/decisions/0001-distro-and-os.md) for why this stack, not the Humble + 22.04 path that some references use.

## Repo layout

```
patrol-drone/
├── ros2_ws/             # ROS 2 workspace (packages under src/)
├── sim/                 # Gazebo worlds, models, PX4 SITL overrides
├── analysis/            # Bag analysis scripts, Jupyter notebooks
├── docker/              # Container definitions (sim, dev, ingest)
├── tests/               # unit / integration / replay tests
├── scripts/             # Utility scripts
└── docs/
    ├── autonomous_drone_patrol_project_plan_v2.md
    ├── phase1_simulation_plan.md
    └── decisions/       # ADRs
```

## How to contribute

The Phase 1 doc has a "How to engage with this doc" section. Short version:
- Settled architectural calls have reasoning in the docs — engage with the reasoning, not the call.
- Open territory: test strategy, mission state machine implementation, container layout, bag schema design.
- Decisions get written down as ADRs. The bar is low — short, specific, capture context/decision/consequences.

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).
