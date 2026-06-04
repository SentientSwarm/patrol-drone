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

> **M2 adds:** ROS 2 Jazzy + the uXRCE-DDS bridge (live `/fmu/*` topics), the vendored `px4_msgs`
> workspace with a green `colcon build`, and the `sim`/`dev` containers — extending this path into
> the full setup-to-running-mission spine (≤20 commands).

## Stack at a glance

| Layer | Choice |
|---|---|
| OS | Ubuntu 24.04 |
| Middleware | ROS 2 Jazzy |
| Flight stack | PX4 v1.16.x or latest stable |
| Simulator | Gazebo Harmonic |
| Bridge | uXRCE-DDS (native) |
| Bags | rosbag2 + MCAP |
| Visualization | Foxglove Studio |

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
