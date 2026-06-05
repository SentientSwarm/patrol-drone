# CLAUDE.md

Guidance for working in this repo. Read alongside `README.md` and `docs/phase1_simulation_plan.md` (the executable plan) and `docs/autonomous_drone_patrol_project_plan_v2.md` (the master architecture).

## What this is

`patrol-drone` — an autonomous drone system for indoor + outdoor (incl. forested) patrol of private property, built on **PX4 + ROS 2 + Jetson**, with a long-horizon track toward embodied AI / learned navigation. Two airframes (a 250mm indoor quad and a Holybro X500-class outdoor platform) share **one** flight stack so tooling transfers rather than fragments.

The deliverable of the current phase is **software, not flight time**. Hardware isn't purchased until Phase 1's exit checklist is green.

## Project phase & status

8-phase plan; we are in **Phase 1 — pre-hardware simulation foundation** (no hardware yet). Phase 1 has 8 milestones (M1–M8), each ~a week, strictly ordered — each produces an artifact the next consumes. **Do not skip ahead.**

- **M1 — toolchain installed, vanilla SITL flying. ✅ COMPLETE.** `make px4_sitl gz_x500` builds and launches, x500 spawns in Gazebo (NVIDIA-accelerated), QGC connects, and the exit criterion was met: stable 60 s hover via QGC Takeoff, then land.
- M2 — ROS 2 Jazzy + uXRCE-DDS bridge (PX4 topics visible in ROS 2). **Not started.**
- M3–M8 — Python mission node → multi-waypoint patrol → custom world + AprilTags → perception/image capture → rosbag2/MCAP logging → bag→DGX→Foxglove replay. See the Phase 1 plan for per-milestone goals and exit tests.

Current branch convention: `phase1/m<n>-<slug>` (e.g. `phase1/m1-host-setup`), PR'd into `main`.

## Stack (pinned) and what's installed on this host

The **canonical pinned stack manifest is [`stack-manifest.toml`](stack-manifest.toml)** (PLAT-7 / AC-8) — the single source of truth every toolchain layer's version resolves to; the README bring-up path and the container build ARGs cite it. The table below is a **human-facing summary** of that file, kept in sync per [ADR-0004](docs/decisions/0004-stack-manifest-location.md) — when a version changes, edit the `.toml` first, then reconcile here. The manifest is a *draft* at M1; M2/SWM-11 finalizes it (incl. the OQ-3 PX4 pin). The **Pinned** column is the contract; **Installed here** records this host's verified state.

| Layer | Pinned | Installed here |
|---|---|---|
| OS | Ubuntu 24.04 LTS (Noble) | 24.04.4 LTS ✓ |
| Middleware | ROS 2 Jazzy Jalisco (LTS; `ros-jazzy-desktop`) | by `setup_phase1.sh` (re-run pending on this host) |
| Build tool | colcon (`python3-colcon-common-extensions`, ROS 2 Jazzy apt) | by `setup_phase1.sh`; in-workspace `colcon build` is M2 work |
| Flight stack | PX4-Autopilot **v1.17.0** (`px4_msgs` **release/1.17**) — latest stable; **provisional pin pending [OQ-3](docs/phase1/01-platform/prd.md)** (the exact tag + matching `px4_msgs` branch settle in the M1–M2 integration spike) | **v1.17.0** at `~/PX4-Autopilot` ✓ |
| Simulator | Gazebo Harmonic (gz-sim 8.x) | gz sim 8.11.0 ✓ |
| PX4↔ROS 2 bridge | uXRCE-DDS — Micro XRCE-DDS Agent (`ros-jazzy-micro-xrce-dds-agent`, ROS 2 apt; matches the sim container), native (**not MAVROS**) | agent by `setup_phase1.sh`; bridge proven in M2 |
| Mission orchestration | Python 3.12 | 3.12.3 ✓ |
| Bags | rosbag2 + **MCAP** plugin (`ros-jazzy-rosbag2-storage-mcap`; sqlite is legacy) | plugin by `setup_phase1.sh`; M7 work |
| Visualization | Foxglove Studio (desktop) | by `setup_phase1.sh`; M8 work |
| Container runtime | Docker Engine + Compose v2 | by `setup_phase1.sh`; M2 builds containers |
| Python tooling | uv 0.11.x | uv 0.11.18 ✓ |
| GCS | QGroundControl (AppImage, latest stable) | `~/Apps/QGroundControl-x86_64.AppImage` ✓ |

As of [ADR-0003](docs/decisions/0003-phase1-bootstrap-scope.md), `setup_phase1.sh` provisions the **full Phase 1 toolchain** (not just M1). "Installed here" reflects this host's *current* state — the rows marked "by `setup_phase1.sh`" install when the (now-expanded) script is re-run. The script installs **prerequisites only**; repo **deliverables** (vendored `px4_msgs`, the containers, `colcon build`, mission code) stay milestone-owned.

Settled architectural calls (don't relitigate without strong evidence — rationale is in the docs): **Jazzy/24.04** (not Humble/22.04), **uXRCE-DDS native** (not MAVROS), **MCAP** bags, **Gazebo Harmonic** (not Classic), JetPack 7.2 path for Jetson.

## Host / environment facts (this machine)

- **GPU:** NVIDIA GeForce RTX 4070 Ti SUPER, proprietary driver 595.71.05. nvidia_drm/modeset loaded.
- **Display server: must be Xorg (X11), not Wayland.** Gazebo Harmonic rendering is unreliable under Wayland here. Verify with `echo $XDG_SESSION_TYPE` (expect `x11`) and `glxinfo | grep 'OpenGL renderer'` (expect the NVIDIA GPU). Gazebo's GUI should show on the NVIDIA GPU in `nvidia-smi`.
  - Ubuntu 24.04 GDM quirk: the login "gear" menu merges the two same-named `ubuntu.desktop` sessions into one **"Ubuntu"** entry and auto-picks the backend; the explicit **"Ubuntu on Xorg"** may not appear separately. On this hardware plain "Ubuntu" currently resolves to Xorg. To remove the ambiguity permanently, set `WaylandEnable=false` in `/etc/gdm3/custom.conf` (the setup script's `--disable-wayland` flag does this).
- **PX4 dir:** `~/PX4-Autopilot` (separate from this repo — not a submodule). Checked out at tag `v1.17.0`.
- **QGC:** AppImage at `~/Apps/QGroundControl-x86_64.AppImage`; run on this same machine (PX4 SITL broadcasts MAVLink on localhost only). QGC listens on UDP 14550; PX4 mavlink is on 18570/14580. AutoConnect→UDP must be on.

## Common commands

```bash
# M1 sim (builds on first run; do NOT interrupt the build)
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# Launch the ground station
~/Apps/QGroundControl-x86_64.AppImage

# Host bootstrap — installs the FULL Phase 1 toolchain by default (idempotent).
# See --help for opt-outs (--skip-ros, --skip-docker, ...) and --with-nvidia / --disable-wayland.
scripts/setup_phase1.sh

# Source ROS 2 (setup_phase1.sh adds this to ~/.bashrc; new shells get it automatically)
source /opt/ros/jazzy/setup.bash

# Python dev env (this repo's non-ROS tooling)
uv sync            # installs dev group: pytest, pytest-cov, ruff
uv run pytest      # unit tests live in tests/unit/ (fast, mock-everything)
uv run ruff check .
```

## Python / uv vs ROS 2 — important boundary

This repo's `pyproject.toml` venv is for code that does **NOT** import ROS at runtime: bag analysis, the DGX upload daemon, dev tooling. `requires-python` is pinned to **3.12** to match the system interpreter ROS Jazzy links against.

- ROS 2 provides `rclpy` and the built `px4_msgs` via the **system** Python (apt + colcon), **never pip**. Anything importing `rclpy` must run in the ROS-sourced environment.
- When M2 needs a venv that can also see system `rclpy`: `uv venv --python 3.12 --system-site-packages`.
- Keep ROS/PX4 packages out of `[dependencies]` regardless.
- Project deps are added per-milestone with `uv add` (e.g. pyyaml at M4, mcap/foxglove tooling at M7/M8). M1 needs no runtime deps. This is the deliverables side of the [ADR-0003](docs/decisions/0003-phase1-bootstrap-scope.md) prerequisites-vs-deliverables line: the *system* toolchain is front-loaded by `setup_phase1.sh`, but Python *project* deps land in `pyproject.toml` as each milestone needs them.

## Repo layout

```
ros2_ws/            # ROS 2 workspace; packages under src/ (patrol_mission, patrol_perception,
                    #   patrol_interfaces, patrol_bringup, external/{px4_msgs,px4_ros_com})
sim/                # Gazebo worlds, models (AprilTags/checkpoints), px4_sitl_overrides
analysis/           # bag analysis (Python/Jupyter)
docker/             # sim / dev / ingest container defs (Phase 1 builds these later)
tests/              # unit/ (fast, no ROS) · integration/ (real SITL) · replay/ (bag regression)
scripts/            # setup_phase1.sh, push-to-github.sh
docs/               # project plan, phase1 plan, decisions/ (ADRs)
```
Most dirs are scaffolding (`.gitkeep`) until their milestone lands. Single monorepo by design.

## Working agreement / conventions

- **Branch and PR everything**, even solo. Short-lived branches off a `main` that's always working-in-sim. Commit/push only when asked.
- **One milestone at a time.** M1→M8 is ordered deliberately; don't bolt on later-phase work (VIO, YOLO, Isaac Sim, multi-drone are explicitly out of Phase 1). This governs **deliverables** (code, messages, worlds, containers, tests committed to git), not the host **toolchain** — `setup_phase1.sh` front-loads the full Phase 1 toolchain up front ([ADR-0003](docs/decisions/0003-phase1-bootstrap-scope.md)); installing ROS ≠ doing M2.
- **Tests before merge.** Unit tests for new state-machine logic (London-style TDD: the `MissionStateMachine` is plain Python, testable without ROS); integration suite must pass; bag-producing changes get a replay regression. Target >80% state-machine coverage, unit suite <5 s.
- **Write down non-obvious decisions** as a short ADR in `docs/decisions/` (context / decision / consequences). The bar is low.
- Code style: `ruff` (line-length 100, py312 target, rules E/F/I/UP/B).

## Where to look

- `docs/phase1_simulation_plan.md` — the executable plan: milestone goals, exit tests, test strategy, containerization, hardware reqs, the full Phase 1 exit checklist. **Primary reference.**
- `docs/autonomous_drone_patrol_project_plan_v2.md` — master architecture, BOMs, 8-phase plan, "Distro and OS decision" rationale.
- `docs/decisions/` — ADRs (ADR-0001 = distro & OS).
- `scripts/setup_phase1.sh` — exactly what host setup does and what's deliberately deferred to M2.
