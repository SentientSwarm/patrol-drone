# patrol-drone

Autonomous drone system for indoor and outdoor patrol on private property — including forested terrain — built on PX4 + ROS 2 + Jetson, with a long-horizon track toward embodied AI and learned navigation.

## Where to start

If you're new to this project, read in this order:

1. **[Project plan](docs/autonomous_drone_patrol_project_plan_v2.md)** — the master architecture, hardware BOMs, phase plan (8 phases), reference implementations, and the "Distro and OS decision" rationale. ~30 minutes.
2. **[Phase 1 simulation plan](docs/phase1_simulation_plan.md)** — the executable plan for Phase 1 (pre-hardware simulation work). This is where active development starts. ~20 minutes.
3. **[ADRs](docs/decisions/)** — captured non-obvious technical decisions. Read these for context on *why* things are the way they are.

## Project status

**Phase 1 — Pre-hardware simulation foundation.** No hardware purchased yet; that decision is gated on Phase 1 exit criteria (see Phase 1 plan).

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
