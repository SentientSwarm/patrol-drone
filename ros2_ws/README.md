# ros2_ws — ROS 2 workspace

Standard colcon workspace. Build from this directory:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Planned packages (under `src/`)

To be created during Milestone M3 of Phase 1 (`ros2 pkg create ...`). See the [Phase 1 plan](../docs/phase1_simulation_plan.md) for the milestone breakdown.

| Package | Type | Purpose |
|---|---|---|
| `patrol_mission` | Python (ament_python) | Mission state machine, waypoint nav, abort logic |
| `patrol_perception` | Python (ament_python) | Perception nodes (AprilTag, image capture, later YOLO and anomaly) |
| `patrol_interfaces` | C++ (ament_cmake) | Custom messages, services, actions (`CheckpointCapture`, etc.) |
| `patrol_bringup` | Python (ament_python) | Launch files, configs, params |

## External dependencies (under `src/external/`)

Vendored, version-pinned dependencies. Plan:

- `px4_msgs` — PX4 message definitions, pinned to the PX4 firmware version we build against. Branch correspondence matters; do not unpin.

## Architectural notes

- **Mission state machine lives as a plain Python class** in `patrol_mission`, separate from the ROS node. This keeps it unit-testable without spinning up ROS. See the Phase 1 plan M3 section for the rationale.
- **Custom interfaces in their own package** from day one. Even if `patrol_interfaces` starts empty, putting messages there later won't require a workspace-wide rebuild.
