# sim — Simulation assets

Gazebo Harmonic worlds, models, and any PX4 SITL configuration overrides.

## Contents

- `worlds/` — custom Gazebo worlds (patrol environments). Used in Milestone M5.
- `models/` — custom models: AprilTag fiducials, checkpoint markers, simple obstacles.
- `px4_sitl_overrides/` — any PX4 SITL parameter overrides or airframe customizations.

## Conventions

- Worlds are SDF format (`.sdf` or `.world`).
- Models follow Gazebo's model directory structure (model.sdf + model.config + meshes/).
- Don't check in large binary meshes — use simple primitives or external mesh references where possible.

See the Phase 1 plan, Milestone M5, for the first world we need: flat terrain with 3+ AprilTag checkpoints at known positions.
