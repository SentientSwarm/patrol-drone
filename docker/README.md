# docker — Container definitions

Two containers for development, plus one for the DGX-side ingestion service.

## Containers

### `sim/` — Simulation environment
Ubuntu 24.04 + ROS 2 Jazzy + PX4 SITL + Gazebo Harmonic + uXRCE-DDS Agent + the project workspace. This is what runs locally for development and in CI for integration tests.

### `dev/` — Interactive development
Same base as `sim/`, plus editor/debugger tooling. Source tree mounted as a volume. Day-to-day code work happens here.

### `ingest/` — DGX-side bag ingestion service
Lightweight container running on the DGX (or any backend). Watches an upload location, indexes bags into the manifest, makes them queryable. Built in Phase 1 Milestone M8.

## Conventions

- Base images: `ros:jazzy-ros-base-noble` or build from `ubuntu:24.04` with explicit ROS 2 install.
- NVIDIA Container Runtime for GPU access (when needed).
- `docker compose up` from the repo root should produce a working sim environment.
- Don't put secrets in Dockerfiles — use `.env` files (which are gitignored).

See the Phase 1 plan, "Containerization" section, for the design rationale.
