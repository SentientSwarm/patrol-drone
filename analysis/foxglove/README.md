# Foxglove — opening a recorded patrol bag

Foxglove Studio is the visualizer for the MCAP bags M7 records. It is an **installed desktop app**
— it is not built or containerized by this repo (settled constraint: plan §"Containerization", DoD
§6). This directory holds a saved layout and the load-and-render procedure that satisfies the M7
Foxglove check (DoD AC-6 / LR-6).

## Prerequisites

- Foxglove Studio installed (desktop app) — see the pinned stack in
  [`stack-manifest.toml`](../../stack-manifest.toml) / `CLAUDE.md`.
- A recorded bag from a patrol run: `patrol_<missionId>_<timestamp>.mcap` (default output dir
  `~/patrol_bags/`). Produce one with:

  ```bash
  ros2 launch patrol_bringup mission_patrol.launch.py \
      checkpoints_yaml:=/abs/path/checkpoints.yaml          # record:=true is the default (M7)
  ```

## Inspect a bag from the CLI (`ros2 bag info`)

```bash
ros2 bag info ~/patrol_bags/patrol_<missionId>_<timestamp>     # point at the bag DIRECTORY
```

Point `ros2 bag info` at the **bag directory**, not a glob: `~/patrol_bags/patrol_*` also matches the
sibling `<bag>.meta.json` sidecar, so `ros2 bag info` gets two arguments and errors with
`unrecognized arguments`. List directories first if unsure: `ls -d ~/patrol_bags/patrol_*/`.

## Open the bag

1. Launch Foxglove Studio.
2. **Open local file…** → select the `.mcap` from `~/patrol_bags/` (Foxglove reads MCAP natively).
3. **Layouts → Import from file…** → `analysis/foxglove/patrol_layout.json` (this directory).

## What should render (the LR-6 check)

With `patrol_layout.json` applied, the bag should populate:

| Panel | Topic(s) | What you should see |
|-------|----------|---------------------|
| **Camera** (Image) | `/drone/camera/image_raw/compressed` | the drone's RGB feed scrubbing with the timeline |
| **Mission state** (Raw Messages) | `/patrol/mission_state` | the state string (TAKEOFF → WAYPOINT → DWELL → … → LAND) |
| **3D pose history** (3D) | `/tf`, `/tf_static`, `/fmu/out/vehicle_local_position*` | the drone's trajectory + frames over the patrol |
| **Checkpoint captures** (Raw Messages) | `/patrol/checkpoint_capture` | one `CheckpointCapture` per checkpoint visit (carries `image_path`) |

The M7 check passes when the camera feed, mission state, and 3D pose history render with these panels
populated. (Manual / E2E tier — not a CI gate; see `docs/phase1/05-logging-replay/design.md` §4.2.6.)

## Notes

- `/drone/camera/image_raw/compressed` is the compressed companion the camera bridge publishes
  (`camera_bridge.launch.py`); the bag records the compressed topic to keep size reasonable.
- `/patrol/checkpoint_capture` carries `string image_path`, not pixels — the captured PNG/JPEG lives
  on disk (04-perception's output dir), referenced by the message. Open the referenced file
  alongside the bag if you want the full-resolution capture.
- The saved layout is operator convenience; nothing in the pipeline depends on it.
