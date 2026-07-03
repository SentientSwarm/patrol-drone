# LR-8 — End-to-end single-artifact verification

The single-artifact guarantee (docset 05-logging-replay, M8 / LR-8, DoD AC-8, exit-checklist #1):
**one** bag from **one** patrol carries through every stage — record → upload → manifest → replay →
Foxglove — with no manual stitching. The automated component tests (TS-6/14/18/19) prove each stage;
this procedure witnesses them composing on a single real artifact.

Run it once before closing M8 (and again whenever the pipeline materially changes). It is a
witnessed procedure, not an automated test, because the Foxglove render (AC-6) is a human visual
check of a desktop app.

## Preconditions

- The sim stack runs (PX4 SITL + Gazebo + uXRCE-DDS agent) — see the M1/M2 bring-up.
- `git lfs` installed; the repo checked out with LFS (only needed to *regenerate* the reference bag,
  not for this run).
- A stand-in DGX target dir (a local path stands in for the DGX — OQ-7), e.g. `/tmp/dgx_landing`.
- Foxglove Studio installed (desktop app).

## Procedure

### 1. Record — run one patrol; get one identified bag

Use `scripts/run_patrol_world_sitl.sh` (the M5 stage runner: brings up the world + camera + QGC and
flies the patrol with `record:=true`). **On a host where the sim runs faster than real-time (camera
reads > ~20 Hz), set `PX4_SIM_SPEED_FACTOR=0.33` so it holds RTF ≈ 1 (camera ~15 Hz)** — otherwise the
record path double-delivers frames and step 4's consistency guard hard-fails (ADR-0013):

```bash
source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
export DISPLAY=:1
PX4_SIM_SPEED_FACTOR=0.33 scripts/run_patrol_world_sitl.sh --skip-doctor
# (or the bare launch, if the stack is already up: ros2 launch patrol_bringup mission_patrol.launch.py record:=true)
```

When the patrol completes, confirm exactly one new bag in the run output dir
(`patrol_<missionId>_<timestamp>/`) with **both** its `metadata.yaml` (rosbag2 finalize marker) **and**
its sibling `<bag>.meta.json` sidecar. Capture the bag path as `$BAG`. (AC-1/AC-2 — PASS.) If the
runner hangs on "tearing down", kill the leftover `QGroundControl` process; the bag is already
finalized by then.

### 2. Upload — the daemon ships it to the stand-in within 30 s

```bash
# In a second terminal, before/while the mission ends. upload_daemon lives under analysis/ — put it
# on PYTHONPATH (run from repo root):
PYTHONPATH=analysis python -m upload_daemon --watch ~/patrol_bags --target /tmp/dgx_landing/ --transport rsync
```

Confirm the bag + sidecar appear under `/tmp/dgx_landing/` within ~30 s of mission end (LR-3 / AC-3).

### 3. Ingest + manifest — index it and query it back

```bash
# `ingest` shells out to `ros2 bag info` to DERIVE the bag facts, so ROS must be sourced first
# (a plain / uv-.venv shell fails with "skipping un-indexable bag"). Use python3, not python.
source /opt/ros/jazzy/setup.bash
# `--watch` is a DAEMON (infinite poll loop, no one-shot flag): run it, wait for the
# "ingest ... indexed <bag>" line, then Ctrl-C. ingest lives under docker/ (PYTHONPATH, from repo root):
PYTHONPATH=docker python3 -m ingest --watch /tmp/dgx_landing --db /tmp/dgx_manifest/bag_manifest.db
#   → wait for:  ingest ... indexed patrol_<...>    then press Ctrl-C
PYTHONPATH=docker python3 -m ingest.manifest_query --recent 1 --db /tmp/dgx_manifest/bag_manifest.db
```

Confirm the query returns the bag's row with mission / time / **duration derived from the bag** /
topic set / metadata (LR-4 / AC-4). The duration + topic counts must match `ros2 bag info $BAG`
(the dumb-producer invariant — facts come from the bag, not the sidecar).

### 4. Replay — the same bag passes the regression assertions

```bash
# Witness the assertions on THIS live bag (not the checked-in reference). Run under system python
# with ROS sourced (NOT the uv .venv — the numpy/uv boundary in CLAUDE.md):
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 tests/replay/verify_live_bag.py --bag "$BAG"
```

Confirm every asserted topic (mission_state, current_waypoint, checkpoint_capture, camera, one
fmu/out) is present at its expected rate (LR-5 / AC-5). The checked-in reference bag is a trimmed
slice of exactly this kind of run.

**Why not just eyeball `ros2 bag info` Hz?** On a GUI-loaded host the sim can't hold real-time; the
record path may double-deliver rendered frames and write an inconsistent MCAP summary, so a raw
`ros2 bag info` count / duration reads ~2x high (measured: camera 30.3 Hz vs a true 15.15 Hz) and
would trip the band with a *false* failure ([ADR-0013](../docs/decisions/0013-m8-live-bag-rate-witness.md)).
`verify_live_bag.py` avoids this two ways: it first runs a **consistency guard** (it hard-fails,
exit 2, if `ros2 bag info` counts disagree with the actual message stream or a topic's frames are
duplicated — "re-record once the sim holds real-time"), then measures each topic's **true** rate from
its own de-duplicated message timestamps (RTF-invariant), not from wall-clock. A clean bag on a
loaded host still passes; a suspect bag never silently passes. The CI reference-bag lane is
unaffected — it runs at RTF ≈ 1.0 on a clean bag.

### 5. Foxglove — the same bag renders

Open `$BAG`'s `.mcap` in Foxglove Studio with the saved layout
(`analysis/foxglove/patrol_layout.json`). Confirm the **camera feed**, **mission state**, and
**3D pose history** panels populate (LR-6 / AC-6).

## Pass criteria (AC-8)

ONE `$BAG` satisfied steps 1–5 with no manual editing of the artifact between stages:

- [x] step 1 — one identified MCAP bag + sidecar produced
- [x] step 2 — that bag on the stand-in target ≤ 30 s after mission end
- [x] step 3 — that bag returned by `manifest_query` with derived facts
- [x] step 4 — that bag passes `verify_live_bag.py` (consistency guard OK + RTF-robust rate PASS)
- [x] step 5 — that bag renders in Foxglove with all panels populated

Record the witnessed `$BAG` name + date here when run:

> _Witnessed: `patrol_20260703T070221Z_20260703_070221` on 2026-07-03 — Egemen Cankaya._
> Recorded at RTF ≈ 1 (`PX4_SIM_SPEED_FACTOR=0.33`, camera 15.16 Hz); metadata.yaml + `.meta.json`
> sidecar both finalized (no reindex); upload `is_complete=True`; manifest `169s / 34 topics` matches
> `ros2 bag info` (dumb-producer); `verify_live_bag.py` exit 0 (consistency OK, rate PASS); Foxglove
> panels render. checkpoint_capture Count 3 (perception captured 3 of 4 checkpoints — witness requires
> ≥1; the 4th-capture gap is a separate perception-timing follow-up).
