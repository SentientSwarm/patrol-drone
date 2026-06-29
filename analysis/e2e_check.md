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

```bash
ros2 launch patrol_bringup mission_patrol.launch.py record:=true
```

When the patrol completes, confirm exactly one new bag in the output dir
(`~/patrol_bags/patrol_<missionId>_<timestamp>/`) with its `<bag>.meta.json` sidecar.
Capture the bag path as `$BAG`. (AC-1/AC-2 — already PASS, ADR-0011.)

### 2. Upload — the daemon ships it to the stand-in within 30 s

```bash
# In a second terminal, before/while the mission ends:
python -m upload_daemon --watch ~/patrol_bags --target /tmp/dgx_landing/ --transport rsync
```

Confirm the bag + sidecar appear under `/tmp/dgx_landing/` within ~30 s of mission end (LR-3 / AC-3).

### 3. Ingest + manifest — index it and query it back

```bash
python -m ingest --watch /tmp/dgx_landing --db /tmp/dgx_manifest/bag_manifest.db   # one-shot or daemon
python -m ingest.manifest_query --recent 1 --db /tmp/dgx_manifest/bag_manifest.db
```

Confirm the query returns the bag's row with mission / time / **duration derived from the bag** /
topic set / metadata (LR-4 / AC-4). The duration + topic counts must match `ros2 bag info $BAG`
(the dumb-producer invariant — facts come from the bag, not the sidecar).

### 4. Replay — the same bag passes the regression assertions

```bash
# Point the replay test at THIS bag (not the checked-in reference) to witness it on the real artifact:
#   the assertions.yaml subset must be present at rate.
ros2 bag play "$BAG"   # while the replay counter subscribes — or run the regression lane locally
```

Confirm every asserted topic (mission_state, current_waypoint, checkpoint_capture, camera, one
fmu/out) is present at its expected rate (LR-5 / AC-5). The checked-in reference bag is a trimmed
slice of exactly this kind of run.

### 5. Foxglove — the same bag renders

Open `$BAG`'s `.mcap` in Foxglove Studio with the saved layout
(`analysis/foxglove/patrol_layout.json`). Confirm the **camera feed**, **mission state**, and
**3D pose history** panels populate (LR-6 / AC-6).

## Pass criteria (AC-8)

ONE `$BAG` satisfied steps 1–5 with no manual editing of the artifact between stages:

- [ ] step 1 — one identified MCAP bag + sidecar produced
- [ ] step 2 — that bag on the stand-in target ≤ 30 s after mission end
- [ ] step 3 — that bag returned by `manifest_query` with derived facts
- [ ] step 4 — that bag passes the replay assertions
- [ ] step 5 — that bag renders in Foxglove with all panels populated

Record the witnessed `$BAG` name + date here when run:

> _Witnessed: `<bag name>` on `<date>` — pending first full-stack run._
