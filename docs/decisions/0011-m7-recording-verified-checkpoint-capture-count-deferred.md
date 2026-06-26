# ADR-0011: M7 MCAP recording verified in SITL; non-zero `/patrol/checkpoint_capture` count carried by ADR-0010's 02/M4 dwell-stability fix

**Status:** Accepted
**Date:** 2026-06-26
**Deciders:** Egemen Cankaya (project owner)

## Context

M7 (docset 05, record side) delivers automatic per-run MCAP recording: a ROS-free recorder core
(`patrol_logging/recorder.py`) plus an includable `record.launch.py` that 02's
`mission_patrol.launch.py` attaches by default (`record:=true`), so every patrol run produces exactly
one `patrol_<missionId>_<timestamp>.mcap` in a known output dir. The M7 exit criteria are DoD
AC-1 (one named MCAP bag), AC-2 (`ros2 bag info` shows expected topics with non-zero counts, size
reasonable), and AC-7 (the bag pipeline **records** `/patrol/checkpoint_capture` — the "one message,
two consumers" guarantee, item 11).

During the owner's manual M7 SITL check (the bag-producing nightly/manual tier the M7 DoD calls
out), a full multi-checkpoint patrol on `run_patrol_world_sitl.sh` produced a real MCAP bag whose
`ros2 bag info` is summarized below. This ADR records what was verified, and why AC-7's *non-zero*
`checkpoint_capture` count is carried by an already-accepted upstream deferral rather than chased
through more live iteration on the M7 branch.

### What was verified (the M7 recorder is proven end-to-end)

A full patrol (`run_patrol_world_sitl.sh`, all M4 acceptance checks PASS) produced one bag,
`patrol_patrol_20260626_080740` — `ros2 bag info`:

- **AC-1:** one MCAP bag, `Storage id: mcap`, named `patrol_<missionId>_<timestamp>` (the recorder's
  `--storage mcap` + naming contract — never sqlite3).
- **AC-2:** `Duration 142 s`, `Bag size 98.5 MiB` (hundreds-of-MB, not GB), `76,780` messages across
  **34 topics**, each expected stream non-zero:
  - the `/fmu/out/.*` regex caught the PX4 v1.17 `_v1`-suffixed surface and more —
    `vehicle_local_position_v1` 7116, `vehicle_attitude` 14232, `sensor_combined` 14231,
    `battery_status_v1` 143, `vehicle_status_v1` 306, etc. (Some `/fmu/out/*` topics legitimately
    show 0 — they are advertised by the bridge but never published in this scenario, e.g. `wind`,
    `airspeed_validated_v1`, `manual_control_setpoint`; the broad regex records what exists, "start
    broad, prune later.")
  - `/drone/camera/image_raw/compressed` 6465 (the compressed companion — imagery, LR-6/LR-7).
  - `/patrol/mission_state` 1424, `/patrol/current_waypoint` 1424, `/patrol/dwell` 4 (all four
    waypoints dwelled), `/tf` 2116, `/tag_detections` 2117.
- **AC-7 (record side):** the recorder **subscribed to `/patrol/checkpoint_capture`** (confirmed in
  the launch log: `Subscribed to topic '/patrol/checkpoint_capture'`) using the same compiled
  `patrol_interfaces/msg/CheckpointCapture` type 04 publishes — i.e. the bag pipeline is wired as the
  named second consumer. The topic appears in `ros2 bag info` with the correct type.

The recorder's launch/process plumbing was also verified directly: a clean SIGINT finalizes the MCAP
and the `OnProcessExit` handler writes the `<bag>.meta.json` sidecar (the one piece outside the
Layer-A unit suite).

### The residual finding (why AC-7's non-zero count is deferred, not chased)

In that bag, `/patrol/checkpoint_capture` has **Count: 0** despite the drone dwelling at all four
waypoints (`/patrol/dwell` 4) and apriltag detecting tags (`/tag_detections` 2117). This is **not**
an M7 recorder defect — the recorder demonstrably subscribes to and would record any message
published on the topic — and it is **not a new M6 defect**. It is exactly the gap
[ADR-0010](0010-m6-capture-verified-dwell-stability-deferred.md) already recorded and accepted:

- M6's capture pipeline is proven end-to-end (a real `cp_north` capture: image + sidecar,
  tag-resolved `checkpoint_id`, 93% confidence), so perception is not at fault.
- A **full multi-checkpoint** patrol does not reliably frame each tag during the (settle-delayed)
  capture window because of a **flight/dwell-stability problem owned by 02 (mission-control) / M4**
  (the `WAYPOINT ↔ DWELL` flap; wider tolerance made it worse, hinting at a logic interaction). When
  the drone *does* settle with a tag framed, the existing pipeline captures with no perception change.

A full-patrol M7 bag therefore lands in precisely ADR-0010's deferred scenario, so its
`checkpoint_capture` count is 0 until that 02/M4 fix lands.

## Decision

**M7 is treated as functionally complete: AC-1 and AC-2 PASS in SITL, and AC-7 is met on the record
side (the bag pipeline subscribes to and records `/patrol/checkpoint_capture` with the correct
type). The remaining AC-7 acceptance — a non-zero `checkpoint_capture` count in a recorded bag — is
carried by [ADR-0010](0010-m6-capture-verified-dwell-stability-deferred.md)'s focused 02/M4
dwell-stability fix, not chased through further blind live iteration on the M7 branch (one milestone
at a time; the fix is scoped to 02/M4, not M7).**

No M7 code change is required for the count: once the 02/M4 fix makes the drone settle so each tag is
framed, the unchanged recorder will record the captures that 04 then publishes.

## Consequences

- **M7 exit checks status:** AC-1 PASS, AC-2 PASS (SITL bag above). AC-7 PASS on the record side
  (topic subscribed/recorded, correct compiled type); its non-zero-count acceptance is deferred to
  the ADR-0010 02/M4 fix. AC-3…AC-6 belong to M8 (upload → DGX → manifest → replay → Foxglove) and
  are out of M7 scope.
- **No silent gap:** the moment the 02/M4 dwell-stability fix lands and a full patrol frames every
  tag, re-running the same `mission_patrol.launch.py` (record on by default) yields a bag with
  `checkpoint_capture` > 0 — confirmable with `ros2 bag info` and no M7 change. This ADR carries the
  gap forward rather than leaving it implicit.
- **Recorded-topic-set evidence for M8:** the verified topic list + counts above are the canonical
  basis for M8's reference bag and replay-assertion subset (design §4.2.5 picks
  `mission_state`/`current_waypoint`/`checkpoint_capture`/compressed-camera/`vehicle_local_position`)
  — when M8 generates the reference bag, the `checkpoint_capture` assertion depends on the ADR-0010
  fix having landed first.
- **`ros2 bag info` footgun recorded:** point it at the bag **directory**, not a glob — `patrol_*`
  also matches the `<bag>.meta.json` sidecar and errors with "unrecognized arguments." Noted in
  `analysis/foxglove/README.md`.
