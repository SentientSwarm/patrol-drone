# ADR-0012: the checkpoint-capture gap was dwell-pose camera framing, not a WAYPOINT↔DWELL flap — fixed by raising the stand-off hover so the down-pitched camera centers the tag

**Status:** Accepted
**Date:** 2026-06-26
**Deciders:** Egemen Cankaya (project owner)

Supersedes the deferral recorded in [ADR-0010](0010-m6-capture-verified-dwell-stability-deferred.md)
and [ADR-0011](0011-m7-recording-verified-checkpoint-capture-count-deferred.md). Refines
[ADR-0008](0008-checkpoint-approach-pose.md) (the stand-off + yaw approach pose).

## Context

ADR-0010/0011 deferred the "all checkpoints captured in one autonomous patrol" result on a
hypothesised **mission-control dwell-stability problem**: a `WAYPOINT ↔ DWELL` flap (127 then 247
DWELL "episodes" with `all_waypoints_dwelled` FAILing on "dwelled indices `[]`"), with the
counterintuitive clue that **loosening `completion.tolerance_m` 0.5 → 1.0 made the flap worse** —
read as a possible *logic* interaction in the `WAYPOINT ↔ DWELL` transition to investigate before
tuning dynamics. The full-patrol MCAP bag recorded `/patrol/checkpoint_capture` **Count: 0**.

This ADR records the focused 02/M4 investigation that deferral called for, what the bug actually was,
the fix, and the before/after capture count.

### Reproduce-first: the flap was already gone; the real bug is framing

The ADR-0011 bag (`patrol_patrol_20260626_080740`, still on disk) was re-analysed directly rather
than re-flown blind. Its observable surface contradicts the dwell-stability hypothesis:

- The `/patrol/mission_state` stream is a **textbook-clean** patrol —
  `TAKEOFF→HOVER→[WAYPOINT→DWELL]×4→RTH→LANDING→DONE` — with **exactly 4 DWELL episodes**, each
  lasting the full ~6.1 s, and **all 4 `/patrol/dwell` capture triggers fired** (idx 0,1,2,3).
- The vehicle **settles dead-on each stand-off target** (cp_north NED (10.86, 11.80, −1.51) vs
  target (11, 12, −1.5)) and is **level at dwell** (pitch mean +0.2°, max +0.9°).

So the `WAYPOINT ↔ DWELL` flap **no longer exists** — it was resolved by the dwell-settle-delay +
`dwell_s` 6 s fixes ADR-0010 already landed. The 127/247-episode figures were from *earlier* runs
before those fixes. (The "wider tolerance → more flapping" clue was real for those pre-fix runs but
is now moot; `completion.tolerance_m` stays at the tested 0.5 m / 2.0 s — never loosened, per
ADR-0010.) There is **no `state_machine.py` logic change** in this fix.

The real cause is **camera framing at the dwell pose** (a geometry/dynamics problem, but a *static*
geometry one, not a PID-settling one):

- Of 2117 `/tag_detections` arrays in the bag, only **42 are non-empty**, and they all occur at
  **+18.6–21.4 s** — during the *approach* to cp_north (tag id 0). **Inside all 4 DWELL windows:
  zero non-empty detections.** 04's settle-delayed capture gate therefore always saw an empty buffer
  → `checkpoint_capture` Count 0.
- **Why:** at the stand-off pose the drone hovered at the **same altitude as the tag** (1.5 m) while
  the airframe camera is rigidly pitched **~20° (0.35 rad) down** (`gz_x500_patrol` `camera_link`
  pose). The tag center then sits **~20° above the camera boresight — 74 % toward the top frame edge
  (~63 px from the top), top corners only ~2.1° from the edge**. Geometrically "in frame" (the
  existing `test_checkpoint_visibility.py` corners-in-FOV oracle passed by that 2° hair), but
  foreshortened at the extreme periphery, so apriltag could not resolve the quad at dwell. During
  *approach* the drone is further/higher, putting the tag lower and more central — the only window it
  was ever detected.

## Decision

**Center the tag in the camera at the dwell pose by raising the checkpoint hover altitude.** The
camera's down-pitch is fixed in hardware, so the only remaining degree of freedom that brings the tag
onto the boresight vertically is altitude. `config._approach_pose` now climbs
`standoff_m * tan(camera_pitch_rad)` **above** the tag (≈ +1.10 m → a 2.60 m hover for the canonical
1.5 m tags at the 3.0 m stand-off), with `camera_pitch_rad` a new `approach` field defaulting to
**0.35 rad** to match the airframe SDF. The +Y stand-off distance and the yaw-to-face-the-tag are
unchanged (ADR-0008). A loader guard rejects `camera_pitch_rad` outside `[0, π/2)`.

This makes the down-pitched boresight land on the tag center: **−0.8° from boresight** (was +20.1°),
margin to the top edge **26.4°** (was 2.1°).

### Regression lock

`test_checkpoint_visibility.py` gains a `test_tag_centered_at_resolved_hover_pose` parametrised over
every canonical checkpoint, asserting the tag center projects within **half the vertical half-FOV**
of boresight. It composes the real `_approach_pose`, the airframe SDF intrinsics/mount, and the
canonical checkpoints, so there is no second source of truth. It was **RED** for the old
same-altitude pose (center_el 0.35 rad vs 0.237 threshold) and **GREEN** after the climb. The
existing corners-in-FOV test is retained. `test_config.py`'s resolved-pose assertions were updated to
the climbed `z` (inline waypoints, which carry no tag, are unchanged).

## Consequences

- **Before/after (the falsifiable signal):** a full patrol via `mission_patrol.launch.py`
  (record on) now produces an MCAP bag with **`/patrol/checkpoint_capture` Count = 9** (was **0** in
  ADR-0011's bag) — and three real on-disk captures `000_cp_north.png` / `001_cp_east.png` /
  `002_cp_south.png` with sidecars (tag-resolved `checkpoint_id`, 93 % confidence, pose `z` 2.62 m
  confirming the climb). Before the fix only `cp_north` ever captured; now all three tag-bearing
  checkpoints do. The 4th waypoint is the inline overlook (no tag) and correctly produces no capture.
- **The patrol is clean — no dwell flap.** The verified post-fix bag, with stale-publisher `DONE`
  contamination from overlapping local runs filtered out, shows exactly the 4-DWELL clean sequence
  above. (Caveat for the verifier: running two patrols back-to-back without a full teardown leaves a
  prior node publishing `DONE` on `/patrol/mission_state`, which inflates the DwellTracker's
  rising-edge episode count — a test-environment artifact, not a code defect. Tear the stack down
  fully between runs.)
- **AC-7 (M7) and AC-6 (M6) acceptance close:** the ADR-0010/0011 deferral is resolved with **no M6
  or M7 code change** — the unchanged recorder records, and the unchanged capture pipeline publishes,
  the captures that the now-correct dwell pose makes possible. This was scoped entirely to 02/M4
  (one `config.py` change), honoring one-milestone-at-a-time.
- **`scripts/run_patrol_world_sitl.sh`:** `WS_SETUP` is now env-overridable (`${WS_SETUP:-…}`) so the
  recorder-bearing run can point at a merged-install prefix. The isolated colcon install trips the
  F-04 `_same_install_prefix` recorder-include guard (every package gets its own prefix), so the
  record-side bag verification above used a merged install of the patrol packages chained over the
  existing `px4_msgs` underlay. This is a verification-host convenience; the guard itself is correct.
- **Coupling:** the climb couples the dwell pose to the airframe camera's `camera_pitch_rad`. If the
  camera mount pitch or the stand-off changes, the centering test fails loud and `camera_pitch_rad`
  (in `patrol_mission.yaml`'s `approach`, or the default) must track the SDF. Relates to ADR-0008.
