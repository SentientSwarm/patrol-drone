# ADR-0010: M6 capture pipeline verified in SITL; full multi-checkpoint capture deferred on a dwell-stability finding

**Status:** Accepted; the deferred multi-checkpoint capture is **resolved by [ADR-0012](0012-m4-dwell-pose-camera-framing-fix.md)** — the gap was dwell-pose camera framing (tag jammed at the top frame edge), not the hypothesised WAYPOINT↔DWELL flap (that was already gone). A full patrol now captures all checkpoints (`/patrol/checkpoint_capture` Count 9, was 0).
**Date:** 2026-06-25
**Deciders:** Egemen Cankaya (project owner)

## Context

During the owner's manual M6 SITL checks (the AC-2/AC-4 nightly/manual tier the M6 DoD calls out),
a live patrol on `run_patrol_world_sitl.sh` produced a **real, well-formed checkpoint capture** for
the first checkpoint and confirmed the end-to-end M6 chain works — but the same run **failed to
capture the other three** checkpoints, and surfaced a mission-control dwell-stability problem that is
**not** an M6/perception defect. This ADR records what was verified, the fixes applied to get there,
and why the remaining gap is deferred rather than chased through more live iteration now.

### What was verified (M6 pipeline is proven end-to-end)

A capture for `cp_north` landed on disk as `captures/<run_id>/000_cp_north.png` (20 KB, a real
frame) plus its JSON sidecar, with:

- `checkpoint_id: "cp_north"` resolved from the AprilTag detection — **AC-4** (not fabricated).
- `pose` at `(11.92, 10.81, 1.62)` with a ~−90° yaw quaternion — the computed stand-off hover facing
  the tag (ADR-0008 geometry), in `frame_id: patrol_world` — **AC-2**.
- `metadata`: `tag_id 0`, `detection_confidence 93.45`, `tag_hamming 0`, `mission_id`,
  `waypoint_index 0`, `image_write_status: ok` — **AC-1 / PCAP-6**.

That single capture exercises the entire path — apriltag detection → capture trigger → checkpoint
resolution → image+sidecar write → `/patrol/checkpoint_capture` publish — so the M6 **deliverable**
(the capture node + the `CheckpointCapture` contract) is demonstrated in live SITL, not just unit
tests.

### Fixes required to reach that capture (each a real defect found live)

1. **`use_sim_time` was unset on the camera-pipeline nodes (root blocker).** The `apriltag` node
   synchronized **0** image/`camera_info` pairs (`Synchronized pairs: 0`, 15 in / 15 in) because the
   bridged image (sim clock) and `camera_info` carried mismatched stamps, so `apriltag_ros`'s
   exact-time `message_filters` sync dropped every frame → no detections → no captures. Fixed by
   setting `use_sim_time: true` on the `camera_info_bridge`, `apriltag`, and `patrol_perception`
   nodes in `patrol_perception.launch.py` so all stamps come from the Gazebo clock.
2. **The `/patrol/dwell` capture trigger fired on the DWELL rising edge, racing detection.** The
   trigger fired at dwell *entry* (t=0), before the drone settled onto the stand-off pose and before
   `apriltag` had buffered a detection there; live logs showed detection lagging dwell entry by ~2 s,
   so the capture gate always saw an empty buffer and skipped. Fixed by firing the trigger
   `_DWELL_SETTLE_S` (2 s) **after** entering DWELL (`node.py`), with `dwell_s` lengthened 3 s → 6 s
   in `patrol_mission.yaml` so the settle delay still fits inside the hold. This is **why** `cp_north`
   captured (its trigger now showed `1 detection buffered at fire`). Unit tests for the dwell event
   updated from rising-edge to settle-delay semantics; full unit suite green (487 passed).
3. **Runner camera gate was too strict for an M6 run (non-blocking).** `run_patrol_world_sitl.sh`'s
   `verify_camera` hard-failed (aborting the patrol before it flew) on (a) the `/compressed`
   companion topic, which `image_transport` advertises **lazily** so it was absent at check time, and
   (b) a camera rate above the band when stacked sims inflated it. Both are M5/M7 concerns, not M6.
   Made non-fatal (warn + poll for the lazy `/compressed`), gated behind `PATROL_STRICT_CAMERA=1` to
   restore the hard check for M5/M7 verification.

### The residual finding (why full multi-checkpoint capture is deferred)

Only `cp_north` is ever framed. The detection timeline showed a tag visible for ~3 s during
`cp_north`'s approach and **never** near the other three dwells. The patrol also flapped
`WAYPOINT ↔ DWELL` heavily (127 then 247 episodes across runs; `all_waypoints_dwelled` FAILed with
"dwelled indices `[]`"), and **loosening `completion.tolerance_m` 0.5 → 1.0 made the flapping worse**
(127 → 247) — counterintuitive, so that change was reverted to the tested 0.5 m / 2.0 s default.

The mechanism is **flight/dwell stability, owned by 02 (mission-control) / M4**, not M6:

- When the drone *does* settle at a stand-off pose with the tag framed (as at `cp_north`), the
  capture pipeline works flawlessly — perception is not at fault.
- The drone does not reliably settle at the `cp_east` / `cp_south` / inline stand-off poses, so their
  tags are never in view during the (now settle-delayed) trigger window.
- The "wider tolerance → more flapping" result suggests a possible **logic** interaction in the
  `WAYPOINT ↔ DWELL` transition (`state_machine.py`, `_within_tolerance_for_hold` /
  `_advance_from_dwell`), not merely PID dynamics — worth a code-level look before more flights.

## Decision

**M6 is treated as functionally verified by the `cp_north` capture, and the full
"all-checkpoints-captured-in-one-autonomous-patrol" result is deferred to a focused 02/M4
dwell-stability fix — not chased through further blind live iteration now.**

The four code/config fixes above are kept (they are correct and independently justified):
`use_sim_time`, the dwell settle-delay trigger + `dwell_s` 6 s, and the runner gate softening. The
tolerance change is reverted. The temporary rclpy DIAG instrumentation added to `perception_node.py`
during diagnosis was removed.

## Consequences

- **M6 exit checks status:** AC-3/AC-5/AC-7 pass (message exists, unit suite green, both packages
  build). AC-1/AC-2/AC-4 are **demonstrated for one checkpoint** (`cp_north`) end-to-end in SITL.
  AC-6 (exactly one capture per visit) is not yet observed across a full multi-checkpoint patrol
  because the other visits never produce a capture to count.
- **The remaining work is an 02/M4 task,** not M6: make the drone hold each stand-off + yaw-to-tag
  dwell pose stably (investigate the `WAYPOINT ↔ DWELL` flap as possibly a logic bug first, given
  wider tolerance worsened it; then tune the approach/yaw if needed) so every checkpoint's tag is
  framed during the settle window. Once stable, the existing pipeline should capture all of them with
  no perception change. Relates to ADR-0008 (the stand-off + yaw approach pose this depends on).
- **The settle-delay trigger is now load-bearing for capture timing** and couples to `dwell_s` and
  `completion.hold_time_s`; the constant carries a comment to revisit all three together if SITL
  timing changes.
- If the 02 dwell-stability fix slips, M6's pipeline remains proven (the `cp_north` artifact and the
  green unit suite stand), and this ADR carries the gap forward rather than leaving it silent.
