# ADR-0009: M6 perception residual review findings are hardened in M7, not M6

**Status:** Accepted
**Date:** 2026-06-25
**Deciders:** Egemen Cankaya (project owner)

## Context

PR #12 (M6 — Perception & Checkpoint Capture, docset 04) is **approved and green** at head
`1574703` (Hermes APPROVE / Risk Low; CodeScene all gates pass; all CI checks pass). During the
review chain the one *blocking* defect — the stale-buffer false-capture, where a stalled
detector/frame/pose stream let a tuple from a previous visit publish as a capture for a *new*
visit — was fixed (commits `5c53e3d` → `a13050f` → `1574703`): each buffer now carries a
node-receipt timestamp and the `CaptureCoordinator` enforces a per-stream freshness window
(`FreshnessWindows`), skipping a stale-but-present buffer exactly like an absent one and leaving
the visit retryable (design §4.4.5/§4.4.6).

Two **non-blocking residual findings** remain, both explicitly tagged by the reviewer as
"harden in a follow-up":

1. **Medium — freshness gate ages from node-receipt time, not source `header.stamp`.**
   `LatestBuffer.update()` stores `self._clock()` as `received_at`
   ([samplers.py:42-44](../../ros2_ws/src/patrol_perception/patrol_perception/samplers.py)); the
   coordinator's `_is_fresh()` ages only that receipt time
   ([coordinator.py:111-116](../../ros2_ws/src/patrol_perception/patrol_perception/coordinator.py)).
   A message that *arrives* recently but carries an *old* `header.stamp` (callback backlog,
   transport delay, **bag replay**) passes the gate as fresh. A reviewer probe with an image whose
   `header.stamp` was `(0,0)` but receipt time `(100,0)` produced `stale_header_published_count 1`.
   This bounds a *stalled callback* but not an *old-but-recently-delivered* sample, weakening the
   "tag currently in view / latest frame" guarantee. Already documented as a Phase-1 boundary in
   design §4.4.5 (the boundary paragraph) and §6 future work ("Time-synchronized frame/pose
   sampling").

2. **Low — capture artifacts can overwrite on same-second `run_id` collisions.** `run_id` is
   second-precision UTC (`%Y%m%dT%H%M%SZ`), each `CaptureWriter` starts `_index = 0`, and
   `Path.write_bytes/write_text` overwrites the deterministic `000_<checkpoint_id>.{png,json}`
   paths ([capture_writer.py:29-47](../../ros2_ws/src/patrol_perception/patrol_perception/capture_writer.py)).
   A fast restart or an accidentally duplicated perception node within the same wall-clock second
   clobbers the earlier instance's on-disk artifacts. The published ROS message still exists (the
   topic is the bag's source of truth, §4.4.5), so this is durability-only, never a wrong capture.

The question raised was whether to fix these on PR #12 now or defer. Fixing finding 1 in
particular carries a real regression risk (see below), so rushing it onto an already-approved PR
trades a *bounded, documented* Phase-1 limitation for a chance of a *new* medium/high defect in
the safety-relevant capture path.

## Decision

**Both residual findings are deferred to M7 (rosbag2/MCAP logging + replay) and tracked there,
not fixed on PR #12.** PR #12 merges as-is (after the owner's manual M6 SITL checks); the debt is
recorded here and surfaced in the M6 design doc so M7 picks it up deliberately.

M7 is the correct home, not "Phase 3+" as the design's future-work row originally implied:

- **M7 is the milestone where finding 1 first bites.** M7 records and *replays* bags; a replayed
  bag delivers source-old `header.stamp` frames by construction — exactly the case the
  receipt-time gate misses. The fix belongs with the feature that makes it load-bearing, and M7
  ships the replay harness to validate it against. (Phase 3+ time-synchronized sampling remains the
  *full* solution — `message_filters` time-sync across streams; M7 adds the narrower source-age
  gate, not the full synchronizer.)
- **Finding 2's collision surface widens at M7.** Run/bag directory alignment (OQ-4) means
  perception captures and bags share a run dir; M7 is the natural point to make `run_id`
  collision-resistant once and apply it to both.

### Implementation notes for M7 (the non-obvious traps)

- **Finding 1 — clock-domain hazard (this is why it is not a one-liner).** The three streams do
  **not** share a clock:
  - Camera `sensor_msgs/Image` and the `apriltag_ros` detection array carry ROS `header.stamp`
    (`builtin_interfaces/Time`, ROS time).
  - PX4 `VehicleLocalPosition` (the pose stream) carries a `timestamp` in **PX4 microseconds from
    a different epoch** delivered over the uXRCE-DDS bridge — **not** a ROS `header.stamp` and not
    directly comparable to `self._clock()`'s `(sec, nanosec)` ROS time.

  Threading "source age" naively across these by subtracting mismatched clocks is exactly how a
  *new* false-skip (drops every valid pose) or false-pass (accepts a stale pose) gets introduced —
  the medium/high regression risk that justifies deferral. M7 must make a deliberate per-stream
  decision: gate the ROS-stamped streams (frame, detections) on `header.stamp` source age, and for
  the PX4 pose either translate the PX4-µs timestamp into ROS time at the single MC-7 boundary or
  keep it on receipt-time with that asymmetry documented. Add unit probes for "fresh receipt +
  stale source/header" per stream.
- **Finding 2.** Make `run_id` collision-resistant (sub-second precision or a short UUID suffix)
  and/or write with exclusive creation / next-unused index when the run dir already exists, so a
  same-second restart cannot clobber. Apply once at the run-id seam so captures and bags share it.

## Consequences

- PR #12 (M6) merges with the blocking defect fixed and two **documented** limitations: a
  receipt-time-only freshness gate and a same-second artifact-overwrite edge. Neither affects M6's
  exit criteria — the ground-truth quasi-static hover at each checkpoint does not produce
  source-old samples in live SITL, and a normal patrol does not restart perception within one
  second.
- The design's §6 future-work row attributing the source-vs-receipt skew to "Phase 3+" is
  **reconciled to M7** for the narrow source-age gate (Phase 3+ retains the full
  time-synchronized-sampling solution). See the design changelog entry referencing this ADR.
- M7 inherits a concrete, pre-analyzed task list (the two findings above, with the clock-domain
  trap called out), rather than rediscovering them from PR comments after PR #12 closes.
- If M7 slips or descopes, both limitations remain merely *bounded and documented*, not silent —
  the design doc and this ADR carry them forward.
