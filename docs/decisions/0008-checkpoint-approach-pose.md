# ADR-0008: Checkpoint waypoints resolve to a stand-off + yaw-to-tag approach pose

**Status:** Accepted
**Date:** 2026-06-22
**Deciders:** Egemen Cankaya (project owner)

## Context

M5 places AprilTag checkpoint markers in the world and attaches a forward, slightly-down RGB camera
so M6 perception can resolve a tag at each checkpoint. The first M5 implementation resolved a
`checkpoint_id` waypoint directly to the tag's center position (`checkpoints[cid]`) with no offset and
an unconstrained yaw, so the drone was commanded to **hover on top of the tag**.

Two consecutive PR #9 reviews flagged this as blocking (SIM-4), and independent geometry confirmed it:
the tag face normal is fixed along world ±Y (the World Composer emits every marker at zero yaw, boxed
thin in Y), and at ~0.12 m range a 0.5 m tag subtends ≈128.7° against the camera's ≈68.8° HFOV — the
camera sits edge-on/on top of the marker, not looking at a resolvable face. The asset-contract tests
(drift, parse strictness, camera-topic agreement) all passed green because none of them assert the
*capability* the geometry is supposed to deliver, so the gap shipped unnoticed.

`checkpoints.yaml` is the canonical, 03-owned tag/world-position source and its header explicitly
forbids waypoint fields (dwell/tolerance/approach) — those are 02's mission concern. So the marker
position and the drone's flight pose must stay separated.

## Decision

Keep `checkpoints.yaml` as the marker source and resolve each checkpoint waypoint to an explicit
**approach pose** at the 02 waypoint-resolution seam:

- A new optional, defaulted `approach: { standoff_m: 3.0 }` section in the mission YAML
  (`patrol_mission.config.Approach`). Non-positive `standoff_m` fails loud.
- `config._approach_pose(tag, standoff_m)` returns the ENU hover point `tag + (0, +standoff_m, 0)`
  (north of the tag, at the tag's altitude) and the ENU yaw that faces the tag center. The +Y
  direction is taken from the world invariant "tags are emitted at zero yaw, face normal along ±Y".
- The ENU yaw converts to PX4 NED yaw at the **single MC-7 boundary** (`frames.enu_yaw_to_ned`,
  `yaw_ned = π/2 − yaw_enu`, wrapped to (−π, π]) — no second heading-conversion site.
- The state machine carries a per-waypoint NED-yaw list parallel to the waypoints (default zeros =
  hold North, the prior behavior) and emits it on the WAYPOINT/DWELL setpoints. The node builds the
  list from the resolved waypoints.
- A pure-geometry oracle (`tests/unit/test_checkpoint_visibility.py`) projects each tag's four corners
  through the camera (intrinsics + mount parsed from the airframe SDF) at the resolved hover pose and
  asserts they fall inside the FOV — locking the geometry against regression.

The world SDF, the World Composer, the AprilTag models, and the `world-drift` gate are **untouched**:
tags are not moved, so no regeneration is needed.

## Consequences

- The "patrol traverses every checkpoint" criterion (AC-3) now genuinely implies a tag-in-frame hover,
  not just position reach; the DoD §7 camera-mount decision is recorded as resolved.
- 02's approach geometry is **coupled to the world invariant** that tags face ±Y (zero-yaw emission).
  If a future change rotates tags, the stand-off direction must follow; the oracle test reads the real
  mount/FOV from the SDF and would catch a corner falling out of frame, but the +Y assumption lives in
  `_approach_pose` and is documented at both ends.
- The top-of-tag VFOV margin is tight (~2° at `standoff_m = 3 m`, governed by the fixed ~20° mount
  pitch). `standoff_m` is the single tunable — increasing it widens the margin. The oracle test is the
  guard if the mount, FOV, or stand-off changes.
- Inline (non-checkpoint) waypoints carry no facing constraint and keep the prior NED-0 heading, so the
  change is scoped to checkpoint waypoints only.
