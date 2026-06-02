# ADR-0001: Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2

**Status:** Accepted
**Date:** 2026-06-02
**Deciders:** Project team

## Context

We need to pick a host OS, ROS 2 distribution, and JetPack version for the project. This decision cascades into every part of the stack: container base images, package compatibility, reference-implementation reusability, and the timing of any future migration.

The candidate stacks at decision time:

**Option A — Ubuntu 22.04 + ROS 2 Humble + JetPack 6.2 (legacy).** This is the historically dominant stack for PX4 + ROS 2 development. PX4's official documentation as of mid-2026 still recommends it. The most prominent reference implementation we plan to draw from (Andrew Bernas' "GPS-Denied Drone with NVIDIA Jetson Orin Nano") uses this stack. Humble reaches end-of-life May 2027.

**Option B — Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2 (modern).** ROS 2 Jazzy is supported through May 2029 (LTS). NVIDIA's Isaac ROS 4.0 release moved its recommended platform to Jazzy on 24.04. JetPack 7.2 (released June 2026) brought Jetson Orin NX/AGX onto Ubuntu 24.04 with a real-time-capable Linux 6.8 kernel, putting them on the same software generation as Jetson Thor. PX4 has not yet officially endorsed this combination but community demonstrations show it working end-to-end.

**Option C — Ubuntu 24.04 + ROS 2 Lyrical Luth.** Released May 2026, LTS through May 2031. Too new — ecosystem (Isaac ROS, third-party packages, documentation) has not caught up. Not seriously considered.

The project has an 8-phase plan running well into 2027 and beyond. The earliest phases are simulation-only; later phases involve Jetson deployment, learned perception models, and ongoing world-model research.

## Decision

Adopt **Option B: Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2** as the project-wide stack, starting from Phase 1 (pre-hardware simulation).

We will rebuild on this stack rather than fork the Bernas reference implementation directly. Their architectural patterns (Isaac ROS VSLAM into PX4 EKF2 via uXRCE-DDS, AprilTag relocalization, IMU calibration discipline) transfer; their specific package versions and container images do not.

## Consequences

### Positive

- **Software runway aligned with project timeline.** Humble EOLs May 2027; Jazzy goes to May 2029. The project will not need a forced mid-flight distro migration.
- **Aligned with NVIDIA's active recommendation.** Isaac ROS is the gating dependency for Phases 3–5 (VIO acceleration, perception, NITROS). Following NVIDIA's recommended platform reduces compatibility friction in the most platform-specific work.
- **Modern kernel and real-time support.** JetPack 7.2's Linux 6.8 with PREEMPT_RT support is meaningfully better for flight control workloads than JetPack 6.x.
- **Unified Orin toolchain.** JetPack 7.2 brings Orin NX/AGX into the same software generation as Jetson Thor. If we ever scale up the compute platform, the migration is incremental.
- **Migration cost paid at the cheapest moment.** Phase 1 is simulation-only: no hardware to reflash, no parameter tuning to preserve, no trained models, no field data. Every later phase makes the migration more expensive.

### Negative

- **PX4 doesn't officially support this combination yet.** PX4's main-branch documentation as of mid-2026 still recommends Humble + 22.04. We are early-adopters, not pioneers — community demonstrations exist — but should expect ~1 week of integration friction in Phase 1 Milestones M1–M2 that wouldn't exist on Humble.
- **Bernas reference can't be forked directly.** We adopt the architectural pattern and parameter values but rebuild the container stack on Jazzy/24.04/Isaac ROS 4.x. More work than a clean fork, less than greenfield.
- **Smaller documentation pool.** Most tutorials and Stack Overflow answers for PX4 + ROS 2 still target Humble. Searches will require translation.
- **Some third-party ROS 2 packages haven't released for Jazzy yet.** Where this happens, we either build from source, vendor patches, or wait. Expect a handful of these.

### Neutral

- **Python 3.12 (Ubuntu 24.04 default) instead of 3.10.** Most code is forward-compatible; deprecation warnings on some libraries.
- **Gazebo Harmonic is native to 24.04.** No change in simulator choice; this was already settled.
- **uXRCE-DDS bridge works identically.** Transport-layer choice unaffected by host distro.

## Alternatives considered

### Option A — Stay on Humble + 22.04 through the project

Rejected because the project timeline (Phase 5–8 running into 2027 and beyond) outlives Humble's support window (May 2027). Migrating mid-project, after we have working VIO, trained models, and accumulated parameter tuning, is strictly more expensive than migrating now.

### Hedge — Stay on Humble through Phase 4, migrate before Phase 5

Defensible. Phase 4 (indoor VIO patrol) is the highest-risk phase and the Bernas reference matches Humble exactly, so building on Humble there reduces variables. After Phase 4, migrate.

Rejected because:
- Phase 5 (forest navigation) and Phase 6 (anomaly detection) are themselves hard problems. Adding a distro migration on top means three new sources of complexity simultaneously.
- The Phase-1-to-Phase-4 friction we'd avoid on Humble (~1 week) is smaller than the Phase-4-to-Phase-5 migration cost (multiple weeks of revalidation across VIO, EKF2 tuning, and any models trained).
- Cleaner mental model: one stack for the project, decided once, ADR-captured.

### Option C — Lyrical Luth on 24.04

Rejected as premature. The release was 11 days ago at the time of this decision; Isaac ROS doesn't target it, and the broader ROS 2 ecosystem hasn't caught up. Revisit in 2027 if we see Isaac ROS or other critical dependencies move.

## References

- ROS 2 release schedule and support timeline: https://docs.ros.org/en/jazzy/Releases.html
- REP 2000 (ROS 2 target platforms): https://www.ros.org/reps/rep-2000.html
- Isaac ROS getting started (current platform recommendation): https://nvidia-isaac-ros.github.io/getting_started/index.html
- JetPack 7.2 release notes: https://developer.nvidia.com/embedded/jetpack
- Master plan, "Distro and OS decision" section: `../autonomous_drone_patrol_project_plan_v2.md`
- Phase 1 plan, "Target stack (pinned)" section: `../phase1_simulation_plan.md`
