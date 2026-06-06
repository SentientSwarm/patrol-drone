# ADR-0003: `setup_phase1.sh` provisions the full Phase 1 toolchain, not just M1

**Status:** Accepted
**Date:** 2026-06-04
**Deciders:** Project owner (egemencankaya14@gmail.com)

## Context

`scripts/setup_phase1.sh` began life as an M1-only host bootstrap: base packages, QGC, uv,
and the PX4 source checkout. ROS 2 was a deliberate no-op placeholder and Docker was absent
entirely. The motivation for that narrow scope was the repo working agreement — "one milestone
at a time" — applied to the setup script.

In practice this means a collaborator on a clean Ubuntu 24.04 machine cannot run one script and
then develop Phase 1: every milestone past M1 first requires a manual, undocumented toolchain
install (ROS 2, the XRCE agent, Docker, the rosbag2 MCAP plugin, apriltag, Foxglove). The
owner's guidance was to "develop a shell script to install the prerequisites so it's
repeatable" — i.e. a fresh host should be fully tooled in one command.

The tension is only apparent. "One milestone at a time" is a discipline about **deliverables** —
the code, messages, worlds, containers, and tests each milestone produces and commits to git.
Installing a *toolchain* on the host does not advance any milestone's deliverables: having ROS 2
installed is not the same as having the uXRCE-DDS bridge proven, `px4_msgs` vendored, or topics
flowing at 50 Hz (that work, and its exit test, still belong to M2).

## Decision

`setup_phase1.sh` installs the **full Phase 1 prerequisite toolchain by default** (ROS 2 Jazzy
+ colcon/rosdep, the Micro XRCE-DDS Agent built from source at the pinned eProsima tag
([ADR-0007](0007-uxrce-dds-agent-from-source.md) — there is no Jazzy apt package), Docker + Compose, the ROS runtime packages
later milestones need, QGroundControl, and Foxglove Studio), with `--skip-*` flags to opt out of
any section. The NVIDIA Container Toolkit is installed only under `--with-nvidia`.

The script explicitly does **not** produce repo **deliverables**. The dividing line:

- **Prerequisites (in the script):** anything installed onto the host — system/apt packages,
  snaps, desktop apps, language toolchains, the PX4 source tree + its dev env.
- **Deliverables (NOT in the script; owned by milestones, committed to git):** vendored
  `px4_msgs`/`px4_ros_com` under `ros2_ws/src/external/`, the `docker/{sim,dev}` Dockerfiles and
  `docker compose` definitions, `colcon build` of the workspace, per-milestone Python deps added
  to `pyproject.toml` via `uv add`, and all mission/perception/logging code.

## Consequences

### Positive
- A fresh host goes from `git clone` to "able to build and run all of Phase 1" in one command,
  matching exit-checklist item 10 (setup-to-running-mission, ≤20 commands).
- Onboarding is reproducible and self-documenting; no tribal-knowledge toolchain steps.
- The prerequisites/deliverables line keeps "one milestone at a time" intact and makes it
  explicit what the `/devloop` milestones still own.

### Negative
- A larger one-shot install surface on an early-adopter stack (the ~1 week of PX4-on-Jazzy
  integration friction the plan warns about now front-loads into one run). Mitigated by keeping
  every section idempotent and individually skippable, so a failure in one part is re-runnable
  without redoing the rest.
- The script now needs maintenance as upstream install methods drift (ROS apt-source `.deb`,
  Docker repo, Foxglove URL).

### Neutral
- The pinned stack (ADR-0001) and two-layer CI (ADR-0002) remain the source of truth for *what*
  versions and *how* CI runs; this ADR only governs the bootstrap script's scope.
- Desktop apps (QGC, Foxglove) install by default but stay desktop apps — not containerized,
  per the plan's containerization section.

## Alternatives considered

- **Keep the script M1-only; document the rest as manual steps.** Rejected: it fails the
  "repeatable in one command" goal and pushes setup friction onto every collaborator.
- **Opt-in `--phase1-full` flag, M1 as the default.** Rejected: a fresh-install user would not
  know to pass it; "just works" wants full-by-default with opt-outs instead.
- **Have the script also vendor `px4_msgs` and scaffold the containers.** Rejected: those are
  deliverables that live in git and are produced/owned by milestones; baking them into setup
  would blur the prerequisites/deliverables line this ADR draws.
