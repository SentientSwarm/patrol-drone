# ADR-0002: Two-layer CI with xenon complexity and 85% coverage gates

**Status:** Accepted
**Date:** 2026-06-03
**Deciders:** Project team

## Context

The repo needs CI for five quality pillars — testing, linting, complexity, type
safety, and test coverage — for a ROS 2 Jazzy + Python 3.12 monorepo whose code
splits into ROS-independent Python (mission state machine, analysis, scripts) and
ROS-dependent nodes that import rclpy and generated message packages. The latter
cannot be imported, type-checked, or unit-tested without a full ROS 2
environment. SITL integration tests are slow and flaky on hosted runners. The
repo is currently a skeleton with no source code, so the CI must be green on the
empty tree and activate as code lands.

## Decision

Adopt a **two-layer CI**:

- **Layer A (`python-quality.yml`)** — fast pure-Python checks on a plain
  `ubuntu-24.04` runner via `uv`: ruff (lint + format), mypy (types, with ROS
  import roots exempted), a **xenon** per-function complexity hard gate
  (`--max-absolute B`, no average gates), pytest on the ROS-free unit suite, and
  shellcheck. The test job also enforces a **≥85% coverage floor** (pytest-cov)
  on the ROS-free mission core.
- **Layer B (`ros-ci.yml`)** — `colcon build` + `colcon test` in a ROS 2 Jazzy
  container via `ros-tooling/action-ros-ci`; ament's Python linters disabled in
  favor of ruff. Coverage here is informational; the hard gate is in Layer A.
- **SITL (`sitl-nightly.yml`)** — deferred scaffold on manual/nightly triggers;
  never a per-PR or required check.

Tool/action versions are pinned (actions to commit SHAs; Dependabot bumps them).
Each gate is guarded to no-op on the empty skeleton and self-activate as packages
and tests appear in M3.

## Consequences

### Positive
- Fast feedback on the bulk of the code without a ROS toolchain.
- The slow/flaky SITL tier is isolated from per-PR CI.
- Per-function complexity gating avoids coupling a PR to unrelated, previously
  accepted code (an average gate would flip green→red on additive effects).
- The 85% coverage floor on the mission core enforces the Phase-1 exit criterion
  (the plan's ">80%" target) on every PR, while leaving exploratory `analysis/`
  and `scripts/` code unconstrained.
- Green on the empty skeleton; jobs auto-activate as packages and tests land.

### Negative
- Two layers to maintain; a small class of cross-layer issues (e.g. a typo in a
  rclpy call path) is only caught in Layer B.
- mypy on ROS-touching code relies on import-root exemptions rather than real
  stubs, so type coverage of the ROS boundary is shallow until stubs exist.
- The coverage `source` path and the test import strategy must be reconciled when
  the first package lands (tracked as an M3 spike in the design spec).

### Neutral
- xenon thresholds live as CLI flags (no config file); coverage `fail_under`
  lives in both `pyproject.toml` and the workflow CLI for unambiguous enforcement.

## Alternatives considered

- **Single-layer (pure-Python) CI only.** Simpler, but leaves the colcon/ament
  build unguarded until someone adds it later. Rejected — the ROS layer is cheap
  to scaffold now (guarded to no-op until packages exist).
- **Everything, including SITL, in per-PR CI.** Maximum coverage, but the
  SITL/Gazebo tier is slow and flaky on hosted runners (the project plan's
  least-confident area). Rejected for per-PR; kept as a nightly scaffold.
- **ruff's built-in mccabe (C901) as the only complexity check.** Convenient but
  only an inline hint, not a hard gate. We keep C901 for fast inline feedback and
  add xenon as the enforcing gate on the same CC boundary.
- **Average / per-module complexity gates (`--max-average`, `--max-modules`).**
  Rejected — an average couples a PR to the complexity of unrelated,
  previously-accepted code, flipping the build red through additive effects.
- **Coverage floor across all first-party Python.** Rejected — a hard floor on
  exploratory `analysis/` notebooks and `scripts/` is friction, not safety. The
  floor targets the ROS-free mission core only.

## References
- Design spec: `docs/design/2026-06-03-ci-workflows-design.md`
- Implementation plan: `docs/design/2026-06-03-ci-workflows-plan.md`
- ADR-0001 (distro/OS): `docs/decisions/0001-distro-and-os.md`
