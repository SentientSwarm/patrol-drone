# ADR-0005: Consumers derive from `stack-manifest.toml`; downloaded artifacts are version-pinned + checksum-verified

**Status:** Accepted
**Date:** 2026-06-05
**Deciders:** Egemen Cankaya (project owner)

## Context

[ADR-0004](0004-stack-manifest-location.md) made `stack-manifest.toml` the canonical pinned-stack
manifest and stated that the Dockerfiles, compose, and README would "pull values via `.env` / build
`ARG`s so no version literal is duplicated anywhere else." The M1 bootstrap PR (#5) shipped the
manifest but did **not** yet wire that derivation: `scripts/setup_phase1.sh` and
`docker/sim/Dockerfile` carried their own copies of the PX4/ROS/base-image versions, and the
downloaded artifacts (QGroundControl, Foxglove, uv) tracked mutable `latest` URLs with no integrity
check. The Hermes review flagged three of these as Medium findings:

- **#1** `px4_msgs_ref` (`release/1.16`) sat on a different release line than `px4_version` (`v1.17.0`).
- **#2** "pinned" setup re-runs could install different artifacts (mutable `latest`, no checksums/digest).
- **#3** Executable consumers duplicated manifest literals, so an OQ-3 pin change would not propagate.

These are reproducibility / single-source-of-truth gaps to close *before* M2 operationally consumes
the manifest.

## Decision

1. **Single source of truth, actually derived.** `setup_phase1.sh` reads every version literal from
   `stack-manifest.toml` via a `manifest_get` helper (stdlib `tomllib`). `docker/sim/Dockerfile`
   carries **no** `ARG` defaults; `scripts/gen_build_args.py` injects them from the manifest
   (`docker build $(scripts/gen_build_args.py) ...`, and an `--env` form for compose).

2. **Pin + verify downloaded artifacts.** The manifest pins QGroundControl and Foxglove to explicit
   release versions with `sha256` checksums (not `latest`); `setup_phase1.sh::verify_sha256` aborts
   on mismatch. uv installs via the version-pinned `astral.sh/uv/<version>/install.sh` installer. The
   ROS base image is pinned by digest (`ros_base_image@ros_base_digest`). PX4 is checked out at the
   pinned tag and its resolved `HEAD` is verified against `px4_commit` (catches an upstream-moved tag).

3. **Align the provisional pair.** `px4_msgs_ref` is set to `release/1.17` to match `v1.17.0`'s
   release line. This stays *provisional* — the final OQ-3 pair is the M1–M2 spike's output; the
   manifest remains a DRAFT until M2/SWM-11.

4. **Enforce in CI.** `scripts/check_manifest_drift.py` (a `python-quality` job) fails when any
   consumer or summary (setup script, Dockerfile, README, CLAUDE.md) drifts from the manifest.

## Consequences

### Positive
- An OQ-3 pin change is a single manifest edit that flows into host setup *and* container builds with
  no duplicated literal to update; the drift gate makes silent divergence a CI failure.
- Re-running setup installs byte-identical artifacts (checksums + digest), shrinking the supply-chain
  trust surface the review called out.
- The repo gains its first unit suite (`tests/unit/test_manifest_drift.py`).

### Negative
- Bumping a desktop-app version now means editing the manifest **and** refreshing its `sha256` (and
  the base-image digest), gathered with `sha256sum` / `docker buildx imagetools inspect`. This is the
  intended cost of reproducibility.
- `setup_phase1.sh` now needs `python3` (3.11+) available before it derives config — true on the
  Ubuntu 24.04 target by default.

### Neutral
- The manifest stays a DRAFT until M2; this ADR governs *how* values are consumed and verified, not
  the final OQ-3 pin.
- `docker/dev` + compose remain M2 deliverables; `gen_build_args.py` is written so they plug in later.

## Alternatives considered

- **CI drift-check over duplicated literals (keep the copies, guard them).** Simpler, but leaves the
  duplication ADR-0004 explicitly forbids ("no version literal is duplicated anywhere else").
  Rejected in favour of real derivation; the drift gate is retained as *additional* enforcement for
  the human-facing summaries that can't derive (README, CLAUDE.md).
- **Defer checksum/digest pinning to M2.** Rejected: the review's reproducibility finding is
  actionable now, and pinning what is already installed on the host is low-risk.
