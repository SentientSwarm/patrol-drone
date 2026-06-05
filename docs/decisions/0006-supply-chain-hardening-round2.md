# ADR-0006: Round-2 supply-chain hardening of the host bootstrap

**Status:** Accepted
**Date:** 2026-06-05
**Deciders:** Egemen Cankaya (project owner)

## Context

[ADR-0005](0005-manifest-derivation-and-pinning.md) made `stack-manifest.toml` the derived single
source of truth and pinned+verified the QGroundControl/Foxglove downloads, the ROS base-image
digest, and the PX4 checkout. A second Hermes review of PR #5 (at HEAD `dc4b1b0`) returned
`CHANGES_REQUESTED` because two installer paths in `scripts/setup_phase1.sh` still fetched/executed
mutable or unverified content, two apt trust roots were accepted without a fingerprint check, the
drift gate had a blind spot, and the commented M2 Dockerfile scaffold still baked literals:

- **Medium #1** — the ROS apt-source `.deb` was selected via the GitHub `releases/latest` API and
  root-installed with no version pin or checksum.
- **Medium #2** — `uv` was installed via `curl … | sh` (remote script piped to a shell).
- **Low #1 / #2** — the Docker and NVIDIA Container Toolkit apt keys were trusted without verifying
  their fingerprints (scoped `signed-by` trust, but the key itself unchecked).
- **Low #3** — `check_manifest_drift.py` only guarded PX4/ROS literals; inlining a literal for
  UV/QGC/Foxglove values left the gate green.
- **Low #4** — the commented M2 runtime scaffold in `docker/sim/Dockerfile` baked `harmonic`/`jazzy`
  literals that a future copy-paste could carry past the single-source-of-truth pattern.

## Decision

Extend ADR-0005's "pin + verify, derive from the manifest" rule to the remaining paths:

1. **ROS apt-source `.deb`** — pinned in the manifest (`[ros_apt_source] version`, `sha256` of the
   noble `_all` asset) and verified with `verify_sha256` before the root apt install. The
   `releases/latest` API call is removed.

2. **`uv`** — installed from the version-pinned GitHub **release tarball**
   (`uv-x86_64-unknown-linux-gnu.tar.gz`), `sha256`-verified (`[tools] uv_tarball_sha256`) before
   extraction. No remote script is piped to a shell. x86_64-linux only (the pinned artifact); other
   arches warn and skip rather than install the wrong binary.

3. **Apt signing keys** — the Docker and NVIDIA keys are verified against pinned primary-key
   fingerprints (`verify_gpg_fingerprint`) before their repos are added. The fingerprints live as
   in-script `readonly` constants with source-URL comments: they are fixed upstream trust roots, not
   versioned pins, so keeping them out of the manifest keeps it version-focused and the drift gate's
   surface small.

4. **Drift gate** — `check_manifest_drift.py` gains a `missing_derivations()` check driven by a
   `DERIVED_VARS` map, asserting every pinned value is assigned via `manifest_get` (an inlined
   literal removes the line and is reported). Unit tests cover the clean and inlined-literal cases.

5. **Dockerfile scaffold** — the commented M2 runtime block is de-literalized to `ARG ROS_DISTRO` /
   `ARG GZ_VERSION` (no defaults) with a TODO to extend `gen_build_args.py` at M2.

## Consequences

### Positive
- Re-running setup installs byte-identical artifacts for the ROS apt-source and `uv` too; no remote
  shell-exec remains in the bootstrap. Substituted apt keys are refused.
- The drift gate now fails closed on any inlined literal across the full pinned set, not just PX4/ROS.

### Negative
- Bumping the ROS apt-source or `uv` version now also means refreshing its `sha256` in the manifest
  (gathered with `sha256sum`); the intended cost of reproducibility.
- The ROS apt-source `sha256` is codename-specific (noble); on a non-noble host the check fails
  closed. Acceptable — the script targets Ubuntu 24.04 and preflight already warns otherwise.

### Neutral
- Fingerprints and checksums are upstream facts captured at pin time; a wrong value fails the
  install or CI immediately rather than passing silently.
- `docker/dev` + compose and the M2 runtime stage remain M2 deliverables.

## Alternatives considered
- **Pin the `uv` `install.sh` checksum and keep `sh`-ing it.** Smaller diff, but still executes a
  remote script; rejected in favour of the reviewer-preferred release-artifact path.
- **Move fingerprints into `stack-manifest.toml`.** More uniform, but fingerprints are not versions
  and would widen the drift gate's surface; kept as in-script constants instead.
