# ADR-0013: the M8 live-bag rate-band false-failure was a GUI-load recording artifact, not a rate bug — the e2e witness now measures the true rate from de-duplicated timestamps and hard-fails an inconsistent bag

**Status:** Accepted
**Date:** 2026-07-01
**Deciders:** Egemen Cankaya (project owner)

Scopes to 05-logging-replay / M8 (LR-5 / LR-8, `analysis/e2e_check.md` step 4). Does **not** touch the
CI `replay-regression` lane ([`tests/replay/test_replay_regression.py`](../../tests/replay/test_replay_regression.py)),
`assertions.yaml`, or the reference bag.

## Context

Verifying a freshly-recorded **live** patrol bag against `tests/replay/assertions.yaml` tripped the
rate band — but only on this host and only with the Gazebo GUI running. `ros2 bag info` reported
`/drone/camera/image_raw/compressed` at ~30 Hz and `/patrol/mission_state` at ~20 Hz (both exactly
2x their 15 Hz / 10 Hz bands), while `/fmu/out/vehicle_local_position_v1` read a correct ~50 Hz. The
checked-in CI test (reference bag) passed and the bands matched the reference bag exactly.

### Reproduce-first: the publisher is correct; the bag is inconsistent

The offending bag (`~/patrol_bags/patrol_20260701T151327Z_20260701_151328`) was analysed directly
rather than re-flown blind. Three independent probes of the *same* camera stream disagree:

| Probe | Camera | What it is |
|---|---|---|
| `ros2 bag info` count / Duration | 9037 / 298.3 s = **30.3 Hz** | the MCAP *summary* (what trips the check) |
| SequentialReader rows / log_time span | 9335 / 781.5 s = **11.95 Hz** | the actual *message stream* |
| **unique sim-timestamps / sim span** | **4537 / 299.4 s = 15.15 Hz** | the *true* publish rate |

The true rate is **15.15 Hz** — exactly the airframe SDF `<update_rate>15</update_rate>` and the
reference bag's 15.16 Hz. So the "30 Hz" is a **measurement/recording artifact**, not a rate bug, and
the mechanism is *not* "the sim clock ran 2x fast" (that would cancel when dividing by the message
timeline). Instead the bag is **internally inconsistent**: its summary (9037 msgs / 298 s) disagrees
with its message stream (9335 rows / 781 s), and each rendered frame's sim-timestamp appears ~2x with
distinct receive times (dup_factor **2.06**). `/fmu/out/*` — lockstep sim-time — is immune
(dup_factor 1.00, correct rate). Cross-check isolating the cause to the *bag*, not the reader: on the
**reference** bag SequentialReader and `ros2 bag info` agree exactly (303 camera / 200 mission_state).

**Why it happens:** under GUI load this host can't hold real-time; the sim sags (camera sim-span
299 s took ~781 s of wall log_time) and the GUI-loaded `ros_gz` bridge / recorder path double-delivers
rendered frames while the MCAP summary is written inconsistently (a not-cleanly-finalized bag is the
likely trigger). The root cause of the duplication / non-finalized summary in the record path is a
**deferred follow-up** (needs a fresh instrumented SITL run — see Consequences); this ADR fixes the
*witness* so the artifact can no longer masquerade as a rate regression.

### What this rules out
- **Not `assertions.yaml`** — bands correct (reference bag: 10.0 / 15.16 / 49.9 Hz, all match).
- **Not** the mission node timer (10 Hz) or camera SDF (15 Hz) — both correct; true rates confirmed.
- **Not a pure RTF-rescale.** RTF-scaling `30.3 × (298/547) = 16.5 Hz` lands in-band only by
  coincidence (the GUI both duplicated frames *and* ran slow), so `count / bag-info-duration` is
  simply the wrong thing to trust for a live bag.

## Decision

**The manual/e2e live-bag witness measures the TRUE publish rate from de-duplicated message
timestamps, and hard-fails a demonstrably inconsistent bag rather than reporting a false rate.** A new
`tests/replay/verify_live_bag.py` (run explicitly under system python + sourced ROS — the CLAUDE.md
numpy/uv boundary) drives a new ROS-free analyzer `tests/replay/rate_report.py`:

1. **Consistency guard first (hard fail, exit 2).** Cross-check `ros2 bag info` counts (via the
   existing `ingest.bag_reader.parse_bag_info`) against the SequentialReader row count, and the
   per-topic sim-stamp `dup_factor`. A bag is untrustworthy if any topic's info-count ≠ stream rows,
   or a header-bearing topic's `dup_factor > 1.2`. The operator is told to re-record once the sim
   holds real-time — a suspect bag never silently passes.
2. **True rate (only on a consistent bag).** Each topic's rate is the count of **unique** message
   timestamps over their span. The de-dup key is the recorder `log_time` (clean and type-agnostic on
   a consistent bag: reference camera 15.23 / mission 10.05 / vlp 49.97 Hz), which is RTF-invariant.
   The sim-time header stamp is read only for the duplicate-frame guard, and only for the two
   header-leading types where a fixed CDR offset is safe (`CompressedImage`,
   `AprilTagDetectionArray`); px4 (no std Header) and `/tf` (leads with an array) are not stamp-read.
3. The band check itself is **reused verbatim** from `replay_assertions.evaluate` /
   `ObservedTopic` / `load_specs` — the ±40% logic is not re-implemented.

Rejected: setting `use_sim_time` on the mission node + recorder to make `count/bag-duration`
self-consistent — that changes safety-relevant mission timing (A-2 keepalive, dwell/timeout windows)
to fix a measurement artifact (poor risk/reward), and the mission node stays wall-cadenced.

## Consequences

- **Before/after (the falsifiable signal):** `verify_live_bag.py` on the offending bag now **exits 2**
  with explicit reasons (info 9037 ≠ rows 9335 on every asserted topic, plus camera dup_factor 2.06),
  where a raw `ros2 bag info` rate would have shown a bogus 30 Hz "regression". On the **reference**
  bag (and any clean live bag) it **exits 0** with the true RTF-robust rates in band (camera 15.23 Hz,
  mission 10.05 Hz, vlp 49.97 Hz).
- **CI lane unchanged.** `test_replay_regression.py`, `assertions.yaml`, and the reference bag are
  byte-for-byte untouched — the reference bag is recorded at RTF ≈ 1.0 with a clean MCAP, where
  `count / bag-info-duration` is already the true rate. No band was widened (widening would have
  weakened the CI guard).
- **Layer-A coverage.** `rate_report.py` is ROS-free and unit-tested in
  `tests/unit/test_rate_report.py` (unique-stamp rate collapses duplication; the consistency guard
  fires on both info≠rows and dup_factor); `verify_live_bag.py` lives under `tests/replay`
  (`norecursedirs`) and carries no `pytest.mark.ros`, so it is never collected.
- **`analysis/e2e_check.md` step 4** now invokes `verify_live_bag.py` and explains why a raw
  `ros2 bag info` Hz can mislead on a loaded host. `tests/replay/README.md` documents the
  live-witness-vs-CI-lane split.
- **Deferred follow-up (out of scope here):** root-cause *why* the GUI-loaded record path duplicates
  frames and writes an inconsistent MCAP summary — candidates: a second GUI-spawned image bridge, a
  non-clean recorder SIGINT finalize under load, or an MCAP index written before the last chunk. This
  needs a fresh instrumented SITL run; the witness's hard-fail makes such a bag visible in the
  meantime.
- **Coupling:** the sim-stamp dup guard's fixed CDR offset is valid only for the two header-leading
  types it is applied to; a new header-bearing rated topic must be added to `_HEADER_LEADING_TYPES`
  in `verify_live_bag.py` (the info≠rows guard is type-agnostic and needs no change). The `1.2`
  dup-factor threshold is a constant next to its rationale in `rate_report.py`.
