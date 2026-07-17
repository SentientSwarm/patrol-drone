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

## Deferred follow-up — RESOLVED (finalize/truncation half)

**Date:** 2026-07-03. The deferred follow-up above had two intertwined symptoms; the
**non-finalized / truncated** half is now root-caused and fixed. (The **frame-duplication** half
stays as stated: it is RTF-driven — record at RTF ≈ 1 on an idle GPU for a clean 15 Hz stream — and
the witness's consistency guard still hard-fails a duplicated bag.)

- **Root cause (finalize/truncation).** The UAT runner
  [`scripts/run_patrol_world_sitl.sh`](../../scripts/run_patrol_world_sitl.sh) tore the recorder down
  before rosbag2 finalized. `verify_patrol.py` returns the instant the patrol is observably complete
  (landed/disarmed) — it does not wait for the recorder to flush — after which the runner's `shutdown`
  sent `kill -TERM` to the `ros2 launch` process and immediately TERMed PX4/gz. Confirmed in the
  `launch` source: **SIGTERM** makes `ros2 launch` cancel its asyncio task and orphan its children
  (`launch_service.py` literally logs *"using SIGTERM can result in orphaned processes"*), so the
  recorder was killed before writing `metadata.yaml`; only **SIGINT** emits the clean `Shutdown` that
  SIGINTs `ros2 bag record` so it finalizes the MCAP — the very "the launch system SIGINT-finalizes
  the MCAP at shutdown" contract [`record.launch.py`](../../ros2_ws/src/patrol_logging/launch/record.launch.py)
  already documents. Even a correct SIGINT was useless while PX4/gz (the recorder's data sources) were
  killed in the same breath.
- **Fix.** The runner now stops the mission launch **cleanly**: a `graceful_stop_mission` step SIGINTs
  the launch after a passing verify and waits (bounded, `FINALIZE_WAIT`, default 90 s) for it to exit
  and for `metadata.yaml` to appear — *before* the stack is torn down and before the bag-content
  assertion runs, so a finalized bag exists by construction. `shutdown` likewise SIGINTs the launch and
  waits for it before killing PX4/gz on the interrupt / failed-verify / camera-only paths. As
  caller-independent insurance, the recorder `ExecuteProcess` in `record.launch.py` gets an explicit
  `sigterm_timeout` (60 s) so rosbag2 gets a real flush window on any clean shutdown (launch's 5 s
  default is tight for a ~100 MiB MCAP). No change to the ROS-free recorder core, the sidecar, the
  acceptance oracle (`verify_patrol.py`), or the CI `replay-regression` lane (still byte-unchanged).
- **Acceptance.** One `run_patrol_world_sitl.sh` run (RTF ≈ 1) now yields **one** finalized bag
  (`metadata.yaml` present, no reindex), duration matching the full flight, `checkpoint_capture`
  Count ≥ 4, `ros2 bag info` succeeds, and `verify_live_bag.py --bag <bag>` exits 0 on the complete
  bag — the single-artifact bag the LR-8 / AC-8 witness (SWM-82) needs.

### Follow-up — process-group signal (the concrete orphaning mechanism)

**Date:** 2026-07-03. A live acceptance run proved the SIGINT-not-SIGTERM fix above was **necessary
but not sufficient** on the failure/interrupt path — it exposed *the* mechanism behind the earlier,
vaguer "a non-clean recorder SIGINT finalize under load" hypothesis. This is that root cause, resolved.

- **What the run showed.** The patrol FAILed acceptance (an RTF-driven dwell-flap, still environmental
  per this ADR), so `verify_patrol.py` returned non-zero and `fly_and_verify_patrol` returned *before*
  reaching `graceful_stop_mission` (the pass-only step) — execution fell through to `shutdown`.
  `shutdown` sent its SIGINT and killed PX4/gz, the runner exited reporting teardown — yet **no
  `metadata.yaml` was written**, `ros2 bag info` failed, and a `ros2 bag record` process was left
  **orphaned to init** holding a ~250 MiB unfinalized `.mcap`.
- **Mechanism.** The mission launch was started with a plain `ros2 launch … & NODE_PID=$!` — **no
  `setsid`**, so it had no dedicated process group. `kill -INT "${NODE_PID}"` signalled only the
  `ros2 launch` parent; its `ros2 bag record` grandchild was reparented to init and never received the
  shutdown, so rosbag2 never ran its SIGINT→`sigterm_timeout` finalize. (PX4 was already immune — it
  is `setsid`-started and torn down with the negative-PID group kill `kill -TERM -- "-${PX4_PID}"`.)
  A manual probe confirmed the orphaned recorder **ignored SIGINT and finalized only on SIGTERM** on
  this host.
- **Fix.** The mission launch is now started with `setsid` (own process group, `PGID == NODE_PID`,
  mirroring `start_px4`), and both `graceful_stop_mission` and `shutdown` signal the **whole group**
  via a shared `stop_launch_group` helper: `kill -INT -- "-${NODE_PID}"` first (the clean `ros2 launch`
  Shutdown that `record.launch.py`'s `sigterm_timeout` is built for), then — if the group outlives
  `FINALIZE_WAIT` — escalate to `kill -TERM -- "-${NODE_PID}"` (which this host's orphaned recorder
  honored). PX4/gz/agent are still killed only *after* the recorder group has exited, so the recorder
  finalizes against live data sources. The diff is limited to the runner; `record.launch.py`, the
  recorder core, `verify_patrol.py`, and the CI `replay-regression` lane are untouched.
- **Acceptance (failure path too).** On **both** a passing and a failing/interrupted patrol the run now
  leaves exactly one finalized bag (`metadata.yaml` present, `ros2 bag info` succeeds without reindex)
  and **no surviving `ros2 bag record` process** (`pgrep -f 'ros2 bag record'` empty after the runner
  exits).
