# Phase 1 wrap-up — findings log

Working document for the Phase 1 exit pass. Every defect, doc error, and review finding surfaced
while running the manual exit-checklist checks lands here **first**; fixes are implemented in one
batch once all checks are done, so the PR #20 merge wall (`dismiss_stale_reviews_on_push: true`)
gets hit as few times as possible.

**Opened:** 2026-07-26 · **Owner:** Egemen Cankaya · **Feeds:** PR #20 (`phase1/wrapup`)

Two sources feed this log:
1. **Manual exit-checklist checks** — the live/GUI runs the devloop explicitly excluded (see
   [`phase1-wrapup-briefing.md`](phase1-wrapup-briefing.md) §"Out of scope for this loop").
2. **The PR #20 automated review** — findings not yet answered.

---

## 1. Status board

| ID | Source | Severity | Summary | Status |
|---|---|---|---|---|
| [F-01](#f-01) | Check 1 | ~~Medium~~ **none** | ~~`checkpoint_capture` 3 of 4 — `cp_west` never captured~~ **FALSE FINDING** — there is no `cp_west`; 3/3 is correct | **Closed — not a defect** (`81a2756`..`e219e04`) |
| [F-02](#f-02) | Check 1 | **Medium** | Mission + perception nodes exit 1 on a clean teardown (double `rclpy.shutdown()`) | **Fixed** (`d51976d`) |
| [F-03](#f-03) | Check 1 | Low | `apriltag_node` segfaults (exit -11) on shutdown — upstream | **Deferred to Phase 2** |
| [F-04](#f-04) | Check 1 | Low | Docs say bags land in `~/patrol_bags`; the runner writes to its mktemp log dir | **Fixed** (docs) |
| [F-05](#f-05) | Pre-check | **High** | SITL nightly aborts at collection — has never been green | **Fixed** (`497c0bd`) |
| [F-06](#f-06) | Pre-check | Low | 05-logging-replay DoD showed AC-3..AC-8 unproven despite the 7/03 witness | **Fixed** (docs) |
| [F-07](#f-07) | Pre-check | Low | README calls `checkpoints.yaml` an interim 02 stand-in; canonical since M5 | **Fixed** (docs) |
| [F-08](#f-08) | PR #20 review | **Medium** | Budget harness fed one JUnit → flake rate is binary, not multi-night | **Fixed** (`497c0bd`) |
| [F-09](#f-09) | PR #20 review | **Medium** | Low-battery test does not prove the injected sample caused the abort | **Fixed** (`8de636b`) — option 2 + option 1 |
| [F-10](#f-10) | Check 4 | Low | Docs still say low-battery abort is "unit-tested", never SITL-observed | **Fixed** (docs) |
| [F-11](#f-11) | Check 4 | **Medium** | `--keep-up --no-patrol` prints a teardown command that SIGINTs the operator's own shell | **Fixed** (`81a2756`) |

**Fixed in code:** F-02, F-05, F-08, F-09, F-11 (+ F-01 closed as a false finding, with an
invariant test so it cannot recur).
**Doc-only:** F-04, F-06, F-07, F-10.
**Deferred to Phase 2:** F-03 (upstream `apriltag_ros`).

**Verification status.** All Layer-A work is verified: 850 unit tests green, `ruff` clean,
shellcheck clean, and both new code fixes (F-02, F-11) were confirmed RED against the pre-fix code
before being fixed. **Not verified live:** F-09's SITL assertions and F-08's artifact retention path
both need a green nightly (blocked on F-05, which itself needs to be pushed and run).

See also [§5 Observations](#5-observations-no-fix-planned) for things seen during the checks that are
deliberately *not* being tracked as findings.

---

## 2. Exit-checklist check status

| # | Exit item | Check | Status |
|---|---|---|---|
| 1 | Full patrol via `mission_patrol.launch.py` | Check 1 | ✅ **PASS** 2026-07-26 |
| 2 | Mission config from checked-in YAML | static | ✅ PASS |
| 3 | State-machine unit suite >80% cov, <5 s | CI | ✅ PASS — 100% cov, 0.20 s (228 tests) |
| 4 | SITL integration test passes in CI | Check 6 | ⛔ **BLOCKED on F-05** — never green |
| 5 | Every run produces an MCAP bag | Check 1 | ✅ PASS |
| 6 | Bags auto-upload + appear in manifest | Check 3 | ✅ **PASS** 2026-07-26 — upload 2 s, manifest `171s / 34 topics` = `ros2 bag info` |
| 7 | Replay regression in CI | CI | ✅ PASS — required lane, pinned GHCR image |
| 8 | Foxglove renders camera/state/pose | Check 2 | ✅ **PASS** 2026-07-26 |
| 9 | `docker compose` build + in-container `colcon build` | local | ✅ PASS 2026-07-26 — both images from scratch, 7 pkgs exit 0 |
| 10 | README setup-to-mission <20 commands | Check 5 | ⬜ Pending — needs a clean VM |
| 11 | `CheckpointCapture` used by perception + bag pipeline | Check 1 | ✅ **PASS** — F-01 closed as a false finding; capture rate is 3/3 |
| 12 | Abort (low-battery + external) observable in SITL | Check 4 | ✅ **PASS** 2026-07-26 — both guards fired live |

---

## 3. Findings

### F-01 — ~~`checkpoint_capture` count is 3 of 4~~ CLOSED: not a defect {#f-01}

**Severity:** ~~Medium~~ **none** · **Owner:** — · **Exit item:** 11 (now unambiguously ✅)

> **RESOLUTION (investigated before patching, as required).** This finding is **false**. There is no
> `cp_west` — the string appears nowhere in the repo except this document and the one line in
> `analysis/e2e_check.md` that repeated the claim (both now corrected).
>
> The route has **four waypoints but three checkpoints**:
>
> | Source | Count |
> |---|---|
> | `sim/config/checkpoints.yaml` | **3** — `cp_north`, `cp_east`, `cp_south` |
> | `sim/models/apriltag_36h11_*` | **3** |
> | tag includes in `sim/worlds/patrol_world.sdf` | **3** |
> | waypoints in `patrol_mission.yaml` | **4** |
>
> The fourth waypoint is an **inline ENU overlook** at `(5.0, 5.0, 2.5)` with no `checkpoint_id` and
> no AprilTag near it, kept deliberately "to exercise the inline-waypoint path alongside checkpoint
> refs" — the mission YAML's own header comment says so.
>
> So `/patrol/dwell` 4 with `checkpoint_capture` 3 is **exactly correct**: dwell fires once per
> waypoint, a capture requires a tag to resolve. Perception behaves correctly at the overlook —
> `CaptureCoordinator._first_detection()` returns `None`, logs the ADR-A gate skip at INFO, and
> deliberately does not latch. **The real capture rate is 3/3 = 100%, not the reported 75%.**
>
> None of the three candidate causes below applies; there was never a tag there to resolve, so
> tag orientation, detection freshness, and lighting are all beside the point.
>
> **What landed instead of a fix:** `tests/unit/test_patrol_capture_expectation.py`, which pins the
> expected capture count to the checkpoint-bearing waypoints (composing the real
> `load_mission_config`, so there is no second source of truth) and cross-checks the count against
> the model library and the composed world. The same misreading now costs a failing test rather than
> an investigation.
>
> **Process note worth keeping:** this was logged as a reproducible Medium defect on the strength of
> two runs agreeing — but both runs agreeing is exactly what correct behavior looks like too.
> "Reproducible" distinguished a real effect from flake; it did not establish that the effect was a
> *fault*. The expected value was never derived from the config before the observed value was called
> wrong.

<details>
<summary>Original finding as filed (retained for the record)</summary>

Two independent full patrols produced **3** captures, not 4:

| Run | Bag | `/patrol/dwell` | `/patrol/checkpoint_capture` |
|---|---|---|---|
| 2026-07-03 | `patrol_20260703T070221Z_...` | 4 | 3 |
| 2026-07-26 | `patrol_20260726T110419Z_...` | 4 | 3 |

On-disk captures from the 7/26 run confirm which one is missing:

```
000_cp_north.png + .json
001_cp_east.png  + .json
002_cp_south.png + .json      ← no cp_west
```

**Why this is not the ADR-0012 dwell bug:** `/patrol/dwell` is **4** in both runs, so the drone did
reach and dwell at all four checkpoints. The stand-off/framing fix is working. The failure is
downstream — perception did not resolve a tag into a capture during the `cp_west` dwell window.

Reproducible across two runs a month apart, so it is a real defect, not SITL flake.

**Candidate causes** (untested — first fix step is to narrow this):
- `cp_west`'s AprilTag orientation/yaw in `sim/config/checkpoints.yaml` vs the approach pose.
- Detection-freshness gate (`max_detection_age_s`) timing out on the last checkpoint.
- Lighting/contrast on that tag's face in `patrol_world.sdf`.

**Repro:**
```bash
PX4_SIM_SPEED_FACTOR=0.33 PATROL_OUTPUT_ROOT=$HOME/patrol_bags \
  scripts/run_patrol_world_sitl.sh --skip-doctor
ros2 bag info $BAG | grep -E "checkpoint_capture|/patrol/dwell"
```
Then diff the `cp_west` dwell window against a working one on `/tag_detections`.

</details>

---

### F-02 — Mission and perception nodes exit 1 on a clean teardown {#f-02}

**Severity:** Medium · **Owner:** 02-mission-control + 04-perception · **Status:** **Fixed** (`d51976d`) — both `main()`s now catch `ExternalShutdownException`, and the mission node's `finally` uses the idempotent `try_shutdown()`. Confirmed RED against both pre-fix entrypoints before fixing: exactly the 4 external-shutdown assertions failed while the `KeyboardInterrupt` cases still passed

A **successful** patrol still ends with three `process has died` errors. Two of them are ours:

```
[ERROR] [patrol_mission-5]:   process has died [exit code 1]
[ERROR] [perception_node-3]:  process has died [exit code 1]
```

Root cause, from `node.log`:

```
rclpy.executors.ExternalShutdownException
During handling of the above exception, another exception occurred:
  File ".../patrol_mission/node.py", line 393, in main
    rclpy.shutdown()
rclpy._rclpy_pybind11.RCLError: failed to shutdown: rcl_shutdown already called
                                on the given context, at ./src/rcl/init.c:333
```

`ExternalShutdownException` *means* the context is already shut down — the runner group-SIGINTs the
launch after `verify_patrol.py` observes the landing.

**The two nodes fail for different reasons** (same symptom, so fix both, but do not assume one patch
covers both):

| Node | `except` clause | `finally` | Why it exits 1 |
|---|---|---|---|
| [`patrol_mission/node.py:389-393`](../ros2_ws/src/patrol_mission/patrol_mission/node.py#L389-L393) | `KeyboardInterrupt` only | `rclpy.shutdown()` | Unguarded `shutdown()` on an already-shutdown context raises `RCLError`; the `ExternalShutdownException` is also uncaught |
| [`patrol_perception/perception_node.py:230-234`](../ros2_ws/src/patrol_perception/patrol_perception/perception_node.py#L230-L234) | `KeyboardInterrupt` only | `rclpy.try_shutdown()` ✅ | Teardown is already safe — the exit 1 is purely the **uncaught `ExternalShutdownException`** propagating out of `main()` |

**Why it matters beyond cosmetics:** a launch that *always* exits non-zero on the happy path trains
you to ignore its exit code, so a genuine node crash during teardown will look identical to a normal
run. It also means the runner and any CI wrapper cannot use exit status as a signal.

**Fix:** catch the exception in both, and make the mission node's teardown non-raising:

```python
from rclpy.executors import ExternalShutdownException

try:
    rclpy.spin(node)
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node()
    rclpy.try_shutdown()      # perception already does this; mission does not
```

Add a Layer-A unit test asserting `main()` returns 0 when the context is already shut down — the
existing `test_node_glue.py` / `test_perception_node_glue.py` are the right homes.

---

### F-03 — `apriltag_node` segfaults on shutdown {#f-03}

**Severity:** Low · **Owner:** upstream `apriltag_ros` · **Status:** **Deferred to Phase 2** (not ours to fix) — tracked as [SWM-85](https://linear.app/wemodulate/issue/SWM-85/apriltag-node-segfaults-exit-11-on-shutdown-upstream-apriltag-ros) in *Patrol Drone 04 Perception*, Backlog

```
[ERROR] [apriltag_node-2]: process has died [exit code -11]   # SIGSEGV
```

Third-party node (`/opt/ros/jazzy/lib/apriltag_ros/apriltag_node`), crashing during SIGINT teardown
*after* the mission has completed and captures are written. No effect on the bag or the captures.

Not ours to fix. Log it, confirm it stays teardown-only, and revisit if it ever segfaults mid-run.

---

### F-04 — Bag output location is documented wrong {#f-04}

**Severity:** Low · **Owner:** 05-logging-replay docs · **Status:** **Fixed** — `analysis/foxglove/README.md` now carries a produced-by/output-dir table naming both paths, the `PATROL_OUTPUT_ROOT` durable-location recipe, and the `px4.log` cleanup note; `analysis/e2e_check.md` step 1 resolves the path

[`analysis/foxglove/README.md:13`](../analysis/foxglove/README.md) says the default output dir is
`~/patrol_bags/`, and [`analysis/e2e_check.md`](../analysis/e2e_check.md) step 1 says "the run output
dir" without resolving it. Both are misleading for the runner path:

```bash
# scripts/run_patrol_world_sitl.sh:562
local run_root="${PATROL_OUTPUT_ROOT:-${LOG_DIR}/run}"
```

`LOG_DIR` is a **mktemp dir** (`/tmp/patrol-world-uat.XXXXXX`). So a runner-produced bag lands in
`/tmp`, not `~/patrol_bags` — and is lost on reboot. `~/patrol_bags` is only the default when
`mission_patrol.launch.py` is invoked directly.

This cost real time on 2026-07-26: the 7/26 bag looked missing because `~/patrol_bags` was empty.

**Fix:** state both paths in `e2e_check.md` step 1 and the Foxglove README, and mention
`PATROL_OUTPUT_ROOT=$HOME/patrol_bags` as the way to get a durable location.

**Related housekeeping:** the runner leaves a **1.17 GB** `px4.log` in its temp dir per run. Worth a
size cap or a note to clean up.

---

### F-05 — SITL nightly aborts at collection; has never been green {#f-05}

**Severity:** High · **Status:** **Fixed** (committed in `497c0bd`, alongside F-08 — same file) · **Exit item:** 4 (still needs a green nightly for evidence)

Zero successful runs in the last 30 (back to 2026-06-26). Older failures were sim-image build
errors; the last two nights got further and died at collection:

```
E   ModuleNotFoundError: No module named 'ingest'
```

`tests/integration/test_upload_ingest_standin.py` resolves `_REPO = parents[2]`, but the nightly
copies only `tests/`, `sim/`, `scripts/` into `/opt` — so `parents[2]` is `/` and `analysis/` +
`docker/` are absent. A bad import is a **collection error, not a deselect**, so the whole `-m ros`
run aborts before a single SITL scenario executes.

**Fix applied** in `.github/workflows/sitl-nightly.yml`: `--ignore` that file. It is owned by the
required `replay-regression` lane, which materializes the LFS bag and sets the paths.

**Verified** by reproducing the container's directory depth: directory-arg collection fails with the
identical error without the flag and collects cleanly with it.

Until this is pushed and a nightly runs green, **exit item 4 has no evidence** and the new SWM-15/32
scenarios have never executed.

---

### F-06 — 05-logging-replay DoD understated what was proven {#f-06}

**Severity:** Low · **Status:** **Fixed** (committed with the doc batch)

AC-3..AC-8 were all unticked despite the 2026-07-03 end-to-end witness. The SWM-16/33 true-up
covered only docsets 01 and 02, so 05 was never reconciled. AC-7 still carried a "non-zero count
deferred" clause that ADR-0012 had already closed.

Ticked with evidence citations; AC-7's clause rewritten as closed.

---

### F-07 — README describes `checkpoints.yaml` as an interim stand-in {#f-07}

**Severity:** Low · **Status:** **Fixed** (committed with the doc batch)

The M4 quickstart called `sim/config/checkpoints.yaml` "an interim 02 stand-in until 03 lands its
own". It has been 03's canonical file since M5 — the file's own header says so.

---

### F-08 — Budget harness gets one JUnit, so flake rate is binary {#f-08}

**Severity:** Medium · **Source:** PR #20 review (jxstanford, 2026-07-25) · **Issue:** SWM-31 · **Status:** **Fixed** (`497c0bd`) — nightly retains each run's JUnit as a run-numbered artifact and pulls the last 10 back before measuring, and the harness now states its sample size and labels an under-powered one (needed size derived from the quarantine threshold, not hard-coded). The retention path itself is unverified until a nightly runs

> `.github/workflows/sitl-nightly.yml:100` — the workflow invokes `measure_sitl_budget.py` with only
> the current `/opt/sitl-junit.xml` [...]. The same workflow says multiple nightly reports must
> accumulate before replacing the provisional budget, but it neither retains nor retrieves prior
> reports. Per-scenario flake rate is therefore binary for a normal one-run input (0% or 100%), not
> the intended multi-night measurement.

**Accepted — this is correct, and now demonstrated.** The harness itself takes `RUN1.xml RUN2.xml
...`, but the workflow only ever hands it one, so the flake-rate half of the quarantine rule cannot
function. The 2026-07-26 live run shows it exactly — every scenario reports `runs 1, fails 0,
flake 0%`, which is not a measurement of anything:

```
scenario                                                runs fails  flake   max_s  verdict
test_fmu_bridge_surface_and_liveness                       1     0     0%    10.0   ok
test_external_abort_mid_patrol_drives_observable_rth       1     0     0%    24.4   ok
test_patrol_visits_all_waypoints_then_returns_home         1     0     0%    81.7   ok
test_low_battery_mid_patrol_drives_observable_rth          1     0     0%    24.4   ok
```

**Fix:** retain a rolling set of JUnit reports (artifact upload + download of the last N runs, or a
committed results dir), feed all of them, and report the sample count in the job summary so a
1-sample verdict is visibly not a measurement.

---

### F-09 — Low-battery test does not prove causal attribution {#f-09}

**Severity:** Medium · **Source:** PR #20 review (jxstanford, 2026-07-25) · **Status:** **Fixed** (`8de636b`) — **option 2 then option 1**, per decision. `/patrol/abort_reason` publishes the latched `AbortReason` (option 2, the honest fix, closing the real observability gap) AND each scenario asserts the inbound `/patrol/abort` count (option 1's negative evidence). SITL assertions unverified live — needs a green nightly

> `tests/integration/test_mission_patrol_low_battery.py:104,109` — the test [...] does not prove that
> an injected sample was consumed by the mission node or that the low-battery guard, rather than
> another event on the shared status path, caused the transition.

**Accepted with a caveat.** The test's docstring argues attribution *by exclusion* (only two live
guards; no `/patrol/abort` is published). That is a fair argument but it is not an assertion — the
test would still pass if the abort came from somewhere else.

**Fix options, cheapest first:**
1. Assert the negative explicitly: subscribe to `/patrol/abort` and assert **zero** messages, so an
   external-signal abort cannot be the cause.
2. Publish an observable abort *reason* on the `/patrol/*` surface. Currently `mission_state` carries
   the state but not the `AbortReason` — this is the real gap and would also help field debugging.

Option 2 is the honest fix but widens the 02 topic contract; worth deciding deliberately rather than
defaulting to option 1.

**Update 2026-07-26 — the test now passes live** (24.4 s, first execution anywhere). That materially
lowers the risk: the injection path works end to end and produces an abort → RTH → settle → disarm.
The attribution gap is *unchanged* though — no `/patrol/abort` was published by that test, so the
by-exclusion argument holds, but nothing in the test **asserts** it. Note also that the external and
low-battery scenarios both took **24.4 s**: identical, because both inject as soon as the patrol is
underway. Expected, not suspicious — but it does show the two are behaviorally indistinguishable
from the outside, which is precisely the reviewer's point.

---

### F-10 — Docs still say the low-battery abort has never fired in SITL {#f-10}

**Severity:** Low · **Source:** Check 4 · **Status:** **Fixed** — all three locations corrected (02 DoD in `8de636b`; `CLAUDE.md` M4 line + the low-battery test's own stale "pending validation" docstring in the doc batch; 05 AC-7 note closed under F-06)

The 2026-07-26 live pass invalidates several claims:

| Location | Says | Should say |
|---|---|---|
| [`02-mission-control/dod.md:129`](phase1/02-mission-control/dod.md) | "external observable in SITL, low-battery **unit-tested**" | both observable in SITL (2026-07-26) |
| [`CLAUDE.md:18`](../CLAUDE.md) | M4 ships "a nightly patrol SITL test with a mid-patrol external-abort assertion" | + the low-battery abort scenario, both live |
| `05-logging-replay/dod.md` AC-7 note | flags a "SITL-observation gap" for the abort half | closed |

Fold into the batch commit — same class as F-06/F-07.

---

### F-11 — `--keep-up` teardown hint SIGINTs the operator's own shell {#f-11}

**Severity:** Medium · **Owner:** 03/tooling · **Found:** Check 4, 2026-07-26 · **Status:** **Fixed** (`81a2756`) — the node rung is emitted only when `NODE_PID` is set, locked by `tests/unit/test_keep_up_teardown_hint.sh` (+ pytest wrapper). Confirmed RED against the pre-fix function, which reproduced the reported hint verbatim. **Audit done:** line 624 was the only `${VAR:-0}` default interpolated into a signal target in this file

With `--no-patrol --keep-up`, the runner prints:

```
tear down: kill -INT -- -0; kill -- -20223; kill 19994 19995 19996  21209
                        ^^
```

[`scripts/run_patrol_world_sitl.sh:624`](../scripts/run_patrol_world_sitl.sh#L624):

```bash
log "  tear down: kill -INT -- -${NODE_PID:-0}; kill -- -${PX4_PID}; kill ${AGENT_PID} ..."
```

`--no-patrol` means no mission node is ever launched, so `NODE_PID` is unset and the `:-0` default
expands to `0`. Under POSIX `kill`, a target of `-0` is **the caller's own process group** — so
copy-pasting the runner's own instruction sends SIGINT to the operator's shell and every job in it,
while leaving the actual stack (agent, gz, PX4, bridge) running and still holding port 8888.

Two failures in one: it does the wrong destructive thing, *and* it does not do the right thing. The
leftover agent on 8888 then produces the classic stale-stack symptoms on the next run (QGC haywire /
no GPS / mag failure), which read as sensor bugs.

Only reachable on the `--no-patrol` path — the same path the Check 4 runbook prescribes.

**Fix:** emit the node-group kill only when there is a node group.

```bash
local node_kill=""
[[ -n "${NODE_PID:-}" ]] && node_kill="kill -INT -- -${NODE_PID}; "
log "  tear down: ${node_kill}kill -- -${PX4_PID}; kill ${AGENT_PID} ${GZ_PID} ${GZ_GUI_PID:-} ${QGC_PID:-} ${BRIDGE_PID}"
```

Worth a shell unit test alongside the existing `test_stop_launch_group.sh` asserting the hint never
contains a bare `-0` target. Also audit the file for other `${VAR:-0}` defaults interpolated into
signal targets.

## 4. Remaining checks

Findings from these get appended above as they are found.

- [x] **Check 1** — full patrol → F-01 (false finding, closed), F-02, F-03, F-04
- [x] **Check 2** — Foxglove render → no findings
- [x] **Check 3** — upload → ingest → manifest → replay on the 7/26 bag → **PASS**, no findings
- [x] **Check 4** — abort × 2 → **4/4 passed**, 140.7 s → F-10
- [ ] **Check 5** — README clean-VM walk
- [ ] **Check 6** — nightly SITL green (needs F-05 pushed)
- [x] **Check 7** — budget harness fed → measured figure below, F-08 confirmed

### Check 4 run record — 2026-07-26

Host: GUI/X11, `PX4_SIM_SPEED_FACTOR=0.33` (RTF ≈ 1). Stack brought up with
`run_patrol_world_sitl.sh --skip-doctor --no-patrol --keep-up`; camera verified steady at
**15.150 Hz** with the `/compressed` companion present; QGC started manually (the runner reports
`qgc=none` under `--no-patrol`, and PX4 refuses to arm offboard without a GCS heartbeat).

```
4 passed, 3 warnings in 140.71s (0:02:20)
  tests/integration/test_fmu_bridge_surface.py           .     [ 25%]
  tests/integration/test_mission_patrol.py               ..    [ 75%]
  tests/integration/test_mission_patrol_low_battery.py   .     [100%]
```

Significance: `test_low_battery_mid_patrol_drives_observable_rth` had **never executed anywhere** —
not in CI, not locally, not in the nightly (which has never been green). It was written against the
design and shipped unproven. It passed on first execution, which is what closes exit item 12's
low-battery half and the AC-7 SITL-observation gap. `test_fmu_bridge_surface` passing also gives 01
DoD AC-2/AC-3 a real assertion instead of the nightly's shell-level bridge gate.

JUnit retained at `/tmp/sitl-run1.xml` — **move it somewhere durable** if it is to serve as run 1 of
the F-08 rolling set.

### Measured SITL budget (SWM-31)

First real measurement, 2026-07-26, GUI host at `PX4_SIM_SPEED_FACTOR=0.33` (RTF ≈ 1):

| Scenario | Wall-clock |
|---|---|
| `test_patrol_visits_all_waypoints_then_returns_home` | **81.7 s** ← observed max |
| `test_external_abort_mid_patrol_drives_observable_rth` | 24.4 s |
| `test_low_battery_mid_patrol_drives_observable_rth` | 24.4 s |
| `test_fmu_bridge_surface_and_liveness` | 10.0 s |

Provisional budget is **480 s** — roughly **6× the observed max**.

**Do not fold 81.7 s in yet.** Two reasons: (a) one sample, and the harness's own guidance is "once
stable"; (b) this was measured on a GUI host with hardware rendering, while the nightly runs
headless with software render in-container — those numbers will differ and could be much slower.
Take the measurement from the nightly (blocked on F-05), or set the budget at the nightly's observed
max with headroom rather than at this figure.

## 5. Observations (no fix planned)

Seen during the checks, deliberately **not** tracked as findings — recorded so they are not
re-investigated later. Promote to an F-number only if one starts costing time.

- **`Warning: There is no current event loop` ×3** (Check 4, one per launch-based scenario). An
  asyncio deprecation raised inside `launch` under Python 3.12, not our code. Harmless; will
  disappear when upstream `launch` stops calling `get_event_loop()`.
- **External and low-battery abort scenarios both take exactly 24.4 s.** Expected, not a copy-paste
  artifact: both inject as soon as the patrol is underway, so both fly an identical profile to the
  abort point. Worth remembering that it also means the two are indistinguishable by timing — the
  substance of [F-09](#f-09).
- **Runner reports `qgc=none` under `--no-patrol`.** By design (`start_qgc()` early-returns unless
  `RUN_PATROL=1 && WITH_GUI=1`). Only a trap in that you must start QGC yourself or offboard arming
  fails with "No connection to the GCS".
- **`test_patrol_visits_all_waypoints_then_returns_home` takes 81.7 s, but a runner patrol produces a
  ~170 s bag.** Not a discrepancy: the test measures launch → assertion satisfied, while the bag
  spans recorder start (pre-arm) through finalize (post-land teardown). Different windows, both
  correct. Not verified in detail — if the gap ever matters, measure it rather than assume.

## 6. Fix batching

The merge wall dismisses every standing approval on each **push**, not each commit — so the batch is
**six commits in a single push**. One dismissal either way, but the topic-contract change stays
bisectable from the teardown fixes and CodeScene's per-file delta stays legible.

| # | Commit | Carries |
|---|---|---|
| 1 | `81a2756` | F-11 — teardown hint must not target the caller's process group |
| 2 | `d51976d` | F-02 — exit 0 on a clean teardown (both nodes) |
| 3 | `e219e04` | F-01 — invariant test pinning captures to checkpoint waypoints |
| 4 | `8de636b` | F-09 — `/patrol/abort_reason` + attribution assertions (+ F-10's 02 DoD half) |
| 5 | `497c0bd` | F-08 rolling JUnit sample + **F-05**'s `--ignore` (same file, was uncommitted) |
| 6 | *(this one)* | F-01/F-04/F-10 doc corrections + F-06/F-07 + this findings log |

Also:

1. F-03 → Phase 2 backlog (upstream `apriltag_ros`); Linear issue filed, no code change.
2. Answer any non-blocking review findings by **PR comment, never a commit**.
3. An org admin still needs to add a ruleset bypass actor before #20 can merge.
4. **Do not** fold the measured 81.7 s into `tests/integration/sitl_budget.yaml` yet — it was
   measured on a GUI host with hardware rendering; the nightly is headless/software-render. The
   budget stays `provisional: true` until a green nightly (blocked on F-05) supplies the figure.
   The harness now prints a sample-size warning that says this at the point of use.
