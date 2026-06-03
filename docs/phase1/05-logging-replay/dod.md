# Definition of Done — Logging & Replay Pipeline

**Phase 1 docset:** 5 of 5 · **Milestones:** M7–M8
**Lifecycle status:** DoD ✅ · PRD ⏳ (/drive) · Design ⏳ (/drive)
**Source:** docs/phase1_simulation_plan.md — M7 ("rosbag2 logging, MCAP format, full sensor capture"), M8 ("Replay pipeline: bag → DGX → Foxglove"); plus "Test strategy", "Containerization", "What's explicitly NOT in Phase 1", "Phase 1 exit checklist".
**Stakeholders:** Project owner (solo dev) — operator who runs missions and maintains the pipeline; downstream — every Phase 2+ flight (the bag is the regression test, training corpus, and debugger per plan §"Project context"); reviewers — anyone debugging a mission via Foxglove or replay test.
**Depends on:** 01-platform (sim/dev containers, `colcon` workspace, ROS 2 Jazzy base), 04-perception (`patrol_interfaces/msg/CheckpointCapture` and `/patrol/checkpoint_capture` topic), 02-mission-control (mission launch entrypoint that invokes recording; mission/perception topics to record).
**Consumed by:** Phase 1 exit checklist (integrative item 1, primary items 5–8); all Phase 2+ phases (replay-based regression testing harness, bag corpus).

## 1. Intent
Deliver the logging-and-replay backbone so that every mission run — sim now, real flight later — automatically produces an MCAP rosbag, that bag round-trips through DGX ingestion into a queryable manifest, a replay regression test asserts on its contents in CI, and Foxglove renders it for human inspection. This is the "build the logging/replay pipeline before the first real flight" commitment from the plan: the bag is the regression test, the training corpus, and the debugger.

## 2. Scope
**In scope:**
- Python wrapper around `ros2 bag record` that the mission launch file invokes automatically, recording all relevant topics in MCAP format to a known output location, with a `patrol_<missionId>_<timestamp>.mcap` naming convention.
- Topic selection sufficient for replay and inspection: `/fmu/out/*`, `/patrol/*` (mission + perception, including `/patrol/checkpoint_capture`), camera image topic, TF tree, mission state / current waypoint / abort signals.
- Bag metadata sufficient to identify and replay a run (mission ID, timestamp, topic set; correlation with the mission config).
- Upload daemon: watches the output directory and syncs new bags to the DGX automatically after mission end.
- DGX-side ingestion: indexes each uploaded bag into a manifest (mission, time, duration, topics, metadata sidecar) queryable at Phase 1 scale.
- Replay verification: a CI regression test that replays a checked-in reference bag and asserts expected topics appear at expected rates.
- Foxglove load-and-render verification of a recorded bag (camera feed, mission state, 3D pose history). Foxglove itself is an installed desktop app, not something this docset builds.

**Out of scope (explicit deferrals — item · rationale · target):**
- Containerizing Foxglove · it is a desktop app, "let them be desktop apps" (plan §"Containerization") · never.
- Postgres / production-grade metadata store · SQLite or DuckDB is enough at Phase 1 scale, "don't reach for Postgres yet" (plan M8) · later phase if scale demands.
- Replay-driven *perception* regression on real detector outputs (e.g. YOLO) · real object detection is Phase 3; Phase 1 replay asserts topic presence/rates, not learned-model outputs (plan §"What's NOT in Phase 1", §"Test strategy") · Phase 3.
- Topic pruning / bag-size optimization beyond "reasonable" (under a few hundred MB for a 5-min mission) · start broad, prune later (plan M7) · later phase.
- Bag schema beyond what Phase 1 topics require (e.g. anomaly events) · those messages arrive with their phases · Phase 6+.

## 3. Capabilities (must-do — seeds the PRD's functional requirements)

1. **(P1) Automatic per-run MCAP recording.** Each mission launch produces exactly one MCAP rosbag in a known output directory, recording the agreed topic set, named `patrol_<missionId>_<timestamp>.mcap`.
   *Customer scenario:* operator runs `mission_patrol.launch.py` and, without any extra command, gets a recorded bag for that flight. *Pain removed:* no manual `ros2 bag record` step that gets forgotten — "every mission run produces a bag, no exceptions. This is the discipline" (plan M7).

2. **(P1) Bag identifiability.** `ros2 bag info <bag>` shows expected topics and message counts; the bag carries metadata sufficient to identify the mission and replay it.
   *Customer scenario:* reviewer opens a months-old bag and can tell which mission config and run produced it. *Pain removed:* nameless/contextless bags that can't be correlated to a mission or replayed.

3. **(P1) Automatic upload to DGX.** A daemon watches the output directory and syncs new bags to the DGX automatically after mission end (target: within 30s).
   *Customer scenario:* operator finishes a mission and the bag is on the DGX without manual `rsync`. *Pain removed:* field-data loss and manual copy steps; "the teams that iterate fastest are the ones who never have to re-fly to debug" (plan §"Why Phase 1 matters").

4. **(P1) Manifest indexing.** Each uploaded bag is indexed into a manifest (mission, when, duration, topics, metadata sidecar contents) and is queryable.
   *Customer scenario:* operator lists recent missions and their topics from the manifest instead of grepping a directory of bags. *Pain removed:* no searchable record of what was flown and what each bag contains.

5. **(P1) Replay regression test in CI.** A test replays a checked-in reference bag via `ros2 bag play`, subscribes to expected topics, and asserts they appear at expected rates; it runs in CI.
   *Customer scenario:* a later-phase change that breaks the topic stream is caught by CI before it reaches hardware. *Pain removed:* "this is the foundation of replay-based regression testing for later phases" (plan M8) — regressions otherwise surface on a vibrating airframe.

6. **(P1) Foxglove renders a recorded bag.** A recorded mission bag opens in Foxglove and renders camera feed, mission state, and 3D pose history with the expected panels populated.
   *Customer scenario:* operator debugs a mission visually by scrubbing the recorded bag in Foxglove. *Pain removed:* no visual debugger for mission outcomes.

7. **(P2) Reference-bag provenance note.** The checked-in reference bag used by the replay test is documented (which mission/config produced it, how to regenerate it) so the regression baseline can be refreshed deliberately.

## 4. Acceptance criteria / Definition of Done (falsifiable — seeds the PRD's UACs)

- [ ] **AC-1 (exit-checklist #5):** *Given* a mission run via `ros2 launch patrol_bringup mission_patrol.launch.py`, *when* it completes, *then* exactly one MCAP rosbag exists in the known output location, named `patrol_<missionId>_<timestamp>.mcap`. (M7 Exit)
- [ ] **AC-2 (exit-checklist #5):** *Given* a produced bag, *when* `ros2 bag info <bag>` is run, *then* it lists the expected topics (`/fmu/out/*`, `/patrol/*`, camera image, TF, mission state/waypoint/abort) with non-zero message counts, and bag size is reasonable (under a few hundred MB for a 5-minute mission). (M7 Exit)
- [ ] **AC-3 (exit-checklist #6):** *Given* a mission ends, *when* the upload daemon runs, *then* the bag is uploaded to the DGX automatically (target within 30s of mission end). (M8 Exit)
- [ ] **AC-4 (exit-checklist #6):** *Given* a bag uploaded to the DGX, *when* ingestion indexes it, *then* it appears in the manifest with mission, time, duration, topics, and metadata. (M8 Exit)
- [ ] **AC-5 (exit-checklist #7):** *Given* a checked-in reference bag, *when* the replay regression test runs in CI, *then* it replays the bag, subscribes to expected topics, and asserts they appear at expected rates — passing in CI. (M8 Exit)
- [ ] **AC-6 (exit-checklist #8):** *Given* a recorded mission bag, *when* it is opened in Foxglove, *then* the camera feed, mission state, and 3D pose history render with expected panels populated. (M8 Exit)
- [ ] **AC-7 (exit-checklist #11, consumer side):** *Given* the perception node publishes `patrol_interfaces/msg/CheckpointCapture` on `/patrol/checkpoint_capture`, *when* a mission runs, *then* the bag pipeline records that topic — demonstrating the message is consumed by the bag pipeline as well as the perception node. (M7 topic list; checklist #11)
- [ ] **AC-8 (supports integrative #1):** *Given* a full multi-checkpoint patrol launched via `mission_patrol.launch.py`, *when* it completes, *then* the bag it produced is the artifact that satisfies AC-1 through AC-6 end-to-end (record → upload → manifest → replay → Foxglove). (M8 Exit; checklist #1 integrative)

## 5. Interfaces

**Owns (contracts this docset defines that others depend on):**
- Bag output contract: known output directory path + filename convention `patrol_<missionId>_<timestamp>.mcap`, MCAP storage format, and the recorded-topic set.
- Recording entrypoint: the Python `ros2 bag record` wrapper invoked by the mission launch file (the launch file in 02 calls into this; the wrapper's invocation interface is owned here).
- Metadata sidecar schema: the per-bag metadata fields used for identification/replay and consumed by manifest ingestion.
- Upload daemon → DGX transfer contract (watched directory in, bag on DGX out).
- Manifest schema: the queryable record (mission, time, duration, topics, metadata) and its store.
- Replay regression test contract: the checked-in reference bag + the expected-topics/rates assertions, runnable in CI.
- Directories: `tests/replay/`, `docker/ingest/`, `analysis/`.

**Consumes (from other docsets / PX4):**
- `/fmu/out/*` PX4 telemetry topics (via 01-platform's uXRCE-DDS bridge).
- `/patrol/*` mission + perception topics, camera image topic, TF tree, mission state / current waypoint / abort signals (from 02-mission-control and 04-perception).
- `patrol_interfaces/msg/CheckpointCapture` on `/patrol/checkpoint_capture` (owned by 04-perception; this docset is the named second consumer per checklist #11).
- The `mission_patrol.launch.py` entrypoint (02-mission-control) that triggers automatic recording.
- The `sim`/`dev` containers and `colcon` workspace (01-platform) the pipeline runs inside.

## 6. Settled constraints (do NOT relitigate — cite the source)
- **MCAP storage plugin, not sqlite3.** "MCAP is the format Foxglove and modern tooling target; sqlite is legacy" (plan M7; plan §"How to engage", "MCAP bag format (sqlite is legacy)").
- **Foxglove Studio is the visualizer**, used as an installed desktop app — not containerized (plan §"Quick stack reference", M8, §"Containerization").
- **rosbag2 + MCAP** is the bag layer; **Python 3.12** for the wrapper/daemon/ingestion (plan §"Target stack").
- **Upload daemon stays dumb; complexity lives on the ingestion side** (plan M8).
- **Manifest store is SQLite or DuckDB, not Postgres**, at Phase 1 scale (plan M8).
- **Replay tests are deterministic and assert topic presence/rates**, not simulator-of-a-simulator behavior; "don't try to mock the simulator" (plan §"Test strategy").
- **ROS 2 Jazzy / Ubuntu 24.04** base (ADR-0001 / docs/decisions/0001-distro-and-os.md; plan §"Target stack").

## 7. Open decisions (handed to /drive — each: question · decision target · why open)
- **Bag schema / recorded-topic finalization** · PRD/Design · plan flags "start broad, prune later" — the exact topic list, compressed-vs-raw image, and whether `CheckpointCapture` carries the image by-value or a stored-path reference (per M6's "or a reference to a stored path for large images") affects bag size and replay fidelity; the path-vs-by-value call is shared with 04 and must be settled jointly.
- **Manifest store: SQLite vs DuckDB** · Design · both are acceptable at this scale (plan M8); the choice trades off analytic query ergonomics (DuckDB) vs ubiquity/simplicity (SQLite).
- **Upload transport: SSH/rsync vs S3-compatible storage** · Design · plan M8 lists both as options for the "dumb" producer; depends on DGX-side access pattern.
- **Reference-bag generation & refresh policy** · Design · how the checked-in reference bag is produced, version-controlled (size/LFS), and deliberately regenerated when topics legitimately change — so the replay baseline doesn't rot.
- **Replay assertion strictness** · Design · which topics are asserted and what rate tolerance keeps the test strict-but-not-flaky (plan §"Test strategy": "keep the count small and the assertions strict").
- **CI runtime budget for replay** · Design · the plan explicitly welcomes pushback on "CI runtime budgets and flakiness" for ROS 2 + SITL; replay is the cheaper-than-SITL regression lane and its budget should be set deliberately.
- **Where the upload daemon and ingestion run in CI** · Design · whether DGX upload/manifest steps are exercised in CI (against a stand-in target) or only locally, given no DGX is required for Phase 1 dev (plan §"Dev hardware requirements").

## 8. Assessment signals (so prd-engine right-sizes the PRD)
| Dimension | Value | One-line justification |
|---|---|---|
| Nature | infrastructure | Logging/replay/ingestion plumbing the whole project iterates on, not a user feature. |
| Complexity | complex | Five coordinated components (record wrapper, upload daemon, ingestion+manifest, replay test, Foxglove verification) spanning dev host and DGX. |
| Urgency | standard | Phase-1-blocking but not emergency; sequenced after M1–M6. |
| Risk | medium | Wrong bag schema / format propagates to every later phase's corpus and regression suite; mostly recoverable but costly to migrate. |
| Reversibility | costly-to-reverse | Bag format and schema become the long-lived corpus + regression baseline; "won't be migrating bag formats two phases from now" is the explicit goal. |
| Scope | cross-service | Spans dev-host recording/upload and DGX-side ingestion/manifest; consumes contracts from 01/02/04. |
| Audience | developer | Solo dev / future maintainers and reviewers debugging via bags and Foxglove. |
**Suggested PRD tier:** Standard (Complexity=complex × Risk=medium → Standard per prd-engine's Complexity×Risk matrix, where Complex sits in the Low-Medium column on the Standard cell; cross-service scope keeps it at Standard rather than Lightweight, and it does not reach High-Critical risk so it holds below Comprehensive).

## 9. Traceability
- **Milestones:**
  - M7 — rosbag2 logging, MCAP format, full sensor capture — docs/phase1_simulation_plan.md#m7--rosbag2-logging-mcap-format-full-sensor-capture
  - M8 — Replay pipeline: bag → DGX → Foxglove — docs/phase1_simulation_plan.md#m8--replay-pipeline-bag--dgx--foxglove
- **Exit-checklist items owned:** 5, 6, 7, 8 (primary). Contributes to 1 (integrative — the bag produced by the full patrol) and 11 (the bag pipeline is the named second consumer of `CheckpointCapture`; 04 owns the message).
- **Packages / dirs:** bag-record wrapper invoked from `ros2_ws/src/patrol_bringup` launch (02); `docker/ingest/` (DGX-side ingestion + manifest); `analysis/` (bag analysis); `tests/replay/` (replay regression test + reference bag).
- **Lifecycle:** dod.md (this) → prd.md (via /drive) → design.md (via /drive)
