# Logging & Replay Pipeline — PRD

> **One-liner:** Make every mission run automatically produce an MCAP rosbag that round-trips bag → DGX manifest → replay-regression-test → Foxglove, so the bag becomes the project's regression test, training corpus, and debugger from day one.

**Date:** 2026-06-03
**Status:** Draft (rev 2 — post self-review)
**Owner:** Project owner (solo dev)
**DRI:** jxstanford@wemodulate.energy

**Docset:** docs/phase1/05-logging-replay (5 of 5) · **Milestones:** M7–M8
**Source of truth:** docs/phase1_simulation_plan.md (M7, M8, "Test strategy", "Containerization", "Phase 1 exit checklist") and this docset's `dod.md`.
**Tier:** Standard (Complexity=complex × Risk=medium → Standard; cross-service scope adds the Cross-Service Impact section).

## Changelog

- **rev 2 (2026-06-03):** Self-review (prd-engine ReviewPRD) revise pass. (1) Extended §Success Metrics so every P1 FR has a tracking metric — added rows for LR-4 (manifest query), LR-7 (CheckpointCapture recorded), and LR-8 (end-to-end single-artifact pass); cross-listed which FR each metric signals. (2) Made the bag-size metric's pass/fail crisp (measurement = `ros2 bag info`/file size on the canonical 5-min patrol, ceiling = the plan's "few hundred MB" bound) and recorded the *precise* MB-ceiling question as OQ-9 rather than inventing a number the DoD left as "reasonable."
- **rev 1 (2026-06-03):** Initial draft from `dod.md`.

## Overview

This docset delivers the logging-and-replay backbone for the autonomous-drone project. Every mission run — sim now, real flight later — must automatically produce an MCAP rosbag; that bag must round-trip through DGX ingestion into a queryable manifest, a replay regression test must assert on its contents in CI, and Foxglove must render it for human inspection. The plan states the commitment directly: "the logging and replay pipeline must exist before the first real flight, not after … the bag is the regression test, the training corpus, and the debugger."

The work spans five coordinated components across two hosts: a Python `ros2 bag record` wrapper (invoked automatically by the mission launch file), an upload daemon (dev host → DGX), DGX-side ingestion + a queryable manifest, a replay regression test that runs in CI, and Foxglove load-and-render verification of a recorded bag. This is infrastructure the whole project iterates on, not a user feature — and the bag format / schema it locks in becomes the long-lived corpus and regression baseline, which is why it is sized at Standard despite being Phase-1 pre-hardware.

## Problem Statement

> **When** an operator runs a patrol mission (in sim now, on hardware later) and later needs to debug, regression-test, or learn from what the drone did,
> **they struggle with** the fact that nothing reliably captures the run — bags get forgotten, are nameless and uncorrelated to a mission, sit on the dev host, and have no automated check that their topic stream is intact,
> **which means** every debugging or regression question forces a re-fly on a vibrating airframe, and regressions surface in flight instead of in CI.

Today there is no logging/replay pipeline at all. The plan's whole argument for building this in Phase 1 — "when the only thing logging is a simulator and there's no field-data anxiety" — is that the alternative (building it after the first real flight) means "the teams that iterate fastest are the ones who never have to re-fly to debug" become the teams you are *not*. The bag schema and format chosen here propagate to every later phase's corpus and regression suite, so getting it right once, now, is the explicit goal ("won't be migrating bag formats two phases from now").

## Goals

### Business goals
- Make the bag the durable regression-test / training-corpus / debugger asset the plan commits to, established before any hardware spend.
- Lock a bag format (MCAP) and metadata/manifest schema that survive into Phase 2+ without a migration.

### User goals
- The operator gets a recorded, identifiable bag for every mission run with zero extra commands.
- The operator can find and inspect past runs (manifest query + Foxglove) instead of grepping a directory.
- A later-phase change that breaks the topic stream is caught by CI, not in flight.

### Non-goals
- Containerizing Foxglove — it is a desktop app; "let them be desktop apps" (plan §"Containerization").
- A production-grade metadata store (Postgres) — SQLite/DuckDB is enough at Phase 1 scale (plan M8).
- Replay-driven *perception* regression on learned-model outputs (e.g. YOLO) — Phase 3; Phase 1 replay asserts topic presence/rates only.

> Brief non-goals above are orientation only. The contract-level deferrals (with rationale and target) are in §"Out of Scope".

## Out of Scope

> Items explicitly **not** part of this Phase 1 docset. Each has a status, rationale, and target. Listing them here (rather than omitting) prevents scope creep and gives reviewers a clear contract for what this PRD does NOT authorize.

| Item | Status | Rationale | Target | Added |
|------|--------|-----------|--------|-------|
| Containerizing Foxglove | Out of scope | It is a desktop app; the plan is explicit — "don't containerize Foxglove … let them be desktop apps" (plan §"Containerization"). This docset only verifies a bag loads in it. | N/A | 2026-06-03 |
| Postgres / production metadata store | Deferred | SQLite or DuckDB is sufficient at Phase 1 scale; "don't reach for Postgres yet" (plan M8). | Later phase if scale demands | 2026-06-03 |
| Replay regression on real detector outputs (YOLO / learned models) | Deferred | Real object detection is Phase 3; Phase 1 replay asserts topic presence/rates, not learned-model outputs (plan §"What's NOT in Phase 1", §"Test strategy"). | Phase 3 | 2026-06-03 |
| Topic pruning / bag-size optimization beyond "reasonable" | Deferred | Start broad, prune later; "reasonable" = under a few hundred MB for a 5-min mission (plan M7). | Later phase | 2026-06-03 |
| Bag schema beyond Phase 1 topics (e.g. anomaly events) | Out of scope | Those messages arrive with their phases — they aren't published yet, so there is nothing to record. | Phase 6+ | 2026-06-03 |
| Authoring the `CheckpointCapture` message / its image representation | Out of scope | Owned by 04-perception; this docset is the named *second consumer* (records the topic). The image-representation contract is consumed here, settled jointly (see OQ-2). | N/A (04 owns) | 2026-06-03 |

## Key Hypotheses

- **H1:** We believe invoking the bag-record wrapper *from the mission launch file* (rather than as a manual step) will result in every run producing a bag with no exceptions, because the recording starts and stops with the mission lifecycle the operator already triggers. *Signal: across a batch of patrol launches, the count of bags equals the count of launches; no run lacks a bag.*
- **H2:** We believe a "dumb producer, smart ingestion" split (a watch-and-sync daemon on the dev host, all indexing logic on the DGX) will keep the producer robust and the manifest the place complexity lives, because the producer then has no schema knowledge to drift. *Signal: the upload daemon needs no change when the manifest schema evolves; manifest changes stay DGX-side.*
- **H3:** We believe asserting topic presence + rates against a checked-in reference bag (not simulator behavior) will give a strict-but-non-flaky CI regression lane cheaper than SITL, because replay is deterministic and the simulator is not in the loop. *Signal: the replay test passes deterministically across CI runs and catches an intentionally-dropped topic, with runtime within the agreed budget (OQ-6).*

## Tenets

> Tie-breakers when implementation trade-offs are ambiguous — *unless you know better ones.*

1. **Every run produces a bag, no exceptions** — when in doubt, record; the discipline (plan M7) outweighs a marginally larger bag.
2. **Dumb producer, smart ingestion** — complexity belongs on the DGX/ingestion side, never in the upload daemon (plan M8); lean toward making the producer simpler.
3. **Format/schema longevity beats short-term convenience** — choose what won't force a bag-format migration two phases out (plan M6/M7), even if a quicker option exists now.
4. **Strict but not flaky** — replay assertions should fail loudly on a real regression and never fail spuriously; keep the asserted topic count small and the assertions strict (plan §"Test strategy").
5. **Start broad, prune later** — record a broad topic set first; defer size optimization until "reasonable" is actually exceeded (plan M7).

## Functional Requirements

> The FR table below is the contract for this PRD. Requirement IDs use the `LR-` (Logging/Replay) prefix. Topic and interface names are grounded in this docset's `dod.md` §5 and the sibling DoDs; anything inferred beyond the DoD is tagged `[INFERRED]`.

### P1: Critical (must ship)

#### LR-1: Automatic per-run MCAP recording
The system SHALL, when a mission is launched via `ros2 launch patrol_bringup mission_patrol.launch.py`, automatically record the agreed topic set into exactly one MCAP rosbag in a known output directory, named `patrol_<missionId>_<timestamp>.mcap`, via a Python wrapper around `ros2 bag record` that the mission launch file invokes — with no separate operator command.

**Customer scenario:** the operator runs `mission_patrol.launch.py` and, without any extra step, gets a recorded bag for that flight.

**Pain removed:** no manual `ros2 bag record` that gets forgotten — "every mission run produces a bag, no exceptions. This is the discipline" (plan M7). Nameless/per-run-missing bags can't be correlated or replayed.

**Acceptance criteria:**
- After a completed `mission_patrol.launch.py` run, exactly one MCAP rosbag exists in the known output location.
- The bag filename matches `patrol_<missionId>_<timestamp>.mcap`.
- The storage format is MCAP (the MCAP storage plugin), not sqlite3.
- Recording is triggered by the launch file include, not a manual command.

**Trace:** UAC-LR-1 (Appendix B)

#### LR-2: Bag identifiability and recorded-topic completeness
The system SHALL record a topic set sufficient for replay and inspection and carry per-bag metadata sufficient to identify the run and replay it, such that `ros2 bag info <bag>` lists the expected topics with non-zero message counts and the bag stays a reasonable size.

**Recorded topic set** (start broad, prune later — plan M7): `/fmu/out/*` (PX4 telemetry, via 01-platform's uXRCE-DDS bridge); `/patrol/*` mission + perception topics including `/patrol/checkpoint_capture`; the camera image topic (recorded as a `sensor_msgs/CompressedImage` topic — see OQ-1/OQ-2); the TF tree; and mission state / current waypoint / abort signals (`/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/abort` per 02's DoD §5).

**Per-bag metadata sidecar** carries at least: mission ID, timestamp, the recorded topic set, and correlation to the mission config — sufficient to identify and replay the run.

**Customer scenario:** a reviewer opens a months-old bag and can tell which mission config and run produced it, and confirm the topics are intact.

**Pain removed:** nameless/contextless bags that can't be correlated to a mission or replayed; missing-topic bags that look fine until replay.

**Acceptance criteria:**
- `ros2 bag info <bag>` lists the expected topics (`/fmu/out/*`, `/patrol/*`, camera image, TF, mission state/waypoint/abort) each with a non-zero message count.
- A per-bag metadata sidecar exists carrying mission ID, timestamp, topic set, and mission-config correlation.
- Bag size is reasonable: under a few hundred MB for a 5-minute mission (precise MB ceiling per OQ-9).

**Trace:** UAC-LR-2 (Appendix B)

#### LR-3: Automatic upload to DGX
The system SHALL run an upload daemon that watches the bag output directory and automatically syncs each new bag to the DGX after mission end, targeting within 30 seconds of mission end, keeping the producer "dumb" (watch + transfer only; no indexing).

**Customer scenario:** the operator finishes a mission and the bag is on the DGX without running `rsync` by hand.

**Pain removed:** field-data loss and manual copy steps — "the teams that iterate fastest are the ones who never have to re-fly to debug" (plan §"Why Phase 1 matters").

**Acceptance criteria:**
- A daemon detects a newly-completed bag in the watched directory and transfers it to the DGX automatically.
- The transfer target is within 30 s of mission end.
- The daemon performs transfer only — no indexing/parsing logic lives in the producer (complexity is on the ingestion side).

**Trace:** UAC-LR-3 (Appendix B)

#### LR-4: Manifest indexing and query
The system SHALL, on the DGX, index each uploaded bag into a manifest record — mission, time, duration, topics, and metadata-sidecar contents — stored in a SQLite or DuckDB store (not Postgres) and queryable at Phase 1 scale.

**Customer scenario:** the operator lists recent missions and their topics from the manifest instead of grepping a directory of bags.

**Pain removed:** no searchable record of what was flown and what each bag contains.

**Acceptance criteria:**
- After upload, ingestion indexes the bag and it appears in the manifest with mission, time, duration, topics, and metadata.
- The manifest is queryable (e.g., list recent runs and their topic sets) at Phase 1 scale.
- The store is SQLite or DuckDB, not Postgres.

**Trace:** UAC-LR-4 (Appendix B)

#### LR-5: Replay regression test in CI
The system SHALL provide a replay regression test that replays a checked-in reference bag via `ros2 bag play`, subscribes to the expected topics, asserts they appear at expected rates (within a defined tolerance), and runs in CI.

**Customer scenario:** a later-phase change that breaks the topic stream is caught by CI before it reaches hardware.

**Pain removed:** "this is the foundation of replay-based regression testing for later phases" (plan M8) — regressions otherwise surface on a vibrating airframe.

**Acceptance criteria:**
- The test plays a checked-in reference bag, subscribes to the expected topics, and asserts presence + rate within tolerance.
- The test runs in CI and passes deterministically.
- Assertions are on topic presence/rates (not simulator-of-a-simulator behavior).
- An intentionally-dropped/renamed topic causes the test to fail (it actually guards the contract).

**Trace:** UAC-LR-5 (Appendix B)

#### LR-6: Foxglove renders a recorded bag
The system SHALL verify that a recorded mission bag opens in Foxglove Studio and renders the camera feed, mission state, and 3D pose history with the expected panels populated. (Foxglove is an installed desktop app, not something this docset builds or containerizes.)

**Customer scenario:** the operator debugs a mission visually by scrubbing the recorded bag in Foxglove.

**Pain removed:** no visual debugger for mission outcomes.

**Acceptance criteria:**
- A recorded mission bag opens in Foxglove Studio.
- The camera feed, mission state, and 3D pose history render with the expected panels populated.
- No containerization of Foxglove is introduced.

**Trace:** UAC-LR-6 (Appendix B)

#### LR-7: CheckpointCapture topic recorded by the bag pipeline
The system SHALL record the `patrol_interfaces/msg/CheckpointCapture` topic `/patrol/checkpoint_capture` (owned/published by 04-perception) into the mission bag, demonstrating that the message is consumed by the bag pipeline as well as by the perception node — satisfying the consumer side of exit-checklist item 11. Per the settled cross-docset contract, `CheckpointCapture` carries `string image_path` (a path to a PNG/JPEG written to disk) plus header / checkpoint_id / pose / metadata — NOT full pixels by value; live frames travel on a separate `sensor_msgs/CompressedImage` topic, which is what the bag records for imagery.

**Customer scenario:** the operator replays/inspects a bag and finds the per-checkpoint capture records present alongside the compressed camera frames, with image bytes resolvable via `image_path`.

**Pain removed:** a bag that records the mission but drops the perception capture stream would break the regression baseline and the "one message, two consumers" guarantee (item 11).

**Acceptance criteria:**
- A mission run's bag contains the `/patrol/checkpoint_capture` topic with non-zero message count.
- The recorded `CheckpointCapture` carries `image_path` (not by-value pixels); the `sensor_msgs/CompressedImage` topic carries the live frames the bag records.
- The same compiled `patrol_interfaces/msg/CheckpointCapture` type published by 04 is the one recorded here (no forked definition).

**Trace:** UAC-LR-7 (Appendix B)

#### LR-8: End-to-end bag artifact (record → upload → manifest → replay → Foxglove)
The system SHALL ensure that the bag produced by a full multi-checkpoint patrol launched via `mission_patrol.launch.py` is the single artifact that satisfies LR-1 through LR-7 end-to-end: it is recorded (LR-1/2), uploaded (LR-3), indexed in the manifest (LR-4), is the basis of (or is interchangeable with) the replay test (LR-5), and renders in Foxglove (LR-6).

**Customer scenario:** the operator runs one full patrol and the resulting bag carries cleanly through the whole pipeline with no manual stitching between stages.

**Pain removed:** a pipeline whose stages each "work" in isolation but don't chain on the same real artifact (the integrative exit-checklist failure mode, item 1).

**Acceptance criteria:**
- A full-patrol run produces a bag that is recorded, uploaded, appears in the manifest, replays in CI, and renders in Foxglove — verified as one end-to-end pass.
- The stages operate on the same bag artifact (the upload daemon's output is the manifest's input; the recorded format is the format Foxglove and `ros2 bag play` consume).

**Trace:** UAC-LR-8 (Appendix B)

### P2: Important (should ship)

#### LR-9: Reference-bag provenance note
The system SHALL document the checked-in reference bag used by the replay test — which mission/config produced it, and how to regenerate it — so the regression baseline can be refreshed deliberately when topics legitimately change.

**Customer scenario:** a maintainer needs to refresh the reference bag after a deliberate topic change and follows the documented regeneration steps instead of guessing.

**Acceptance criteria:**
- A provenance note (e.g., `tests/replay/README` or sidecar) records the source mission/config and the regeneration procedure for the reference bag.
- The note states the version-control handling of the bag artifact (size / LFS — see OQ-4).

## Scope Authority

The FR table above is the **contract** for this PRD. The design document (`docs/phase1/05-logging-replay/design.md` — to be added when the design is created) realizes these FRs as components, sequences, and milestone tasks.

**The design must not introduce surface area beyond this PRD's FR table without a corresponding PRD revision.** If the design proposes a new topic to record, a new manifest field, a new transfer mechanism, or a new CI stage not authorized by an FR, the PRD must be updated first — adding the FR through the PRD's revision flow.

Conversely, **this PRD must not specify implementation detail beyond the FR shape.** The choice of manifest store (SQLite vs DuckDB), upload transport (SSH/rsync vs S3-compatible), reference-bag storage mechanics, replay-assertion strictness internals, and CI placement of upload/ingestion belong in the design — they are carried below as Open Questions, not resolved here.

This discipline keeps the design honest and the PRD lean.

## Success Metrics

> Each metric ties to one or more P1 FRs (cross-listed in the FR column) so that every P1 FR has at least one signal that would tell us it succeeded or failed. Baselines are "N/A (new)" because no logging/replay pipeline exists today.

| Metric | FR(s) signalled | Baseline (current) | Target | How Measured | Timeline |
|--------|-----------------|-------------------|--------|--------------|----------|
| Runs that produce a bag | LR-1 | N/A (new) | 100% of `mission_patrol.launch.py` runs | Count bags vs count launches over a batch | M7 exit |
| Bag size for a 5-min mission | LR-2 | N/A (new) | Under a few hundred MB (concrete ceiling per OQ-9); pass = file size is in the hundreds-of-MB range, not GB | `ros2 bag info` / file size on a canonical 5-min patrol | M7 exit |
| Expected topics present in the bag | LR-2, LR-7 | N/A (new) | All expected topics (incl. `/patrol/checkpoint_capture`) listed with non-zero message counts | `ros2 bag info` topic+count check on a recorded bag | M7 exit |
| Time from mission end to bag on DGX | LR-3 | N/A (new) | ≤ 30 s | Timestamp delta: mission end → bag present on DGX | M8 exit |
| Uploaded bag indexed + queryable in manifest | LR-4 | N/A (new) | Every uploaded bag appears in the manifest and is returned by a "list recent runs + topics" query | Run the manifest query after an upload; confirm the bag's row (mission/time/duration/topics/metadata) | M8 exit |
| Replay regression test in CI | LR-5 | N/A (new) | Passes deterministically; catches a dropped topic | CI run result + a deliberate-break check | M8 exit |
| Foxglove render of a recorded bag | LR-6 | N/A (new) | Camera + mission state + 3D pose panels populated | Manual load-and-render verification | M8 exit |
| End-to-end single-artifact pass | LR-8 | N/A (new) | One full-patrol bag carries through record → upload → manifest → replay → Foxglove with no manual stitching | One scripted/witnessed end-to-end run on a single bag artifact | M8 exit |

## Technical Considerations

### Integration points
- **01-platform:** runs inside the `sim`/`dev` containers and the `colcon` workspace; consumes `/fmu/out/*` via the uXRCE-DDS bridge.
- **02-mission-control:** the `mission_patrol.launch.py` entry-point includes/invokes this docset's bag-record wrapper; the bag records `/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/abort`.
- **04-perception:** the bag records `/patrol/checkpoint_capture` (type `patrol_interfaces/msg/CheckpointCapture`) and the separate `sensor_msgs/CompressedImage` frame topic.
- **DGX:** ingestion + manifest live on the DGX; no DGX is required for Phase 1 *dev* (plan §"Dev hardware requirements"), so CI exercise of upload/ingestion is an Open Question (OQ-7).

### Data storage
- **Bags:** MCAP files in a known output directory on the dev host (`patrol_<missionId>_<timestamp>.mcap`), synced to the DGX.
- **Per-bag metadata sidecar:** mission ID, timestamp, topic set, mission-config correlation (consumed by ingestion).
- **Manifest:** SQLite or DuckDB table on the DGX (mission, time, duration, topics, metadata). Not Postgres.
- **Reference bag:** a checked-in artifact under `tests/replay/` (storage/LFS handling per OQ-4).
- **Directories owned:** `tests/replay/`, `docker/ingest/`, `analysis/`; the bag-record wrapper is invoked from `ros2_ws/src/patrol_bringup` (package shell owned by 01, launch contents by 02).

### Rabbit holes
> Things that look simple but could explode in scope. Contain them early.

- **Recording everything raw, especially raw camera images.** Recording `sensor_msgs/Image` by value blows up bag size past "reasonable." Contain it: record `sensor_msgs/CompressedImage` for frames; `CheckpointCapture` carries `image_path`, not pixels (settled contract; OQ-1/OQ-2). Prune topics only after measuring.
- **Replay test flakiness.** Rate assertions that are too tight (or based on wall-clock) turn the CI lane flaky and erode trust. Contain it: small asserted-topic set, presence-first, tolerance bands; deliberate-break test to confirm it actually guards (OQ-5, OQ-6).
- **Reference-bag rot / repo bloat.** A large binary baseline checked into git balloons the repo or silently drifts from the real topic set. Contain it: keep it small, document regeneration, decide LFS vs in-repo deliberately (OQ-4, LR-9).
- **CI needing a real DGX.** Wiring upload + ingestion into CI could pull a DGX dependency into a phase that explicitly needs none. Contain it: decide whether upload/ingestion run in CI against a stand-in target or only locally (OQ-7).

### Potential challenges
- **Bag schema is costly to reverse** once it is the long-lived corpus + regression baseline. Mitigation: settle the topic set and the `CheckpointCapture` image representation jointly with 04 *before* the reference bag is generated (OQ-1, OQ-2); record the decision (the plan asks for ADRs on non-obvious calls).
- **MCAP storage plugin availability** in the Jazzy/24.04 container. Mitigation: the plan pins MCAP as a settled constraint; ensure the storage plugin is installed in the container base (01-platform) and verified by `ros2 bag info` reading MCAP.

## Cross-Service Impact

> Included because Scope = cross-service: this docset spans the dev host (record + upload) and the DGX (ingestion + manifest), and consumes contracts from 01/02/04. "Services" here are the Phase 1 docsets and the two hosts.

### Affected Services

| Service / docset | Impact | Changes Required |
|------------------|--------|-----------------|
| 02-mission-control | The `mission_patrol.launch.py` entry-point must include this docset's bag-record wrapper | A launch include that starts/stops the recorder with the mission lifecycle (wrapper invocation interface owned here; the include lives in 02's launch file) |
| 04-perception | The bag records `/patrol/checkpoint_capture`; relies on the settled image-representation contract | None to 04's code; this docset depends on `CheckpointCapture` carrying `image_path` and on a separate `CompressedImage` topic existing (OQ-1/OQ-2 — joint with 04) |
| 01-platform | Pipeline runs in the `sim`/`dev` containers; consumes `/fmu/out/*`; MCAP storage plugin must be present | MCAP storage plugin available in the container base; `colcon` workspace builds the wrapper/test packages |
| DGX (host) | New ingestion service + manifest store | `docker/ingest/` service indexing uploaded bags into a SQLite/DuckDB manifest |

### Interface Changes
- **Owned by this docset (others depend on):** the bag output contract (known directory + `patrol_<missionId>_<timestamp>.mcap` + MCAP + recorded-topic set); the `ros2 bag record` wrapper invocation interface; the per-bag metadata-sidecar schema; the upload-daemon → DGX transfer contract; the manifest schema; the replay regression-test contract (reference bag + assertions).
- **Consumed (settled cross-docset contract, confirmed at combined review (2026-06-03)):** `CheckpointCapture` carries `string image_path` + header/checkpoint_id/pose/metadata (NOT by-value pixels); live frames travel on a separate `sensor_msgs/CompressedImage` topic which is what the bag records (owned by 04, consumed here; per M7 "compressed image to keep bag size manageable").

### Deployment Coordination
- **Order:** the recorded-topic set and the `CheckpointCapture` image representation must be agreed (with 02/04) before the reference bag is generated, because the reference bag bakes in the topic set.
- **Independence:** the upload daemon (dumb producer) and the ingestion side can evolve independently by design (H2) — the producer has no schema knowledge.
- **Compatibility:** MCAP + the metadata sidecar schema are intended to be forward-stable into Phase 2+ ("won't be migrating bag formats two phases from now").

### Testing Implications
- **Replay regression (this docset's core test):** plays the checked-in reference bag, asserts topic presence/rates in CI (LR-5).
- **Contract dependency:** the bag must contain `/patrol/checkpoint_capture` with the same compiled `patrol_interfaces` type that 04 publishes (LR-7) — a cross-docset contract test in spirit.
- **End-to-end:** one full-patrol bag carried through record → upload → manifest → replay → Foxglove (LR-8), which is the integrative exit-checklist item 1's bag portion.

## Milestones

### Phase M7: Automatic MCAP recording (record side)
- Python `ros2 bag record` wrapper invoked automatically by `mission_patrol.launch.py` (LR-1).
- Broad recorded-topic set + per-bag metadata sidecar; MCAP format; naming convention (LR-2).
- `/patrol/checkpoint_capture` recorded with the settled image representation (LR-7).
- **Validation:** a full patrol run produces exactly one MCAP bag in the known location; `ros2 bag info` shows expected topics (including `/patrol/checkpoint_capture`) with non-zero counts; bag size under a few hundred MB for a 5-min mission.

### Phase M8: Replay pipeline (bag → DGX → Foxglove)
- Upload daemon: watch output dir, sync new bags to DGX within ~30 s (LR-3).
- DGX ingestion + queryable SQLite/DuckDB manifest (LR-4).
- Replay regression test in CI against a checked-in reference bag (LR-5); provenance note (LR-9).
- Foxglove load-and-render verification (LR-6); end-to-end single-artifact pass (LR-8).
- **Validation:** bag auto-uploads to DGX within 30 s, appears in (and is returned by a query of) the manifest, replay test passes in CI (and catches a deliberate topic break), and the bag renders in Foxglove with expected panels — with one full-patrol bag witnessed carrying through every stage (LR-8).

## Open Questions

> Structured form (status / decision target / rationale), no owner column. The cross-docset contracts below are recorded with the settled defaults applied but flagged **Provisional — confirmed at combined review (2026-06-03)**, per the auto-pilot policy (combined human review of all 5 pairs at the end). Design-target OQs are deferred to the design pass.

| # | Question | Status | Decision target | Rationale (why open / what would resolve it) |
|---|----------|--------|-----------------|----------------------------------------------|
| OQ-1 | Final recorded-topic set + compressed-vs-raw image: which exact topics are recorded, and is the live camera frame recorded as `sensor_msgs/CompressedImage`? | Provisional (default applied: broad set per LR-2; live frames as `CompressedImage`) — confirmed at combined review (2026-06-03) | PRD/Design | Plan says "start broad, prune later" and "consider compressed image to keep bag size manageable" (M7). The default keeps bag size reasonable; jointly affects 04 and bag fidelity. Confirm in combined review. |
| OQ-2 | `CheckpointCapture` image representation: by-value pixels vs `image_path` stored-path reference | Provisional (default applied: `string image_path` + separate `CompressedImage` topic the bag records) — confirmed at combined review (2026-06-03) | PRD/Design, jointly with 04 | Settled cross-docset contract default (consumed from 04; bag-size impact). Plan M6 hedges "Image, or a reference to a stored path for large images"; M7 wants compressed image. Must be confirmed jointly with 04's matching OQ. |
| OQ-3 | Manifest store: SQLite vs DuckDB | Deferred | Design | Both acceptable at this scale (plan M8); trades DuckDB analytic-query ergonomics vs SQLite ubiquity/simplicity. No PRD-level impact (LR-4 only requires "SQLite or DuckDB, not Postgres"). |
| OQ-4 | Reference-bag generation, version-control (size / LFS), and refresh policy | Deferred | Design | How the checked-in baseline is produced, stored, and deliberately regenerated when topics legitimately change, so the replay baseline doesn't rot (relates to LR-9). |
| OQ-5 | Replay assertion strictness: which topics are asserted and what rate tolerance keeps it strict-but-not-flaky | Deferred | Design | Plan §"Test strategy": "keep the count small and the assertions strict." Concrete topic subset + tolerance bands are a tuning call. |
| OQ-6 | CI runtime budget for replay | Deferred | Design | Plan explicitly welcomes pushback on "CI runtime budgets and flakiness" for ROS 2 + SITL; replay is the cheaper-than-SITL lane and its budget should be set deliberately. |
| OQ-7 | Where the upload daemon + ingestion run in CI (real stand-in target vs local-only), given no DGX is required for Phase 1 dev | Deferred | Design | Plan §"Dev hardware requirements": no DGX needed for dev. Whether DGX upload/manifest steps are exercised in CI (against a stand-in) or only locally affects LR-3/LR-4 CI coverage. |
| OQ-8 | Upload transport: SSH/rsync vs S3-compatible storage | Deferred | Design | Plan M8 lists both for the "dumb" producer; depends on DGX-side access pattern. LR-3 is transport-agnostic at the FR level. |
| OQ-9 | Precise bag-size ceiling for the LR-2 metric (the exact MB number behind "a few hundred MB") | Deferred | Design (after first canonical 5-min recording) | The plan/DoD deliberately leave this as "reasonable … under a few hundred MB" rather than a fixed number; inventing a precise ceiling now would over-constrain before the first measured recording exists. Resolve by measuring a canonical 5-min patrol and setting the ceiling from it. Until then, the metric's pass/fail is "hundreds-of-MB, not GB." |

## Appendix B: User Acceptance Criteria

> Every P1 FR has a corresponding UAC in Given/When/Then form; numbering matches the FR ID. Each maps to a falsifiable AC in this docset's `dod.md` §4.

### UAC-LR-1: Automatic per-run MCAP recording
**GIVEN** a mission launched via `ros2 launch patrol_bringup mission_patrol.launch.py`
**WHEN** the mission completes
**THEN** exactly one MCAP rosbag exists in the known output location, named `patrol_<missionId>_<timestamp>.mcap`, produced by the launch-invoked wrapper with no separate operator command. *(dod AC-1)*

### UAC-LR-2: Bag identifiability and recorded-topic completeness
**GIVEN** a produced bag
**WHEN** `ros2 bag info <bag>` is run
**THEN** it lists the expected topics (`/fmu/out/*`, `/patrol/*`, camera image, TF, mission state/waypoint/abort) each with non-zero message counts, a per-bag metadata sidecar carries mission ID / timestamp / topic set / config correlation, and the bag is under a few hundred MB for a 5-minute mission. *(dod AC-2)*

### UAC-LR-3: Automatic upload to DGX
**GIVEN** a mission has ended and the upload daemon is watching the output directory
**WHEN** the new bag appears
**THEN** the daemon uploads it to the DGX automatically, targeting within 30 s of mission end, performing transfer only (no indexing in the producer). *(dod AC-3)*

### UAC-LR-4: Manifest indexing and query
**GIVEN** a bag uploaded to the DGX
**WHEN** ingestion indexes it
**THEN** it appears in the SQLite/DuckDB manifest with mission, time, duration, topics, and metadata, and is queryable at Phase 1 scale. *(dod AC-4)*

### UAC-LR-5: Replay regression test in CI
**GIVEN** a checked-in reference bag
**WHEN** the replay regression test runs in CI
**THEN** it replays the bag via `ros2 bag play`, subscribes to the expected topics, asserts they appear at expected rates within tolerance, passes deterministically, and fails if an expected topic is dropped/renamed. *(dod AC-5)*

### UAC-LR-6: Foxglove renders a recorded bag
**GIVEN** a recorded mission bag
**WHEN** it is opened in Foxglove Studio (desktop app)
**THEN** the camera feed, mission state, and 3D pose history render with the expected panels populated, with no Foxglove containerization introduced. *(dod AC-6)*

### UAC-LR-7: CheckpointCapture topic recorded by the bag pipeline
**GIVEN** the perception node (04) publishes `patrol_interfaces/msg/CheckpointCapture` on `/patrol/checkpoint_capture`
**WHEN** a mission runs and is recorded
**THEN** the bag contains `/patrol/checkpoint_capture` with non-zero message count, the recorded message carries `image_path` (not by-value pixels) with live frames on a separate `sensor_msgs/CompressedImage` topic, and it is the same compiled `patrol_interfaces` type 04 publishes — demonstrating the bag pipeline as the named second consumer (item 11). *(dod AC-7)*

### UAC-LR-8: End-to-end bag artifact
**GIVEN** a full multi-checkpoint patrol launched via `mission_patrol.launch.py`
**WHEN** it completes
**THEN** the single bag it produced is recorded (LR-1/2), uploaded (LR-3), indexed in the manifest (LR-4), replayable by the regression test (LR-5), and renders in Foxglove (LR-6) — one artifact carried end-to-end with no manual stitching between stages. *(dod AC-8)*
