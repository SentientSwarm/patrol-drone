# Logging & Replay Pipeline — System Design Working Document

**Status:** Draft (accepted at medium floor — ReviewDesign pass 1 clean, no ≥medium findings; no ReviseDesign pass required)
**Version:** 0.1.0
**Date:** 2026-06-03
**Projects:** Patrol Drone Phase 1 (pre-hardware simulation) — Docset 05 of 5: Logging & Replay Pipeline
**Authors:** jxstanford@wemodulate.energy (solo dev / DRI)
**Location:** docs/phase1/05-logging-replay/design.md (canonical file on disk — content reproduced below unchanged)

> **Requirements source (sole):** `docs/phase1/05-logging-replay/prd.md` (Logging & Replay Pipeline PRD, rev 2). This design realizes the PRD's FR table (LR-1 … LR-9) as components, layers, interfaces, sequences, and milestones. It does not introduce surface area beyond that FR table; anything inferred is tagged `[INFERRED]` and traced to the FR it serves.

---

## Design Review Result (software-design ReviewDesign — auto-pilot)

**Reviewed:** 2026-06-03 · **Reviewer:** Claude (SoftwareDesign / ReviewDesign) · **Document version:** 0.1.0 (Draft)
**Overall Rating: Ready.** Reviewed against all 13 ReviewDesign dimensions including the D2 PRD-trace audit (reverse direction) and the D13 Verified-Preconditions grep test, with every §3.2 citation independently re-verified against the actual repo and sibling DoDs at review time. No finding at or above medium severity. Per the auto-pilot policy (revise only for ≥medium findings), no ReviseDesign pass was triggered; the design is accepted as-is on pass 1.

| # | Dimension | Rating | Key Finding |
|---|-----------|--------|-------------|
| D1 | Structure Compliance | Strong | All template sections present and well-formed; naming conventions (UAC-LR-N, INF-LN, M7/M8, T7.x/T8.x) followed; component/layer/systemic-interface sections distinct |
| D2 | Requirements Coverage + PRD-trace audit | Strong | Forward: every UAC-LR-1…8 + LR-9 + INF-L1…L3 mapped. Reverse: zero unauthorized scope — `bag_id`/`ingested_utc` marked internal, `S3Transport` marked deferred interface-stub, Foxglove layout marked operator-convenience |
| D3 | Design Question Depth + FAQ drift | Strong | Q1-Q7 substantive; Q2 CLIs/columns match §4.2.x exactly, Q5 matches §6.2, Q7 OQ statuses match §2 — zero P1 drift |
| D4 | Open Question Resolution + staleness | Strong | Every OQ has status + decision target + rationale; design-target OQs resolved; OQ-1/OQ-2 Provisional with explicit shepherding action; new deferral OQ-10 got an OQ entry |
| D5 | Security Model | Adequate | Right-sized single-operator SSH posture; IdP/mesh explicitly OOS; concern reframed as data integrity with named enforcement points (no multi-layer authz needed → Adequate, not a gap) |
| D6 | Error Handling | Strong | §4.4.5 failure-modes table covers persistent-state / network / plugin with detection + degradation + recovery; failure-path sequence shown |
| D7 | Testability | Strong | Concrete unit/integration/replay/E2E scenarios per milestone; component isolation (mockable bag reader, local stand-in) |
| D8 | UX Completeness | N/A (right-sized) | CLI/infra, no UI; operator-surface tables + Foxglove layout are the correct manifestation — fabricating wireframes would violate the minimal/right-size rule |
| D9 | Component & Layer Architecture | Strong | 8 components inventoried with boundaries; layer view derived from §3.1 zones, not invented; no layer violations |
| D10 | Systemic / Platform Interfaces | Strong | Interfaces elicited from 01 containers + ADR-0002 CI + plan stack; each with current-state/change/priority + failure mode |
| D11 | Implementation Plan Quality | Strong | True walking-skeleton: M7 skeleton (record→Foxglove) → M8 layer-1 (upload→manifest→CI-replay→E2E); crisp demo gates; OOS + Files-Touched per task |
| D12 | Inventory Triangle Consistency | Strong | Inventory ↔ diagram ↔ operator-surface enumerate the same 8 components; Transport marked internal-helper |
| D13 | Verified Preconditions | Strong | Every external claim has command + quoted result + file:line; all citations re-verified at review time against the live repo + sibling DoDs |

**Cross-consistency check:** No issues. Q1↔§4.2, Q2↔§4.2, Q3↔§4.4, Q6↔§4.2/§4.4.5, Q7↔§2, §3↔§4, §4.2↔§4.3, §4.2↔§6, AppB↔§4.1 all consistent. Only `[ ]` markers in the doc are §6.4 Definition-of-Done acceptance items (intended, not blockers); the 6 TBD instances are legitimate (Linear project/team = separate post-design step; `/patrol/*` topic types = correctly "TBD (02-owned)", not invented here).

---

## 1. Introduction

This design covers the logging-and-replay backbone for the autonomous-drone project: the infrastructure that makes every mission run automatically produce an MCAP rosbag, round-trips that bag through DGX ingestion into a queryable manifest, asserts on its contents via a replay regression test in CI, and verifies it renders in Foxglove. It is docset 05 of Phase 1 — the terminal consumer that records the topic surface owned by 01-platform (`/fmu/out/*`), 02-mission-control (`/patrol/mission_state|current_waypoint|dwell|abort`), and 04-perception (`/patrol/checkpoint_capture` + the compressed camera frames). The PRD frames the stakes plainly: the bag chosen here becomes the project's long-lived regression test, training corpus, and debugger, so the bag format (MCAP) and the metadata/manifest schema must survive into Phase 2+ without a migration.

The design decomposes into five coordinated components spanning two hosts. On the **dev host**: a `BagRecorderWrapper` (a Python wrapper around `ros2 bag record` invoked from 02's mission launch file) and an `UploadDaemon` (a dumb watch-and-sync producer). On the **DGX**: an `IngestService` that indexes uploaded bags into a `ManifestStore` (SQLite). In the **repo/CI**: a `ReplayRegressionTest` that plays a checked-in reference bag and asserts topic presence/rates. Foxglove Studio is an installed desktop app — the design verifies a bag loads in it but builds nothing inside it and does not containerize it.

The guiding architectural split, inherited from the PRD's tenets, is **"dumb producer, smart ingestion"**: the recorder and the upload daemon carry no schema knowledge (so they never drift), and all indexing/query complexity lives DGX-side where it can evolve independently. The design is deliberately right-sized for Phase 1 simulation — SQLite not Postgres, rsync/SSH not a bespoke transfer service, presence/rate assertions not learned-model regression. It is infrastructure, not a user-facing product; the "operator" is the solo dev and future maintainers.

### Source Projects (Linear)

| # | Project | Est. | Wave |
|---|---------|------|------|
| 1 | Patrol Drone Phase 1 — 05 Logging & Replay (TBD — Linear bootstrap is a separate post-design step) | — | M7–M8 |

### Related Projects

| Project | Relevance |
|---------|-----------|
| 01-platform (`docs/phase1/01-platform/dod.md`) | Provides the `sim`/`dev` containers, the `ros2_ws` colcon workspace, the `/fmu/out/*` telemetry surface, the MCAP storage plugin in the container base, and the empty `docker/ingest/` container slot. |
| 02-mission-control (`docs/phase1/02-mission-control/dod.md`) | Owns `mission_patrol.launch.py` (which includes this docset's recorder) and `/patrol/{mission_state,current_waypoint,abort}` (recorded topics). |
| 04-perception (`docs/phase1/04-perception/dod.md`) | Owns `patrol_interfaces/msg/CheckpointCapture` + `/patrol/checkpoint_capture` + the camera frame topic — recorded by this pipeline; supplies the consumed image-representation contract. |
| 03-sim-environment (`docs/phase1/03-sim-environment/dod.md`) | Owns the RGB camera topic that becomes the recorded `sensor_msgs/CompressedImage` frame source (surfaced via 02/04 at record time). |

### Reformulation Summary

No structural reformulation of the PRD was required. The PRD's FR table (LR-1 … LR-9) maps cleanly onto five components plus a documentation artifact; the milestone split (M7 record side, M8 replay pipeline) is preserved as the walking-skeleton's skeleton-then-layers ordering.

---

## 2. Open Questions & Assumptions

Carried forward from the PRD's Open Questions table. The two cross-docset contracts (OQ-1, OQ-2) are recorded **Provisional — confirmed at combined review (2026-06-03)** with the settled defaults applied (per the auto-pilot policy; the human does a combined review of all 5 docset pairs at the end). Design-target OQs are resolved below where the design can settle them, or kept Deferred with a decision target where they need a first measured recording / a build-time decision.

| # | Item | Source | Status | Decision target | Resolution / rationale |
|---|------|--------|--------|-----------------|------------------------|
| OQ-1 | Final recorded-topic set + compressed-vs-raw image (live frame as `sensor_msgs/CompressedImage`?) | PRD OQ-1 | **Provisional — confirmed at combined review (2026-06-03)** (default applied) | PRD/Design | Default applied: broad recorded set per §4.2.1 `recorded_topics` (LR-2); live camera frame recorded as `sensor_msgs/CompressedImage`. Jointly affects 04. Confirm in combined review before the reference bag is generated. |
| OQ-2 | `CheckpointCapture` image representation: by-value pixels vs `image_path` reference | PRD OQ-2 (joint with 04) | **Provisional — confirmed at combined review (2026-06-03)** (default applied) | PRD/Design (joint w/ 04) | Default applied (settled cross-docset contract): `CheckpointCapture` carries `string image_path`; live frames travel on a separate `sensor_msgs/CompressedImage` topic which is what the bag records for imagery. Must match 04's matching OQ at combined review. |
| OQ-3 | Manifest store: SQLite vs DuckDB | PRD OQ-3 | **Resolved (this design)** | Design | **Resolved: SQLite** for the manifest store (§4.2.4). Rationale: ubiquity + zero-dependency stdlib `sqlite3` in Python 3.12, single-file portability, adequate query ergonomics at Phase 1 scale (tens–hundreds of bags). DuckDB's analytic edge is not needed until corpus-scale analytics (a later phase); the `ManifestStore` interface is kept store-agnostic so a DuckDB swap is a one-component change. LR-4 only requires "SQLite or DuckDB, not Postgres." |
| OQ-4 | Reference-bag generation, version-control (size / LFS), refresh policy | PRD OQ-4 | **Resolved (this design)** | Design | **Resolved: Git LFS** for the checked-in reference bag, kept deliberately small (a trimmed ~30–60 s slice, not a full 5-min patrol), with a provenance note (LR-9, §4.2.6). Rationale: a multi-MB MCAP binary bloats normal git history; LFS keeps the working tree light and the artifact deliberately regenerated. Regeneration procedure documented in `tests/replay/README.md`. |
| OQ-5 | Replay assertion strictness: which topics asserted, what rate tolerance | PRD OQ-5 | **Resolved (this design)** | Design | **Resolved (§4.2.5):** assert a small fixed subset — `/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/checkpoint_capture`, the camera `CompressedImage` topic, and one `/fmu/out/*` topic (`vehicle_local_position`). Presence-first (count > 0), then mean-rate within a ±40% tolerance band measured over the replayed window. Keeps the count small and strict (PRD tenet 4). |
| OQ-6 | CI runtime budget for replay | PRD OQ-6 | **Resolved (this design)** | Design | **Resolved: ≤ 90 s wall-clock** for the replay test in CI (a trimmed reference bag plays in real-or-faster time + assertion overhead). It is the cheaper-than-SITL lane (PRD H3); a soft CI gate, revisited only if a longer reference bag is later required. |
| OQ-7 | Where upload daemon + ingestion run in CI (real stand-in vs local-only) | PRD OQ-7 | **Resolved (this design)** | Design | **Resolved: local stand-in target in CI; no real DGX dependency** (§4.4.2). The `UploadDaemon` targets a configurable destination; CI points it at a local directory / loopback SSH ("DGX stand-in") so the upload→ingest→manifest chain is exercised without a DGX (no DGX is required for Phase 1 dev). The replay test (LR-5) is the per-PR gate; the upload/ingest integration test runs against the stand-in. |
| OQ-8 | Upload transport: SSH/rsync vs S3-compatible storage | PRD OQ-8 | **Resolved (this design)** | Design | **Resolved: SSH/rsync (`rsync` over SSH) for Phase 1**, behind a `Transport` abstraction (§4.2.3a) so an S3-compatible backend is a drop-in alternative. Rationale: rsync is dumb, resumable, dependency-light, and matches the dev-host→DGX access pattern; S3 is deferred until a managed object store is actually in play. LR-3 is transport-agnostic at the FR level. |
| OQ-9 | Precise bag-size ceiling for the LR-2 metric | PRD OQ-9 | **Deferred** | Design (after first canonical 5-min recording) | Deliberately left until a canonical 5-min patrol is recorded and measured; inventing a precise MB number now over-constrains. Until measured, pass/fail is "hundreds-of-MB, not GB" (§4.2.1). Decision target: first M7 canonical recording. |
| OQ-10 | `[INFERRED]` Metadata-sidecar serialization format (JSON vs YAML) | This design | **Resolved (this design)** | Design | **Resolved: JSON sidecar** (`<bag>.meta.json`) alongside the `.mcap` (§4.2.2). Rationale: stdlib `json` round-trips losslessly, is trivially queryable by `IngestService`, and avoids a YAML dependency in the dumb producer. Serves LR-2's "per-bag metadata sidecar" requirement; no new FR surface (it is the realization of LR-2's sidecar, not a new contract). |

> **Reviewer note (deferred, sub-medium):** §3.3 attributes the camera-frame topic to "03 (via 04)" while the PRD integration section attributes the `CompressedImage` topic to 04. This is a benign attribution nuance fully inside OQ-1/OQ-2's pending-confirmation scope (04 owns the topic shape/publication; 03 owns the underlying RGB sensor in sim) — it changes no FR, component, or contract, so it is carried as a deferred note rather than triggering a revise pass. Confirm the exact owner at combined review when OQ-1/OQ-2 are confirmed.

---

## 3. Existing Foundation

This is a **greenfield infrastructure** docset landing in a repo skeleton that 01-platform provisions. The "existing foundation" is therefore (a) the empty package/directory slots already committed, (b) the pinned stack + container base from 01, and (c) the consumed topic surface from 01/02/04. There is no prior logging/replay code to extend.

### 3.1 Repo + Host Architecture (two hosts, four zones)

```
            DEV HOST (laptop / workstation — no DGX required for Phase 1 dev)
  ┌──────────────────────────────────────────────────────────────────────┐
  │  sim / dev containers (01-platform)  ── ros2_ws colcon workspace      │
  │                                                                        │
  │   ros2 launch patrol_bringup mission_patrol.launch.py  (02)           │
  │        │ include                                                       │
  │        ▼                                                               │
  │   [BagRecorderWrapper]  ──writes──►  bag output dir + .meta.json       │
  │                                          │ (watched)                   │
  │                                          ▼                             │
  │                                   [UploadDaemon] ──rsync/SSH──┐        │
  └──────────────────────────────────────────────────────────────┼───────┘
                                                                   │
            DGX HOST (or local stand-in in CI)                     ▼
  ┌──────────────────────────────────────────────────────────────────────┐
  │  docker/ingest/ container                                              │
  │   [IngestService] ──indexes──► [ManifestStore (SQLite)]  ◄── query     │
  └──────────────────────────────────────────────────────────────────────┘

            REPO / CI                          DESKTOP (operator)
  ┌─────────────────────────────┐     ┌──────────────────────────────────┐
  │ tests/replay/               │     │  Foxglove Studio (installed app,  │
  │  [ReplayRegressionTest]     │     │   NOT built/containerized here)   │
  │  + reference bag (Git LFS)  │     │   opens recorded .mcap natively   │
  └─────────────────────────────┘     └──────────────────────────────────┘
```

| Zone | Owns | Persistence | Logging/Replay Awareness (current state) |
|------|------|-------------|------------------------------------------|
| **Dev host (record)** | `BagRecorderWrapper`, `UploadDaemon` | Local bag output dir + `.meta.json` sidecars | **None today** — slots empty; recorder + daemon are new in this design |
| **DGX host (ingest)** | `IngestService`, `ManifestStore` | SQLite manifest file on DGX | **None today** — `docker/ingest/` is a `.gitkeep`-only empty container slot (verified §3.2) |
| **Repo / CI** | `ReplayRegressionTest`, reference bag, provenance note | Git LFS (reference bag), CI artifacts | **None today** — `tests/replay/` is `.gitkeep`-only (verified §3.2) |
| **Desktop** | Foxglove load-and-render verification | n/a (read-only viewer) | Foxglove is a settled installed desktop app (01 + plan §"Containerization") |

### 3.2 Verified Preconditions

Each row is a claim this design depends on, verified against the actual repo / sibling DoDs at design time **and re-verified at ReviewDesign time**. The Result column quotes the verified shape; the Citation column points to the proving location.

| Claim | Verification | Result | Citation |
|-------|--------------|--------|----------|
| `docker/ingest/` exists as an empty container slot this docset fills (not already implemented) | `ls -la docker/ingest` | `.gitkeep` only — empty directory | `docker/ingest/.gitkeep` (repo root) |
| `tests/replay/` exists as the slot for the replay test + reference bag | `ls -la tests/replay` | `.gitkeep` only — empty directory | `tests/replay/.gitkeep` |
| `analysis/` exists as the slot for bag analysis tooling | `ls -la analysis` | `.gitkeep` + `README.md` only | `analysis/README.md`, `analysis/.gitkeep` |
| 01-platform commits to making the **MCAP storage plugin** available in the container base (not this docset's job to install) | Read 01 DoD AC-8 / capability 6 | "Pinned stack manifest … pins every layer (OS, ROS 2 Jazzy, PX4 v1.16.x, … MCAP plugin, colcon, Docker)" | `docs/phase1/01-platform/dod.md` line 53 (capability 6) |
| 04-perception **owns** `patrol_interfaces/msg/CheckpointCapture` with an image-or-path field; the image representation is a joint-open decision (consumed here, not authored here) | Read 04 DoD AC-3 + §5 + §7 | "`patrol_interfaces/msg/CheckpointCapture` … image field (sensor_msgs/Image OR a stored-path string — see §7) … by-value-vs-path image decision is open … Must be settled jointly with 05" | `docs/phase1/04-perception/dod.md` lines 52, 60, 84 |
| 02-mission-control **owns** `mission_patrol.launch.py` (which includes this recorder) and the mission topics `/patrol/{mission_state,current_waypoint,abort}` | Read 02 DoD §2 + §5 | "`mission_patrol.launch.py` (the Phase-1 exit command)" + "`/patrol/mission_state`, `/patrol/current_waypoint`, `/patrol/abort` … 05-logging-replay records these" | `docs/phase1/02-mission-control/dod.md` lines 22, 59–61 |
| 01-platform owns the `/fmu/out/*` telemetry surface this pipeline records | Read 01 DoD §5 | "native `/fmu/out/*` topics (notably `/fmu/out/vehicle_local_position`) … consumed by … 05 (recorded topics)" | `docs/phase1/01-platform/dod.md` line 75 |
| Python 3.12 + colcon workspace are the runtime for the wrapper/daemon/ingestion | Read plan Target stack + `pyproject.toml` | `requires-python = ">=3.12"` | `pyproject.toml:5`; `docs/phase1_simulation_plan.md` (Target stack) |
| **No DGX is required for Phase 1 dev** (constrains CI exercise of upload/ingest) | Read plan §"Dev hardware requirements" | "**A DGX is not required for Phase 1.**" | `docs/phase1_simulation_plan.md` line 355 |
| CI is a two-layer architecture (pure-Python per-PR + slow SITL/integration tier) the replay test plugs into | Read ADR-0002 ref in 02 DoD | "mission-core unit tests … per-PR on a pure-Python runner; SITL integration is the slow/flaky tier … (ADR-0002)" | `docs/decisions/0002-ci-architecture.md`; `docs/phase1/02-mission-control/dod.md` line 78 |

### 3.3 Consumed Topic Surface (the record contract's input)

The recorder records these; their **shapes are owned elsewhere** and this design only consumes them (it never forks a definition).

| Topic / pattern | Type | Owner | Recorded for |
|-----------------|------|-------|--------------|
| `/fmu/out/*` (incl. `vehicle_local_position`) | `px4_msgs/*` | 01-platform | Telemetry, 3D pose history (LR-2, LR-6) |
| `/patrol/mission_state` | `std_msgs/String` (resolved in 02 design OQ-3) | 02 | Mission state panel (LR-2, LR-6) |
| `/patrol/current_waypoint` | `std_msgs/Int32` (resolved in 02 design OQ-3) | 02 | Waypoint trace (LR-2) |
| `/patrol/dwell` | `std_msgs/Int32` (02 design OQ-7, atomic capture event added 2026-06-21 per PR #8) | 02 | Per-checkpoint capture-trigger record (LR-2) |
| `/patrol/abort` | `std_msgs/Bool` (resolved in 02 design OQ-3) | 02 | Abort signal record (LR-2) |
| `/patrol/checkpoint_capture` | `patrol_interfaces/msg/CheckpointCapture` (carries `image_path`) | 04 | Per-checkpoint capture record (LR-2, LR-7) |
| `/drone/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | 03 (owns the camera; 04 subscribes to live `/drone/camera/image_raw`) | Live imagery the bag records (LR-2, LR-6, LR-7) |
| TF tree (`/tf`, `/tf_static`) | `tf2_msgs/TFMessage` | 01/PX4 | 3D pose history in Foxglove (LR-2, LR-6) |

### 3.4 Architectural Decision: Dumb Producer / Smart Ingestion

**Decision:** The dev-host components (`BagRecorderWrapper`, `UploadDaemon`) carry **zero manifest/schema knowledge**. They record a configured topic list, write a flat JSON sidecar, and byte-copy the bag to the DGX. All parsing, indexing, duration/topic extraction, and query logic live in `IngestService` + `ManifestStore` on the DGX.
**Rationale:** PRD tenet 2 + hypothesis H2 — the producer then has no schema to drift; the manifest can evolve DGX-side without touching the producer.
**Implication:** `IngestService` re-derives topic/duration facts from the bag itself (`ros2 bag info` / the MCAP file), treating the sidecar as identity/correlation metadata only — so a sidecar bug can never silently corrupt the indexed topic facts.

### 3.5 Codebase Snapshot

| Repository | Branch | Commit | Date | Relevant Paths |
|-----------|--------|--------|------|---------------|
| `patrol-drone` | `main` | `8d03170` | 2026-06-03 | `docker/ingest/`, `tests/replay/`, `analysis/`, `ros2_ws/src/` (empty), `docs/phase1/05-logging-replay/`, `pyproject.toml`, `docs/decisions/0002` |

---

## 4. Detailed Design

### 4.1 UC Traceability Matrix

Every P1 UAC (UAC-LR-1 … UAC-LR-8) and the P2 FR (LR-9) from the PRD's Appendix B maps to at least one component.

| Design Component | Covers FRs / UACs | Milestone |
|-----------------|-------------------|-----------|
| **BagRecorderWrapper** (§4.2.2) | LR-1 (UAC-LR-1), LR-2 (UAC-LR-2), LR-7 (UAC-LR-7) | M7 |
| **UploadDaemon** + **Transport** (§4.2.3) | LR-3 (UAC-LR-3) | M8 |
| **IngestService** + **ManifestStore** (§4.2.4) | LR-4 (UAC-LR-4) | M8 |
| **ReplayRegressionTest** (§4.2.5) | LR-5 (UAC-LR-5) | M8 |
| **FoxgloveVerification** (§4.2.6) | LR-6 (UAC-LR-6) | M7/M8 |
| **ReferenceBag + ProvenanceNote** (§4.2.6) | LR-9, LR-5 (UAC-LR-5) | M8 |
| **End-to-end artifact flow** (§4.5 Sequence 4) | LR-8 (UAC-LR-8) | M8 |

*Coverage check:* UAC-LR-1…8 each appear; LR-9 (P2) appears. No UAC is unmapped. Inferred requirements INF-L1…L3 (Appendix B) are covered by §4.2.2 (sidecar), §3.4 (integrity invariant), and §4.2.6 (reference bag).

### 4.2 Component Architecture

#### 4.2.1 Component Inventory

| Component | Type | Boundary (in / out of scope) | Responsibility | Dependencies |
|-----------|------|------------------------------|----------------|--------------|
| **BagRecorderWrapper** | module (Python, in `ros2_ws`) | IN: wrap `ros2 bag record`, pick topics, name bag, write sidecar. OUT: the launch file (02 owns the include), the topic shapes (01/02/04 own). | Record the configured topic set into one MCAP bag named `patrol_<missionId>_<timestamp>.mcap`; emit `<bag>.meta.json` | `ros2 bag record` + MCAP storage plugin (01); `recorded_topics` config |
| **UploadDaemon** | service (Python daemon, dev host) | IN: watch output dir, transfer new bags + sidecars. OUT: any indexing/parsing (that's DGX-side). | Detect a completed bag and byte-copy it (bag + sidecar) to the DGX target within ~30 s, dumbly | `Transport`; watched dir path; bag-complete marker |
| **Transport** | module (Python, internal to UploadDaemon) | IN: a `send(local, remote)` contract. OUT: knowing what a bag *is*. | Pluggable transfer mechanism (rsync-over-SSH default; S3 alt stub) | rsync/ssh binaries (or S3 client) |
| **IngestService** | service (Python, `docker/ingest/` container, DGX) | IN: detect uploaded bag, derive facts, upsert manifest row. OUT: producing bags; recording. | Index each uploaded bag (mission, time, duration, topics, sidecar metadata) into the manifest | `ros2 bag info`/MCAP reader; `ManifestStore`; uploaded bag + sidecar |
| **ManifestStore** | module + data store (SQLite file, DGX) | IN: upsert + query manifest rows. OUT: ingestion logic. | Persist + serve the queryable manifest; store-agnostic interface | `sqlite3` (stdlib); SQLite file path |
| **ReplayRegressionTest** | test (pytest, `tests/replay/`) | IN: play reference bag, assert presence+rate. OUT: simulator behavior (not mocked); learned-model outputs (Phase 3). | Replay a checked-in reference bag via `ros2 bag play`, assert expected topics at expected rates, run in CI | `ros2 bag play`; ReferenceBag; assertion config |
| **ReferenceBag + ProvenanceNote** | data artifact + doc (`tests/replay/`, Git LFS) | IN: a small checked-in MCAP + regeneration note. OUT: being a full mission bag (kept trimmed). | The regression baseline the replay test asserts against, with documented provenance/regeneration | BagRecorderWrapper output (its source); Git LFS |
| **FoxgloveVerification** | external + procedure | IN: a documented load-and-render check. OUT: building/containerizing Foxglove (it's a desktop app). | Verify a recorded bag opens in Foxglove with camera/mission-state/3D-pose panels populated | Foxglove Studio (installed desktop app); a recorded bag |

#### 4.2.2a Component Dependency Diagram

```
                          recorded_topics (config)
                                   │
  mission_patrol.launch.py (02) ── include ──► [BagRecorderWrapper]
                                                     │ writes
                                          bag .mcap + <bag>.meta.json
                                                     │ (watched dir)
                                                     ▼
                                              [UploadDaemon] ──uses──► [Transport]
                                                     │ transfers (rsync/SSH)
                                                     ▼
                                              [IngestService] ──derives facts──► (ros2 bag info)
                                                     │ upsert
                                                     ▼
                                              [ManifestStore] ◄── query (list recent runs + topics)

  [BagRecorderWrapper] ──(trimmed slice, regenerate)──► [ReferenceBag + ProvenanceNote]
                                                              │ played by
                                                              ▼
                                                     [ReplayRegressionTest] (CI)

  (any recorded .mcap) ──opened in──► [FoxgloveVerification] (Foxglove desktop app)
```

*Inventory triangle check:* every inventory row (§4.2.1) appears as a node above and has a per-component spec (§4.2.2–§4.2.6). `Transport` is an internal helper of `UploadDaemon` (no standalone consumer surface) — surfaced as a sub-spec under §4.2.3a. Consumer-facing manifestation for this infrastructure is the operator surface (the launch include + the `manifest_query` CLI + the `pytest tests/replay` entry point), enumerated per component below — there is no SDK.

---

#### 4.2.2 BagRecorderWrapper

**Type:** module (Python, lands in `ros2_ws/src/patrol_logging/` — `[INFERRED]` package name — a new package in the colcon workspace; 01 owns the workspace, this docset owns this package's contents)
**Boundary:** Owns: topic-set selection, MCAP storage-plugin selection, bag naming, sidecar emission, start/stop with mission lifecycle. Delegates: the launch *include* (02's `mission_patrol.launch.py` calls it), all topic *shapes* (01/02/04).
**Location:** `ros2_ws/src/patrol_logging/patrol_logging/recorder.py` (new); launch fragment exposed as an includable `record.launch.py` (new) that 02's launch file includes.
**Dependencies:** `ros2 bag record` CLI + MCAP storage plugin (01); `recorded_topics.yaml` config.

**Consumer/operator surface:**

| Surface | Form | Notes |
|---------|------|-------|
| Launch include | `record.launch.py` (included by 02's `mission_patrol.launch.py`) | Starts the recorder at mission start, stops at mission end — no separate operator command (LR-1) |
| Config | `ros2_ws/src/patrol_logging/config/recorded_topics.yaml` | The `recorded_topics` list (broad set; OQ-1 default) |
| Output | `patrol_<missionId>_<timestamp>.mcap` + `<bag>.meta.json` in the known output dir | Naming + MCAP format are the owned bag-output contract |

Recorder logic:

```python
class BagRecorderWrapper:
    def start(self, mission_id: str, output_dir: Path, topics: list[str]) -> None:
        """
        Guards: MCAP storage plugin available; output_dir writable; topics non-empty.
        Effect: spawns `ros2 bag record --storage mcap -o patrol_<mission_id>_<ts> <topics>`.
        Side effects: begins writing exactly one MCAP bag; records start time for the sidecar.
        """

    def stop(self) -> BagResult:
        """
        Effect: stops the recorder cleanly (SIGINT to the record process) so the MCAP finalizes.
        Side effects: writes <bag>.meta.json sidecar (mission_id, timestamps, recorded topic set,
                      mission-config correlation); returns the bag path for downstream watch.
        """
```

Metadata sidecar (`<bag>.meta.json`, JSON per OQ-10):

```python
@dataclass
class BagSidecar:
    mission_id: str            # correlates bag → mission config (LR-2)
    bag_filename: str          # patrol_<missionId>_<timestamp>.mcap
    started_utc: str           # ISO-8601
    ended_utc: str             # ISO-8601
    recorded_topics: list[str] # the topic set requested at record time
    mission_config_ref: str    # path/ref to the mission YAML that produced this run (LR-2 correlation)
    # NOTE: duration/topic-message-counts are NOT trusted from here — IngestService re-derives them
    #       from the bag itself (§3.4 dumb-producer invariant).
```

*Traces to: LR-1 (UAC-LR-1), LR-2 (UAC-LR-2), LR-7 (UAC-LR-7).*

> **PRD-trace / scope note:** the `recorded_topics` list realizes LR-2's "record a topic set sufficient for replay"; it adds no topic beyond the LR-2 enumeration. `/patrol/checkpoint_capture` is recorded because LR-7 names it. The sidecar realizes LR-2's "per-bag metadata sidecar." No new contract beyond the FR table.

---

#### 4.2.3 UploadDaemon

**Type:** service (Python daemon, dev host)
**Boundary:** Owns: watching the output dir, detecting a *completed* bag, invoking `Transport.send` for the bag + its sidecar, retry-on-failure. Delegates: any indexing/parsing (DGX-side, §3.4); the transfer mechanism itself (to `Transport`).
**Location:** `analysis/upload_daemon/` — `[INFERRED]` directory (a dev-host Python daemon; lives under `analysis/` per the PRD's directory ownership) — entry point `upload_daemon.py` (new).
**Dependencies:** `Transport` (§4.2.3a); watched-dir path; bag-complete marker (the recorder's clean stop + sidecar presence is the "bag complete" marker).

**Operator surface:**

| Surface | Form | Notes |
|---------|------|-------|
| Daemon process | `python -m upload_daemon --watch <dir> --target <dest>` | Long-running; dumb watch+transfer only (LR-3) |
| Config | `--target` (rsync/SSH dest or local stand-in dir), `--transport rsync\|s3` | Target is configurable so CI uses a local stand-in (OQ-7) |

Daemon logic:

```python
class UploadDaemon:
    def on_bag_complete(self, bag_path: Path) -> None:
        """
        Guards: a finalized .mcap AND its <bag>.meta.json both present (atomic "complete" marker).
        Effect: Transport.send(bag) then Transport.send(sidecar) to the configured target.
        Side effects: NONE on manifest/index — transfer only (dumb producer, PRD tenet 2).
        Recovery: on transfer failure, retry with backoff; the bag stays on disk until confirmed.
        Target: completes within 30 s of mission end (LR-3) for a reasonable-size bag.
        """
```

##### 4.2.3a Transport (internal helper of UploadDaemon)

**Type:** module. **Boundary:** a single `send(local_path, remote_path) -> bool` contract; knows nothing about bags.
**Location:** `analysis/upload_daemon/transport.py` (new).

```python
class Transport(Protocol):
    def send(self, local_path: Path, remote_path: str) -> bool: ...

class RsyncSshTransport(Transport):   # Phase 1 default (OQ-8)
    """rsync -a over SSH; resumable, dependency-light."""

class S3Transport(Transport):         # deferred alternative (OQ-8) — interface parity only
    """[INFERRED] drop-in for an S3-compatible store; NOT implemented in Phase 1."""
```

*Traces to: LR-3 (UAC-LR-3).*

> **PRD-trace / scope note:** `S3Transport` is an interface stub for OQ-8 parity, **not implemented in M8** — it adds no behavior beyond LR-3's transport-agnostic requirement and is marked deferred. Only `RsyncSshTransport` ships.

---

#### 4.2.4 IngestService + ManifestStore

**Type:** service + data store (Python in `docker/ingest/` container, SQLite file — DGX or CI stand-in)
**Boundary (IngestService):** Owns: detecting an uploaded bag, deriving authoritative topic/duration facts *from the bag*, reading the sidecar for identity/correlation, upserting one manifest row. Delegates: persistence + query (to `ManifestStore`); transfer (UploadDaemon did it).
**Boundary (ManifestStore):** Owns: schema, upsert, query. Delegates: deriving facts (IngestService does that).
**Location:** `docker/ingest/ingest_service.py`, `docker/ingest/manifest_store.py`, `docker/ingest/manifest_query.py`, `docker/ingest/Dockerfile` (all new — fills the empty `docker/ingest/` slot, verified §3.2).
**Dependencies:** an MCAP/`ros2 bag info` reader; `sqlite3` (stdlib); uploaded bag + sidecar.

**Operator surface:**

| Surface | Form | Notes |
|---------|------|-------|
| Ingest trigger | watch the DGX landing dir (mirror of the producer pattern) | Indexes each newly-arrived bag (LR-4) |
| Manifest query | `python -m manifest_query --recent N` (and `--mission <id>`) | "list recent runs + their topic sets" (LR-4) |

ManifestStore schema (SQLite — OQ-3 resolved):

```python
# Table: bag_manifest  (manifest_store.py)
#   bag_id         TEXT PRIMARY KEY   -- bag filename (patrol_<missionId>_<timestamp>.mcap)
#   mission_id     TEXT NOT NULL      -- from sidecar (LR-4 "mission")
#   recorded_utc   TEXT NOT NULL      -- start time (LR-4 "time")
#   duration_s     REAL NOT NULL      -- DERIVED from the bag, not the sidecar (§3.4)
#   topics_json    TEXT NOT NULL      -- DERIVED topic list + per-topic msg counts (LR-4 "topics")
#   metadata_json  TEXT NOT NULL      -- the sidecar contents (LR-4 "metadata")
#   ingested_utc   TEXT NOT NULL      -- internal bookkeeping (when ingestion ran)
```

IngestService logic:

```python
class IngestService:
    def index(self, bag_path: Path, sidecar_path: Path) -> None:
        """
        Guards: bag readable as MCAP; sidecar parses as JSON.
        Effect: derive duration_s + topics(+counts) from the bag itself; read mission_id/recorded_utc
                from the sidecar; ManifestStore.upsert(...) one row.
        Idempotent: re-indexing the same bag_id updates in place (re-upload safe).
        """
```

*Traces to: LR-4 (UAC-LR-4).*

> **PRD-trace / scope note:** the manifest columns map 1:1 to LR-4's required fields (mission, time, duration, topics, metadata). `ingested_utc`/`bag_id` are internal bookkeeping (marked internal), not new external contract. Store is SQLite, not Postgres (LR-4).

---

#### 4.2.5 ReplayRegressionTest

**Type:** test (pytest, `tests/replay/`)
**Boundary:** Owns: playing the reference bag, subscribing to the asserted topic subset, asserting presence + rate, running in CI, the deliberate-break self-check. Delegates: producing the reference bag (BagRecorderWrapper); the reference bag's storage (Git LFS).
**Location:** `tests/replay/test_replay_regression.py`, `tests/replay/assertions.yaml` (new — fills the empty `tests/replay/` slot, verified §3.2).
**Dependencies:** `ros2 bag play`; the ReferenceBag (§4.2.6); the asserted-topic config.

Asserted subset + tolerance (OQ-5 resolved):

| Asserted topic | Check | Why this one |
|----------------|-------|--------------|
| `/patrol/mission_state` | count > 0; mean rate within ±40% | Mission liveness |
| `/patrol/current_waypoint` | count > 0 | Mission progression |
| `/patrol/checkpoint_capture` | count > 0 | LR-7 contract (capture stream recorded) |
| camera `CompressedImage` topic | count > 0; mean rate within ±40% | Imagery present (LR-6/LR-7) |
| one `/fmu/out/*` (`vehicle_local_position`) | count > 0; mean rate within ±40% | Telemetry liveness |

Test logic:

```python
def test_replay_topics_present_and_rated(reference_bag, assertions):
    """
    GIVEN the checked-in reference bag
    WHEN replayed via `ros2 bag play` while subscribers count messages over the window
    THEN every asserted topic has count > 0 AND (for rated topics) mean rate within ±40%.
    Deterministic: plays a fixed bag (not the simulator) — PRD H3.
    Budget: ≤ 90 s wall-clock in CI (OQ-6).
    """

def test_dropped_topic_fails(reference_bag_missing_one_topic):
    """
    GIVEN a reference bag with one asserted topic removed
    WHEN the same assertions run
    THEN the test FAILS — proving the guard actually guards (LR-5 deliberate-break AC).
    """
```

*Traces to: LR-5 (UAC-LR-5).*

> **PRD-trace / scope note:** the asserted subset is "small and strict" per PRD tenet 4; assertions are presence/rate only (not learned-model outputs — that's Phase 3, PRD non-goal). No assertion on simulator behavior.

---

#### 4.2.6 ReferenceBag + ProvenanceNote, and FoxgloveVerification

**ReferenceBag + ProvenanceNote**
**Type:** data artifact (Git LFS) + doc.
**Boundary:** Owns: a small trimmed reference MCAP + a regeneration/provenance note. Delegates: bag production (BagRecorderWrapper).
**Location:** `tests/replay/reference/patrol_reference.mcap` (Git LFS), `tests/replay/README.md` (provenance note).
**Dependencies:** Git LFS; a recorded bag to trim from.

| Provenance field (LR-9) | Content |
|-------------------------|---------|
| Source mission/config | which `mission_*.yaml` + run produced the slice |
| Regeneration procedure | exact commands to re-record + trim to the ~30–60 s slice |
| VC handling | Git LFS (OQ-4); kept small so the repo stays light |

*Traces to: LR-9, LR-5 (UAC-LR-5).*

**FoxgloveVerification**
**Type:** external (desktop app) + manual/scripted procedure.
**Boundary:** Owns: a documented load-and-render check (camera feed, mission state, 3D pose history panels populated). Delegates: everything inside Foxglove (it's an installed app — not built, not containerized).
**Location:** `analysis/foxglove/README.md` + a saved Foxglove layout `analysis/foxglove/patrol_layout.json` — `[INFERRED]` (a layout file is operator convenience; it adds no build surface).
**Dependencies:** Foxglove Studio (installed desktop app); a recorded `.mcap`.

*Traces to: LR-6 (UAC-LR-6).*

> **PRD-trace / scope note:** No Foxglove containerization is introduced (PRD Out-of-Scope, LR-6 AC). The saved layout is operator convenience, not a built artifact.

---

### 4.3 Layer View

This pipeline is not a UI/API/DB web stack; its "layers" are the **data-flow stages** of a bag's lifecycle, which is the natural architecture for this infrastructure (derived from §3.1's four zones, not invented).

#### 4.3.1 Layer Mapping

| Layer (lifecycle stage) | Components | Key Responsibilities |
|-------------------------|-----------|----------------------|
| **Record (dev host)** | BagRecorderWrapper | Turn a mission run into one identified MCAP bag + sidecar |
| **Transfer (dev host → DGX)** | UploadDaemon, Transport | Dumbly move the bag to the DGX (or CI stand-in) |
| **Index/Query (DGX)** | IngestService, ManifestStore | Make the bag findable + queryable |
| **Verify/Consume (repo+CI / desktop)** | ReplayRegressionTest, ReferenceBag+ProvenanceNote, FoxgloveVerification | Assert the bag's contents (CI) and render it (human) |

#### 4.3.2 Record layer — Design Notes
**Conventions:** Python 3.12, colcon package, ros2 launch include pattern (matches 01's workspace conventions).
**New in this design:** the `patrol_logging` package + `record.launch.py` include + `recorded_topics.yaml`.
**Integration points:** included by 02's `mission_patrol.launch.py`; output dir watched by the Transfer layer.

#### 4.3.3 Transfer layer — Design Notes
**Conventions:** dumb producer (PRD tenet 2); dependency-light CLI tools (rsync/ssh).
**New in this design:** `UploadDaemon` + `Transport` abstraction.
**Integration points:** input = Record layer's output dir; output = Index layer's landing dir (real DGX or CI stand-in).

#### 4.3.4 Index/Query layer — Design Notes
**Conventions:** containerized service in `docker/ingest/` (01 provides the slot); stdlib SQLite.
**New in this design:** `IngestService`, `ManifestStore`, the `docker/ingest/` Dockerfile + service + query CLI.
**Integration points:** consumes the Transfer layer's landed bags; serves `manifest_query`.

#### 4.3.5 Verify/Consume layer — Design Notes
**Conventions:** pytest in `tests/replay/` (plugs into ADR-0002 CI tiers); Foxglove as a desktop app.
**New in this design:** `ReplayRegressionTest`, the reference bag + provenance note, the Foxglove layout/procedure.
**Integration points:** consumes a recorded/reference bag; the replay test is a CI gate.

---

### 4.4 Systemic / Platform Interfaces

Interface categories elicited from the actual Phase 1 platform (01's containers + ADR-0002 CI + the plan's stack), not a generic checklist.

#### 4.4.1 Interface Integration Summary

| Interface | Current State (Section 3) | Design Changes | Priority |
|-----------|--------------------------|----------------|----------|
| **Containerization** | 01 provides `sim`/`dev` base + an empty `docker/ingest/` slot; MCAP plugin pinned in the base | This design fills `docker/ingest/` with the ingest service container; recorder/daemon run in the `dev`/`sim` containers | P1 |
| **CI (ADR-0002 two-layer)** | Per-PR pure-Python tier + slow SITL/integration tier | Replay test runs as a CI gate (cheaper-than-SITL lane); upload→ingest integration runs against a local stand-in (OQ-7) | P1 |
| **Observability (logging)** | Plain Python logging in containers (no metrics stack in Phase 1) | Recorder/daemon/ingest emit structured log lines (bag name, transfer status, index result) — no new telemetry infra | P2 |
| **Configuration** | YAML configs in `ros2_ws`; env/CLI flags | `recorded_topics.yaml`, upload `--target`/`--transport`, replay `assertions.yaml` — all file/flag, no secrets store | P2 |
| **Security (transport)** | SSH access dev→DGX (operator-managed keys) | rsync-over-SSH uses existing SSH key auth; no new identity system (single-operator Phase 1) | P2 |

#### 4.4.2 CI Interface (ADR-0002)
**Current state:** §3.2 — two-layer CI: per-PR pure-Python + slow SITL tier.
**Design changes:** The `ReplayRegressionTest` runs in the deterministic lane (no simulator), budgeted ≤ 90 s (OQ-6). The upload→ingest→manifest chain is exercised in CI against a **local stand-in target** (a temp dir / loopback SSH), so no DGX is pulled into CI (OQ-7; "no DGX required for Phase 1 dev").
**Failure mode:** if the reference bag (Git LFS) fails to materialize in CI, the replay test errors loudly (not silently skips) — LFS-pull failure is a hard CI failure (§4.4.5).

#### 4.4.3 Containerization Interface
**Current state:** §3.2 — `docker/ingest/` is an empty slot; MCAP plugin pinned in 01's base.
**Design changes:** add `docker/ingest/Dockerfile` (the ingest service); recorder/daemon need no new image (run in 01's `dev`/`sim`).
**Failure mode:** if the MCAP storage plugin is absent from the base, recording fails fast at `start()` (guard) rather than silently writing sqlite3 — surfaced as a hard error citing the missing plugin (§4.4.5).

#### 4.4.4 Observability Interface
**Current state:** plain Python logging; no Prometheus/Grafana in Phase 1.
**Design changes:** structured log lines at each stage (record-started/stopped, transfer-ok/retry, index-ok). No metrics backend (right-sized for Phase 1).
**Failure mode:** logs are the only observability; a stuck daemon is detected by absence of "transfer-ok" lines, not by an alert (acceptable single-operator posture).

#### 4.4.5 Cross-cutting Failure Modes

| Category | Failure mode | Detection | Degraded behavior | Recovery |
|----------|--------------|-----------|-------------------|----------|
| **Persistent state** | Bag output disk full mid-record | `ros2 bag record` write error | Recorder stops; `stop()` still writes a (partial) sidecar marking the run incomplete | Operator frees space; partial bag retained for inspection, not uploaded as "complete" |
| **Persistent state** | MCAP not finalized (recorder killed, not clean-stopped) | No `<bag>.meta.json` sidecar present | UploadDaemon's "complete" guard fails → bag is NOT uploaded | Operator re-runs or manually finalizes; daemon only ships bags with a sidecar |
| **Persistent state** | DGX manifest SQLite file locked/corrupt | `sqlite3` error on upsert | IngestService retries; on corruption, logs + halts indexing for that bag | Operator restores/rebuilds SQLite (it is just an index — re-ingest from bags rebuilds it) |
| **Persistent state** | Concurrent re-upload of the same bag | Two index calls for one `bag_id` | Upsert is idempotent (PK = `bag_id`); second write updates in place | None needed — idempotent by design |
| **Network dependency** | DGX unreachable (transient) during upload | `Transport.send` returns false / times out | Daemon retries with backoff; bag stays on dev host (never deleted before confirmed) | Auto-recovers when DGX returns; queued bags drain |
| **Network dependency** | DGX unreachable (persistent) | Repeated `send` failures past a threshold | Daemon keeps bags locally + logs a persistent-failure warning | Operator fixes connectivity; no data loss (producer is the source of truth until confirmed) |
| **Network dependency** | Sidecar arrives but bag does not (partial transfer) | IngestService sees sidecar without a readable bag | Ingestion defers the row until the bag is present/readable | Daemon re-sends; idempotent index upsert on success |
| **Plugin / extension** | Git LFS pointer not resolved in CI (reference bag absent) | Replay test cannot open the bag | Replay test **fails loudly** (hard CI failure, not skip) | Fix LFS config / re-pull; reference bag is required, not optional |
| **Plugin / extension** | MCAP storage plugin missing in container base | Recorder `start()` guard fails | Recording fails fast (no silent sqlite3 fallback) | Operator/01 fixes the base image; pinned-stack manifest is the contract |
| **Identity provider** | (all sub-modes) | [OOS: single-operator Phase 1 — SSH key auth only; no IdP, no tokens, no multi-tenant identity] |
| **Mesh / cross-cluster** | (all sub-modes) | [OOS: two static hosts (dev + DGX) over rsync/SSH; no service mesh, no cross-cluster federation in Phase 1] |

---

### 4.5 Key Interaction Sequences

#### Sequence 1: Happy path — automatic record on mission launch (LR-1, LR-2, LR-7)

```
Operator            mission_patrol.launch.py(02)   BagRecorderWrapper        bag dir
  |                          |                            |                     |
  ├─ ros2 launch … ─────────►│                            |                     |
  │                          ├─ include record.launch.py ►│                     |
  │                          │                            ├─ start(mission_id,  |
  │                          │                            │   topics) ──────────► (ros2 bag record
  │                          │  (mission flies patrol)    │   --storage mcap)    │  writing .mcap)
  │                          ├─ mission end ─────────────►│                     |
  │                          │                            ├─ stop() ────────────► finalize .mcap
  │                          │                            ├─ write <bag>.meta.json ►│
  │  ◄── bag present in known dir (patrol_<id>_<ts>.mcap) ─────────────────────────┤
```

#### Sequence 2: Upload → ingest → manifest (LR-3, LR-4)

```
bag dir        UploadDaemon       Transport        IngestService(DGX)     ManifestStore
  │                │                  │                   │                    │
  ├─ new bag+sidecar ►│ (watch fires) │                   │                    │
  │                ├─ guard: .mcap + .meta.json both present                    │
  │                ├─ send(bag) ─────►│ rsync/SSH ───────►│ (lands on DGX)      │
  │                ├─ send(sidecar) ─►│ ─────────────────►│                     │
  │                │  ◄── ok (≤30s) ──│                   ├─ derive duration +  │
  │                │                  │                   │   topics from bag   │
  │                │                  │                   ├─ upsert row ───────►│
  │                │                  │                   │  ◄── stored ────────┤
  Operator ── manifest_query --recent N ────────────────────────────────────► returns rows
```

#### Sequence 3: Error/edge — DGX unreachable, then recovers (failure path)

```
UploadDaemon          Transport            DGX
  │                      │                  ✗ (unreachable)
  ├─ send(bag) ─────────►│ ──timeout──────► ✗
  │  ◄── false ──────────│                  │
  ├─ retry w/ backoff (bag stays on dev host, NOT deleted)
  │      … DGX returns …
  ├─ send(bag) ─────────►│ ─────────────────► ✓
  │  ◄── true ───────────│
  └─ mark bag confirmed-uploaded (data loss impossible: producer held the bag)
```

#### Sequence 4: End-to-end single artifact (LR-8)

```
ONE full-patrol bag B = patrol_<id>_<ts>.mcap
  B ──recorded by──► BagRecorderWrapper        (LR-1/LR-2/LR-7)
  B ──uploaded by──► UploadDaemon → DGX        (LR-3)
  B ──indexed by───► IngestService → ManifestStore; appears in manifest_query  (LR-4)
  B ──(or trimmed slice) is the basis of──► ReplayRegressionTest passes in CI   (LR-5)
  B ──opened in────► Foxglove: camera + mission-state + 3D-pose panels populated (LR-6)
  ▲ same artifact at every stage — no manual stitching (the integrative claim)
```

---

### 4.6 Data Model Changes (Consolidated)

#### Manifest (SQLite — DGX, OQ-3 resolved)

| Table | Change | Detail |
|-------|--------|--------|
| `bag_manifest` | **New table** | `bag_id` (PK), `mission_id`, `recorded_utc`, `duration_s` (derived), `topics_json` (derived list+counts), `metadata_json` (sidecar), `ingested_utc`. Realizes LR-4's queryable record. |

#### On-disk sidecar (dev host)

| Artifact | Change | Detail |
|----------|--------|--------|
| `<bag>.meta.json` | **New artifact** | JSON sidecar (OQ-10): `mission_id`, `bag_filename`, `started_utc`, `ended_utc`, `recorded_topics`, `mission_config_ref`. Realizes LR-2's per-bag metadata. |

No changes to any 01/02/04-owned message or topic (this docset only *records* them).

---

## 5. Design Questions FAQ

### Q1: Main components and interactions
Five components plus two artifacts (§4.2.1): **BagRecorderWrapper** (records the bag, dev host, in `ros2_ws/src/patrol_logging/`), **UploadDaemon** + its **Transport** helper (dumbly ships the bag to the DGX, under `analysis/upload_daemon/`), **IngestService** + **ManifestStore** (index + query, in `docker/ingest/` on the DGX), **ReplayRegressionTest** (CI gate, in `tests/replay/`), and **FoxgloveVerification** (a desktop-app load-and-render check). Build order follows the milestone split: M7 = record side (recorder + Foxglove check), M8 = the replay pipeline (daemon → ingest → manifest → replay test). Data flows record → transfer → index/query → verify/consume (§4.3, §4.5). Every component named here appears in §4.2.1 inventory and has a §4.2.x spec.

### Q2: Core API contracts and data models
This is infrastructure, not a REST service — the "API contracts" are CLI/launch/config surfaces and one data store. **Launch include:** `record.launch.py` (included by 02's `mission_patrol.launch.py`). **CLIs:** `python -m upload_daemon --watch <dir> --target <dest> --transport rsync|s3`; `python -m manifest_query --recent N | --mission <id>`. **Data store:** the SQLite `bag_manifest` table (§4.6) with columns `bag_id, mission_id, recorded_utc, duration_s, topics_json, metadata_json, ingested_utc`. **Sidecar:** `<bag>.meta.json` (§4.2.2). **Internal contract:** `Transport.send(local_path, remote_path) -> bool` (§4.2.3a). Every surface here traces to LR-1…LR-4 (no surface without an FR); each CLI/table named here matches its §4.2.x spec exactly.

### Q3: Deployment and infrastructure dependencies
Runs inside 01's `sim`/`dev` containers (recorder, daemon) and a new `docker/ingest/` container (ingest service + SQLite). New infra this design introduces: `docker/ingest/Dockerfile`; the SQLite manifest file; the Git-LFS reference bag. Config keys introduced: `recorded_topics.yaml`; upload `--target`/`--transport`; replay `assertions.yaml`. No new managed services, no Postgres, no metrics stack (right-sized). No DGX is required for Phase 1 dev — CI uses a local stand-in target (OQ-7). MCAP storage plugin must be present in the container base (01 owns; §4.4.3 fails fast if absent).

### Q4: External components and interfaces
External dependencies: **Foxglove Studio** (installed desktop app — not built/containerized; §4.2.6); **`ros2 bag record`/`ros2 bag play`** + the **MCAP storage plugin** (from 01's base); **rsync/ssh** binaries (Transport, §4.2.3a); **Git LFS** (reference bag, §4.2.6). Consumed (not owned) topic shapes: `/fmu/out/*` (01), `/patrol/*` + camera frame (02/04), `patrol_interfaces/msg/CheckpointCapture` (04). Each external dep appears as a §4.4 row or a §3.3 consumed-topic row, and vice versa.

### Q5: Testing strategy (unit, integration, E2E)
**Unit** (pure-Python, per-PR tier): sidecar construction (BagRecorderWrapper), bag-naming, daemon "complete" guard logic, ManifestStore upsert/query, IngestService fact-derivation (mockable bag reader). **Integration** (against a local DGX stand-in, OQ-7): upload→ingest→manifest chain on a small fixture bag. **Replay regression** (the core test, §4.2.5): plays the reference bag, asserts presence+rate, plus a deliberate-break self-check; ≤ 90 s (OQ-6). **E2E** (M8 exit, partly manual): the LR-8 single-artifact pass + Foxglove render. These categories match the §6.2 milestone testing tables exactly.

### Q6: Security implications and auth interactions
Single-operator Phase 1: the only trust boundary is dev-host→DGX, secured by **existing SSH key auth** (rsync over SSH) — no new identity system, no tokens, no multi-tenant isolation (§4.4.1, §4.4.5 IdP row marked OOS). Bags and the manifest are non-sensitive sim data. The relevant integrity concern is *data integrity*, not access control: the dumb-producer invariant (§3.4) means the producer never deletes a bag before transfer is confirmed, and IngestService re-derives topic facts from the bag (a tampered/buggy sidecar can't corrupt indexed topic truth). Defense-in-depth here = "the bag on the dev host is the source of truth until the DGX confirms receipt," enforced at the producer (no-delete-before-confirm) and at ingestion (facts-from-bag-not-sidecar).

### Q7: Technical risks and open questions
Top risks: **(R1)** wrong recorded-topic set / image representation baked into the reference bag (OQ-1/OQ-2, Provisional — settled defaults applied, confirmed jointly with 04 before the reference bag is generated). **(R2)** replay flakiness from too-tight rate assertions (mitigated: small subset, presence-first, ±40% band — OQ-5). **(R3)** reference-bag rot / repo bloat (mitigated: Git LFS + trimmed slice + provenance note — OQ-4, LR-9). **(R4)** bag size exceeding "reasonable" (OQ-9 deferred to first measured 5-min recording; compressed image contains it). Every Provisional/Deferred status here matches a §2 OQ row (OQ-1, OQ-2 Provisional; OQ-9 Deferred); every Resolved item (OQ-3,4,5,6,7,8,10) matches a §2 Resolved row.

---

## 6. Implementation Plan

### 6.0 Linear Project
**Project:** TBD — Linear bootstrap is a separate post-design step (`/drive` stops at the PRD+Design pair for combined human review).
**Team:** TBD (confirm team/project with the user before creating issues).
**Initiative:** Patrol Drone Phase 1.
**Created from:** Section 6 of this document.

### 6.1 Milestone Overview

Walking-skeleton shape: **M7 (skeleton)** delivers the thinnest end-to-end *record-and-prove* slice — a launch produces a bag that a human can open in Foxglove. **M8 (layer 1)** thickens that bag into the full automated pipeline (upload → manifest → CI replay) and adds the end-to-end single-artifact guarantee. Components legitimately span both milestones (BagRecorderWrapper is exercised in both); coverage is checked across the set, not per-milestone.

| # | Milestone | Type | Shippable Demo | Scope | Dependencies | Exit Criteria | Linear |
|---|-----------|------|----------------|-------|-------------|---------------|--------|
| M7 | Automatic MCAP recording | **skeleton** | Operator runs `mission_patrol.launch.py`, gets `patrol_<id>_<ts>.mcap` in the known dir, and **opens it in Foxglove** with camera + mission-state + 3D-pose panels populated | BagRecorderWrapper + `record.launch.py` include + `recorded_topics.yaml` + JSON sidecar; records `/patrol/checkpoint_capture`; Foxglove load check | 01 (containers, MCAP plugin), 02 (launch include point), 04 (`CheckpointCapture` + camera topic) | `ros2 bag info` shows expected topics (incl. `/patrol/checkpoint_capture`) with non-zero counts; bag hundreds-of-MB not GB; bag opens in Foxglove with expected panels | *post-approval* |
| M8 | Replay pipeline (bag → DGX → manifest → CI → Foxglove) | **layer 1: automation + regression** | The M7 bag now auto-uploads to the DGX (or stand-in) within 30 s, appears in `manifest_query`, the CI replay test passes (and catches a dropped topic), and one full-patrol bag is witnessed carrying through every stage | UploadDaemon + Transport; IngestService + ManifestStore (`docker/ingest/`); ReplayRegressionTest + reference bag (Git LFS) + provenance note; LR-8 end-to-end pass | M7 | bag on DGX ≤30 s; appears in manifest query; replay test green in CI + deliberate-break fails; one bag witnessed end-to-end (LR-8) | *post-approval* |

### 6.2 Milestone Details

#### M7: Automatic MCAP recording

**Type:** skeleton
**Goal:** every mission launch produces exactly one identified MCAP bag, recording the agreed topics (incl. `/patrol/checkpoint_capture`), that a human can already open in Foxglove.
**Shippable demo:** Operator runs `ros2 launch patrol_bringup mission_patrol.launch.py`, sees `patrol_<missionId>_<timestamp>.mcap` appear, runs `ros2 bag info` (expected topics, non-zero counts), and opens the bag in Foxglove with the camera feed, mission state, and 3D pose history panels populated.
**Dependencies:** 01 (containers + MCAP plugin), 02 (the `mission_patrol.launch.py` include slot), 04 (`/patrol/checkpoint_capture` + camera `CompressedImage` topic).
**Exit criteria:** one MCAP bag in the known dir; `ros2 bag info` shows expected topics including `/patrol/checkpoint_capture` with non-zero counts; bag size hundreds-of-MB not GB; Foxglove render works.

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| Upload daemon, manifest, replay test | PRD Milestones M8; design §4.2.3–§4.2.5 | M8 |
| Precise bag-size MB ceiling | PRD OQ-9; design §2 OQ-9 | Design (after first canonical recording) |
| S3 transport | PRD OQ-8; design §4.2.3a | out-of-version (rsync ships; S3 is interface-only) |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T7.1 | Create `patrol_logging` colcon package shell | `ros2_ws/src/patrol_logging/package.xml` (new); `ros2_ws/src/patrol_logging/setup.py` (new) | BagRecorderWrapper | Record | S | 01 workspace | *post-approval* |
| T7.2 | Implement recorder start/stop wrapping `ros2 bag record --storage mcap`, with `patrol_<id>_<ts>` naming | `ros2_ws/src/patrol_logging/patrol_logging/recorder.py` (new) | BagRecorderWrapper | Record | M | T7.1 | *post-approval* |
| T7.3 | `recorded_topics.yaml` broad topic set (OQ-1 default, incl. `/patrol/checkpoint_capture` + camera `CompressedImage`) | `ros2_ws/src/patrol_logging/config/recorded_topics.yaml` (new) | BagRecorderWrapper | Record | S | T7.2 | *post-approval* |
| T7.4 | JSON sidecar emission on clean stop | `ros2_ws/src/patrol_logging/patrol_logging/recorder.py` (modify) | BagRecorderWrapper | Record | S | T7.2 | *post-approval* |
| T7.5 | `record.launch.py` includable fragment (started/stopped with mission lifecycle) | `ros2_ws/src/patrol_logging/launch/record.launch.py` (new) | BagRecorderWrapper | Record | M | T7.2 | *post-approval* |
| T7.6 | Foxglove layout + load-and-render verification note | `analysis/foxglove/README.md` (new); `analysis/foxglove/patrol_layout.json` (new) | FoxgloveVerification | Verify/Consume | S | T7.2 | *post-approval* |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | recorder naming + sidecar construction | bag name matches `patrol_<id>_<ts>.mcap`; sidecar carries mission_id/timestamps/topics/config-ref; storage flag is mcap not sqlite3 |
| Integration | recorder under a real (short) launch | one bag produced; `ros2 bag info` shows expected topics with non-zero counts |
| E2E | Foxglove render | a recorded bag opens with camera + mission-state + 3D-pose panels populated |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| `analysis/foxglove/README.md` | dev/operator | how to open a bag in Foxglove + the saved layout |
| `recorded_topics.yaml` comments | dev | what is recorded and why (the broad set) |

#### M8: Replay pipeline (bag → DGX → manifest → CI → Foxglove)

**Type:** layer 1: automation + regression
**Goal:** the bag M7 produces now flows automatically dev-host→DGX, is indexed + queryable, is guarded by a CI replay regression test, and one full-patrol bag is witnessed carrying through every stage end-to-end.
**Shippable demo:** after a patrol, the bag auto-appears on the DGX (or CI stand-in) within 30 s, `manifest_query --recent 1` returns its row (mission/time/duration/topics/metadata), `pytest tests/replay` passes in CI (and a deliberate dropped-topic variant fails), and the operator confirms the same bag renders in Foxglove.
**Dependencies:** M7.
**Exit criteria:** ≤30 s to DGX; appears in + returned by the manifest query; replay test green in CI + catches a deliberate break; LR-8 single-artifact pass witnessed.

##### Out of Scope

| Item | Source | Deferred to |
|------|--------|-------------|
| Postgres / production metadata store | PRD Out-of-Scope; design OQ-3 (SQLite chosen) | later phase if scale demands |
| Replay regression on learned-model (YOLO) outputs | PRD Out-of-Scope; PRD non-goal | Phase 3 |
| Real DGX dependency in CI | PRD OQ-7; design §4.4.2 (local stand-in) | never (stand-in is the Phase 1 posture) |
| S3 transport implementation | PRD OQ-8; design §4.2.3a | out-of-version (interface parity only) |

##### Tasks

| # | Task | Files Touched | Component | Layer | Size | Dependencies | Linear |
|---|------|---------------|-----------|-------|------|-------------|--------|
| T8.1 | `Transport` protocol + `RsyncSshTransport` (S3 stub for parity) | `analysis/upload_daemon/transport.py` (new) | Transport | Transfer | M | M7 | *post-approval* |
| T8.2 | `UploadDaemon`: watch dir, "complete" guard (bag+sidecar), transfer + retry/backoff, ≤30 s target | `analysis/upload_daemon/upload_daemon.py` (new) | UploadDaemon | Transfer | M | T8.1 | *post-approval* |
| T8.3 | `ManifestStore`: SQLite schema + upsert/query (store-agnostic interface) | `docker/ingest/manifest_store.py` (new) | ManifestStore | Index/Query | M | M7 | *post-approval* |
| T8.4 | `IngestService`: derive duration+topics from bag, read sidecar, idempotent upsert | `docker/ingest/ingest_service.py` (new) | IngestService | Index/Query | M | T8.3 | *post-approval* |
| T8.5 | `manifest_query` CLI (`--recent N`, `--mission <id>`) | `docker/ingest/manifest_query.py` (new) | ManifestStore | Index/Query | S | T8.3 | *post-approval* |
| T8.6 | `docker/ingest/Dockerfile` (ingest service container, fills the empty slot) | `docker/ingest/Dockerfile` (new) | IngestService | Index/Query | S | T8.4 | *post-approval* |
| T8.7 | Generate + trim the reference bag (Git LFS) + provenance note | `tests/replay/reference/patrol_reference.mcap` (new, LFS); `tests/replay/README.md` (new); `.gitattributes` (modify) | ReferenceBag + ProvenanceNote | Verify/Consume | M | M7 | *post-approval* |
| T8.8 | `ReplayRegressionTest`: play reference bag, assert presence+rate, deliberate-break self-check; assertion config | `tests/replay/test_replay_regression.py` (new); `tests/replay/assertions.yaml` (new) | ReplayRegressionTest | Verify/Consume | M | T8.7 | *post-approval* |
| T8.9 | Wire replay test + stand-in upload/ingest into CI (ADR-0002 tiers) | `.github/workflows/*.yml` (modify) | ReplayRegressionTest | Verify/Consume | M | T8.8 | *post-approval* |
| T8.10 | LR-8 end-to-end single-artifact verification (scripted/witnessed) | `analysis/e2e_check.md` (new) | End-to-end flow | all | S | T8.2, T8.4, T8.8 | *post-approval* |

##### Testing

| Test Type | Scope | Key Scenarios |
|-----------|-------|---------------|
| Unit | daemon guard, ManifestStore, IngestService fact-derivation | "complete" guard rejects bag without sidecar; upsert is idempotent; duration/topics derived from bag not sidecar |
| Integration | upload→ingest→manifest against a local stand-in (OQ-7) | a fixture bag lands, is indexed, and is returned by `manifest_query` |
| Replay (core) | reference bag presence/rate + deliberate break | passes deterministically ≤90 s; dropped-topic variant fails |
| E2E | LR-8 single artifact | one full-patrol bag witnessed: record→upload→manifest→replay→Foxglove |

##### Documentation

| Artifact | Audience | Content |
|----------|----------|---------|
| `tests/replay/README.md` | dev | reference-bag provenance + regeneration (LR-9) |
| `docker/ingest/` README/Dockerfile comments | dev/ops | how ingestion + the manifest run on the DGX (or stand-in) |
| `analysis/e2e_check.md` | dev | the end-to-end single-artifact verification procedure |

### 6.3 Layered Delivery Sequence

**Skeleton + layering rationale:**

1. **M7 (skeleton, Automatic MCAP recording)** is the thinnest end-to-end slice that already crosses Record → Verify/Consume: a launch produces a real bag a human can open in Foxglove. After M7, a stakeholder can demo "run a mission, get an inspectable bag in Foxglove" — the recorder and the bag-output contract are proven before any pipeline is built on top. This intentionally de-risks the riskiest-to-reverse decision (the bag schema / topic set) first.
2. **M8 (layer 1: automation + regression)** thickens that same bag with the rest of the lifecycle — Transfer (UploadDaemon), Index/Query (IngestService + ManifestStore), and an automated CI Verify lane (ReplayRegressionTest) — and adds the LR-8 single-artifact guarantee. After M8, the demo also shows the bag auto-arriving on the DGX, being queryable, and a CI gate that catches a topic regression. Why this layer next: it converts the manual M7 artifact into the automated regression/corpus pipeline the PRD commits to, without changing the M7 bag contract.

**What gets demoable, when:**
- After M7: run a mission → identified MCAP bag → open in Foxglove (panels populated).
- After M8: M7 demo + auto-upload to DGX (≤30 s) + manifest query + CI replay gate (catches a dropped topic) + one bag witnessed end-to-end.

**Scope-shedding plan:**
- If schedule slips, shed M8 tasks in this order: S3 stub (already out-of-version) → Foxglove layout polish → manifest-query ergonomics. The hard floor is M7 (a recorded, identified, Foxglove-openable bag) — that alone is a shippable, useful artifact and the basis for the reference bag.
- Within M8, the replay regression test (LR-5) is a hard floor among the layer's value (it is the "bag becomes the regression test" payoff); upload/manifest can degrade to manual if needed, but the CI replay gate should not be shed.

**Parallel work opportunities:**
- Within M8, the **Transfer** track (T8.1–T8.2) and the **Index/Query** track (T8.3–T8.6) are independent until they meet at the stand-in integration test — they can be built concurrently. The **reference bag + replay test** (T8.7–T8.9) depends only on M7's recorder output, not on the upload/ingest tracks, so it can proceed in parallel too. (M7 is inherently serial as the skeleton.)

### 6.4 Definition of Done

A milestone is complete when:
- [ ] All tasks are implemented and code-reviewed
- [ ] All specified tests pass (unit, integration, replay, E2E as applicable)
- [ ] **Shippable demo runs end-to-end** (M7: record→Foxglove; M8: record→upload→manifest→replay→Foxglove)
- [ ] Documentation artifacts are written and reviewed
- [ ] No P1 bugs remain
- [ ] Systemic interfaces (containerization, CI, observability) are integrated per Section 4.4

---

## 7. Changelog

### v0.1.0 — 2026-06-03

**Initial version** — Created via CreateDesign workflow from `docs/phase1/05-logging-replay/prd.md` (rev 2). Resolved design-target OQs (OQ-3 SQLite, OQ-4 Git LFS, OQ-5 assertion subset + ±40%, OQ-6 ≤90 s, OQ-7 local stand-in, OQ-8 rsync/SSH, OQ-10 JSON sidecar); carried OQ-1/OQ-2 Provisional (settled defaults applied, pending combined human review) and OQ-9 Deferred (precise MB ceiling pending first measured recording).

**ReviewDesign (auto-pilot) — 2026-06-03:** Evaluated against all 13 ReviewDesign dimensions; D2 PRD-trace audit and D13 grep test run with §3.2 citations independently re-verified against the live repo + sibling DoDs. Overall **Ready**; no ≥medium finding. No ReviseDesign pass triggered (medium-floor policy). One sub-medium deferred note recorded (§2 reviewer note): camera-frame topic owner attribution (03 vs 04) to confirm at OQ-1/OQ-2 combined review; §3.3 updated to mark 04 as the topic-shape/publication owner pending that confirmation. Version left at 0.1.0 (no requirements/component cascade; review-only annotations added).

---

## Appendix A: Workstream Overviews

Single-workstream design (one docset, solo dev). The two milestones M7/M8 are the delivery waves; see §6. No separate workstream decomposition needed.

---

## Appendix B: User Acceptance Criteria

Carried in-substance from the PRD's Appendix B (the sole requirements source). Each maps to the design component in §4.1 and to a falsifiable AC in `dod.md` §4.

### B1. Logging & Replay Pipeline

**UAC-LR-1: Automatic per-run MCAP recording** *(→ BagRecorderWrapper §4.2.2; dod AC-1)*
GIVEN a mission launched via `ros2 launch patrol_bringup mission_patrol.launch.py`
WHEN the mission completes
THEN exactly one MCAP rosbag exists in the known output location, named `patrol_<missionId>_<timestamp>.mcap`, produced by the launch-invoked wrapper with no separate operator command.

**UAC-LR-2: Bag identifiability and recorded-topic completeness** *(→ BagRecorderWrapper §4.2.2; dod AC-2)*
GIVEN a produced bag
WHEN `ros2 bag info <bag>` is run
THEN it lists the expected topics (`/fmu/out/*`, `/patrol/*`, camera image, TF, mission state/waypoint/abort) each with non-zero message counts, a per-bag metadata sidecar carries mission ID / timestamp / topic set / config correlation, and the bag is under a few hundred MB for a 5-minute mission.

**UAC-LR-3: Automatic upload to DGX** *(→ UploadDaemon §4.2.3; dod AC-3)*
GIVEN a mission has ended and the upload daemon is watching the output directory
WHEN the new bag appears
THEN the daemon uploads it to the DGX automatically, targeting within 30 s of mission end, performing transfer only (no indexing in the producer).

**UAC-LR-4: Manifest indexing and query** *(→ IngestService + ManifestStore §4.2.4; dod AC-4)*
GIVEN a bag uploaded to the DGX
WHEN ingestion indexes it
THEN it appears in the SQLite/DuckDB manifest with mission, time, duration, topics, and metadata, and is queryable at Phase 1 scale.

**UAC-LR-5: Replay regression test in CI** *(→ ReplayRegressionTest §4.2.5; dod AC-5)*
GIVEN a checked-in reference bag
WHEN the replay regression test runs in CI
THEN it replays the bag via `ros2 bag play`, subscribes to the expected topics, asserts they appear at expected rates within tolerance, passes deterministically, and fails if an expected topic is dropped/renamed.

**UAC-LR-6: Foxglove renders a recorded bag** *(→ FoxgloveVerification §4.2.6; dod AC-6)*
GIVEN a recorded mission bag
WHEN it is opened in Foxglove Studio (desktop app)
THEN the camera feed, mission state, and 3D pose history render with the expected panels populated, with no Foxglove containerization introduced.

**UAC-LR-7: CheckpointCapture topic recorded by the bag pipeline** *(→ BagRecorderWrapper §4.2.2; dod AC-7)*
GIVEN the perception node (04) publishes `patrol_interfaces/msg/CheckpointCapture` on `/patrol/checkpoint_capture`
WHEN a mission runs and is recorded
THEN the bag contains `/patrol/checkpoint_capture` with non-zero message count, the recorded message carries `image_path` (not by-value pixels) with live frames on a separate `sensor_msgs/CompressedImage` topic, and it is the same compiled `patrol_interfaces` type 04 publishes.

**UAC-LR-8: End-to-end bag artifact** *(→ §4.5 Sequence 4; dod AC-8)*
GIVEN a full multi-checkpoint patrol launched via `mission_patrol.launch.py`
WHEN it completes
THEN the single bag it produced is recorded (LR-1/2), uploaded (LR-3), indexed in the manifest (LR-4), replayable by the regression test (LR-5), and renders in Foxglove (LR-6) — one artifact carried end-to-end with no manual stitching between stages.

#### Inferred Requirements [INFERRED]

**INF-L1: Metadata-sidecar serialization is JSON** *(ref: UAC-LR-2; OQ-10)*
GIVEN the recorder finishes a run
WHEN it writes the per-bag metadata sidecar
THEN the sidecar is `<bag>.meta.json` (JSON), parseable by `IngestService` without a YAML dependency. (Realization detail of LR-2's sidecar, not a new contract.)

**INF-L2: Dumb-producer integrity invariant** *(ref: UAC-LR-3, UAC-LR-4; §3.4)*
GIVEN a bag is being transferred
WHEN transfer has not yet been confirmed
THEN the producer never deletes the local bag, AND `IngestService` derives topic/duration facts from the bag itself (not the sidecar) — so neither a transfer failure nor a buggy sidecar can lose or corrupt the indexed truth.

**INF-L3: Reference bag is small + LFS-tracked + regenerable** *(ref: UAC-LR-5, LR-9; OQ-4)*
GIVEN the replay regression baseline
WHEN it is checked into the repo
THEN it is a trimmed slice tracked in Git LFS with a documented regeneration procedure, so the repo stays light and the baseline is deliberately refreshable.
