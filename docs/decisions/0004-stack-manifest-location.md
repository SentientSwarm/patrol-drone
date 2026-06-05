# ADR-0004: Pinned stack manifest is a standalone `stack-manifest.toml`; CLAUDE.md table is a synced summary

**Status:** Accepted
**Date:** 2026-06-04
**Deciders:** Egemen Cankaya (project owner)

## Context

Docset 01-platform requires a **pinned stack manifest** — a single source of truth pinning every
toolchain layer (OS, ROS 2 Jazzy, PX4, Gazebo Harmonic, uXRCE-DDS, Python, colcon, Docker). This is
PLAT-7 in the PRD and AC-8 in the Definition of Done.

The 01-platform design at full depth ([design.md](../phase1/01-platform/design.md) v0.3.0, §4.2.9 /
component C9) specifies this manifest as a **standalone `stack-manifest.{toml,yaml}` at the repo
root**: a structured file whose `px4_version` / `px4_msgs_ref` pair is the *single edit point* for
OQ-3 (the exact PX4 tag + matching `px4_msgs` branch the M1–M2 spike settles), and which the
Dockerfiles (`docker/sim`, `docker/dev`), `docker-compose.yml`, and the README all cite — pulling
values via `.env` / build `ARG`s so no version literal is duplicated.

An earlier draft of this ADR proposed co-locating the manifest *inside* CLAUDE.md's existing
"Stack (pinned)" table, to avoid maintaining a second file. That draft carried an explicit
**promote-to-standalone trigger**: *"if the manifest is consumed by automation — a CI pin check, a
container build-ARG source, or a programmatic drift gate — promote it to a standalone file."* The
design's container build-ARG plumbing is exactly that condition. The trigger fired before any of
this work merged, so the standalone form is adopted directly.

## Decision

The **canonical pinned stack manifest is `stack-manifest.toml` at the repo root.** It is the single
source of truth; the `px4_version` / `px4_msgs_ref` pair is the OQ-3 edit point; Dockerfiles,
compose, and the README cite it.

CLAUDE.md's "Stack (pinned)" table is retained as a **maintained, human-facing summary** — a fast
in-context reference for humans and the AI assistant — that points to `stack-manifest.toml` as the
source of truth. The `.toml` is authoritative; the table must not drift from it. When a version
changes, edit the `.toml` first, then reconcile the table.

## Consequences

### Positive
- One machine-parseable source of truth; the OQ-3 resolution is a two-line edit (`px4_version`,
  `px4_msgs_ref`) that the container ARGs and README consume — nothing else moves.
- Matches the design (§4.2.9) and unblocks M2's compose/`.env`/ARG plumbing without scraping Markdown.
- Versions live in a format CI can read for a future pin-drift / determinism gate (PRD H3 / INF-P2).

### Negative
- Two representations of the pinned versions (the `.toml` and the CLAUDE.md summary table) must be
  kept in sync. Mitigated by naming the `.toml` canonical and the table explicitly a *summary*, and
  by the "edit the `.toml` first" rule above.

### Neutral
- AC-8 / PLAT-7 are satisfied by the `.toml` (every layer explicitly pinned); the CLAUDE.md table is
  convenience, not the contract.
- Reverses this ADR's own earlier draft (manifest-in-CLAUDE.md); that decision was not wrong, its
  promote trigger simply fired immediately.

## Alternatives considered

- **Manifest in CLAUDE.md only (the earlier draft).** Cleaner in file count, but not machine-parseable;
  the design's build ARGs would have to scrape a Markdown table embedded in a larger context file.
  Rejected once the build-ARG-source trigger fired.
- **Two copies with no designated canonical.** Guarantees drift — the exact failure a "single source
  of truth" manifest exists to prevent. Rejected.
