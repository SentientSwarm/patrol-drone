Phase 1 wrap-up: burn down the remaining Linear backlog for the patrol-drone Phase 1 close-out.
All milestone deliverables (M1–M8) are merged to main via PR #16; this is the closing pass — no
new milestone work, no Phase 2+ scope (VIO/YOLO/Isaac/multi-drone stay out).

## Issues to close (SWM team, all currently Backlog)

Real work (this is the devloop scope):
- SWM-84 — Flip the `replay-regression` CI lane onto the published GHCR digest. NOW UNBLOCKED (the
  #16 merge put docker/ci/replay-lane/** on main). Run build-replay-lane-image.yml, take the digest,
  follow docker/ci/replay-lane/README.md §"Flipping the lane onto the image": point container.image
  at ghcr.io/sentientswarm/patrol-drone/replay-lane@sha256:<digest>, add the credentials block, and
  delete the two job-time apt steps. DoD: lane pulls the pinned image, zero job-time apt, stays green
  + required.
- SWM-16 (01-platform) + SWM-33 (02-mission-control) — Documentation + test consolidation true-up:
  reconcile PRD/Design/DoD/README against what actually shipped. Specs: docs/phase1/01-platform/ and
  docs/phase1/02-mission-control/ (design.md/prd.md/dod.md).
- SWM-14 — Finalize the ≤20-command README setup-to-running-mission budget across docsets (platform
  spine ≤12). Exit-checklist item #10.
- SWM-15 (01) + SWM-32 (02) — Grow e2e/integration test coverage beyond the two canonical missions.
- SWM-31 — Re-measure the SITL runtime/flakiness budget (OQ-5): replace the provisional ≤8 min/
  scenario figure with a MEASURED one and tune the quarantine threshold. NOTE: this needs real SITL
  timings; if the manual exit-checklist runs have not produced them yet, build the measurement
  harness and flag the numbers as the blocking input rather than inventing them.

Linear-only (not devloop work — just move the state):
- SWM-34 — UAT harness umbrella. All 6 sub-issues are Done; move it to Done.

## Source of truth
- docs/phase1_simulation_plan.md — the 12-item Phase 1 exit checklist (§"Phase 1 exit checklist").
- docs/phase1/README.md — exit-item → docset traceability.
- docs/pr16-review-loop-briefing.md — read §9 before opening any PR (the merge-wall rules below).

## Constraints (these bit hard on PR #16 — respect them)
- MERGE WALL: the `require-pr` ruleset makes the automated reviewer's approval structurally mandatory
  on this solo repo, and `dismiss_stale_reviews_on_push: true` nukes every standing approval on any
  push. So: batch real fixes into as few pushes as possible, and answer non-blocking/Low/"Follow-up"
  findings with a PR COMMENT, never a commit. (Get a ruleset bypass actor set before merge — that is
  on the human, note it in the PR body.)
- CodeScene 10.0 delta gate per changed file — run the self-check before pushing (no duplicated test
  blocks: parametrize/share fixtures; flatten nested conditionals). ruff line-length 100, py312.
- Unit suite stays <5 s and green; bag-producing changes get a replay regression.
- Branch off main (phase1/wrapup or per-group), PR into main. Commit/push only when I say.

## Linear mechanics
Key is in repo-root .env (not shell env): `set -a; . ./.env; set +a`. No Linear MCP — POST GraphQL
issueCreate/issueUpdate directly to https://api.linear.app/graphql with Authorization: $LINEAR_API_KEY.
SWM team 256341d4-8a99-4949-b9ce-03c04443550e; Done state 6938ea93-fe02-4ff4-977b-68bd03747032. See
the linear-project-mapping memory for project IDs.

## Out of scope for this loop (the human owns these)
The live/manual exit-checklist checks — full patrol via mission_patrol.launch.py, SITL abort
(low-battery + external), auto-upload+manifest, Foxglove render, container `docker compose build` +
in-container colcon. Devloop should assume those are run separately and may feed SWM-31.

Start with orient/scope: confirm the issue list against Linear live, then propose a batching plan
(suggest SWM-84 first as a self-contained quick win, SWM-34 as an immediate close) before executing.
