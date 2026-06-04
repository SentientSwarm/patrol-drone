# Architecture Decision Records

Short documents that capture non-obvious technical decisions, their context, and consequences. Once accepted, ADRs don't get edited — superseded ADRs are replaced by new ADRs that reference them.

## Why ADRs

Months from now, somebody (probably us) will look at a settled call and ask "why did we do it that way?" The reasoning is the thing that ages best. ADRs preserve it.

## Format

Each ADR is numbered (`NNNN-short-title.md`), starts at `0001`, never reused. Use this template:

```markdown
# ADR-NNNN: <Short title>

**Status:** Proposed | Accepted | Superseded by ADR-XXXX | Deprecated
**Date:** YYYY-MM-DD
**Deciders:** <names>

## Context

What's the situation? What problem are we solving? What constraints are real?

## Decision

What did we decide? State it plainly.

## Consequences

What follows from this decision? Be honest about the negatives, not just the positives.

### Positive
### Negative
### Neutral

## Alternatives considered

What else did we look at? Why did we not pick those?
```

## Index

| # | Title | Status |
|---|---|---|
| [0001](0001-distro-and-os.md) | Ubuntu 24.04 + ROS 2 Jazzy + JetPack 7.2 | Accepted |
| [0002](0002-ci-architecture.md) | Two-layer CI with xenon complexity and 85% coverage gates | Accepted |
| [0003](0003-phase1-bootstrap-scope.md) | `setup_phase1.sh` provisions the full Phase 1 toolchain, not just M1 | Accepted |

## When to write an ADR

When the decision is non-obvious, hard to reverse, or you can imagine someone reasonably asking "why?" later. If the answer to "should this be an ADR?" is "I'm not sure," write it. The bar is low; the format is short.

What's NOT an ADR-worthy decision:
- Code style choices (use a linter config)
- Library versions for small/swap-able deps
- Naming conventions

What IS:
- OS / distribution / language version
- Architectural patterns (sync vs async, monolith vs services)
- Major framework or library choices (state machine library, ORM, etc.)
- Data formats and schemas
- Deployment topology
- Anything where future-you will read the decision and need the *why*
