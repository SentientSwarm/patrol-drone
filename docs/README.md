# docs

Project documentation. The source of truth for architecture and rationale.

## Contents

- **[autonomous_drone_patrol_project_plan_v2.md](autonomous_drone_patrol_project_plan_v2.md)** — Master plan. Architecture, hardware BOMs, 8-phase development plan, reference implementations, distro/OS rationale.
- **[phase1_simulation_plan.md](phase1_simulation_plan.md)** — Executable plan for Phase 1 (pre-hardware simulation). Active working document.
- **[decisions/](decisions/)** — Architecture Decision Records (ADRs). Captured non-obvious technical decisions.

## When to update what

- **Master plan** — updated when the overall architecture, BOM, or phase plan changes. Versioned (v2 currently; bump when there's a significant rewrite).
- **Phase 1 plan** — updated as we learn things during Phase 1 execution. Living document until Phase 1 is complete; then it freezes and Phase 2's plan takes over as the active doc.
- **ADRs** — written when we make a non-obvious technical call. Once accepted, ADRs don't get edited (except for status changes). Superseded ADRs get a new ADR that supersedes them; the original stays for historical context.
