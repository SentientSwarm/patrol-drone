# tests — Test infrastructure

Three tiers. Different fidelity, different runtime, different purpose.

## `unit/` — Fast, mock-everything (target: <5s total)

Plain pytest tests. No ROS, no Gazebo, no PX4. Tests the mission state machine, message construction, config parsing, waypoint sequencing, abort logic.

The mission state machine is the most important thing to cover here. Aim for >80% coverage of state transitions.

Run on every commit (pre-commit hook + CI).

## `integration/` — Real SITL (target: a few minutes per test)

Spin up PX4 SITL + ROS 2 nodes via a test harness, run a canonical mission, assert on resulting state and bag contents. Slow and easier to write flakily — keep the count small and the assertions strict.

A handful of canonical scenarios is enough. Run in CI but tolerate longer runtimes.

## `replay/` — Deterministic bag replay (target: seconds per test)

Take a known-good bag (committed to the repo or fetched from artifact storage), replay it through perception nodes, assert outputs match a saved reference. Catches regressions in detection or mission logic without needing the simulator.

Build this as soon as Milestone M7 produces bags.

## Conventions

- pytest for everything (works for both unit and integration via plugins like `pytest-launch`).
- Test data and reference outputs in `tests/data/` (create when needed).
- Don't mock the simulator. If a test needs flight dynamics, use real SITL. Simulator-of-a-simulator is a bad trade.

See the Phase 1 plan, "Test strategy" section, for the full rationale.
