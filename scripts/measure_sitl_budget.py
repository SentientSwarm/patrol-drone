#!/usr/bin/env python3
"""Measure the SITL runtime/flakiness budget from nightly JUnit results (OQ-5, SWM-31).

Design OQ-5 fixed the SITL budget as a **provisional** ≤8 min/scenario with a quarantine-not-expand
rule (>1-in-5 flake rate or >2x budget overrun), explicitly deferring the *measured* wall-clock
figure to "once 01's landed SITL" (MZ.1). This harness is the deferred measurement: it reads one or
more pytest JUnit XML reports (each = one nightly SITL run) and reports, per scenario, the measured
wall-clock (max + mean), the flake rate across runs, and whether the scenario is over budget or
quarantine-worthy per the thresholds in the budget config.

**It does not invent the number.** With no run reports it prints the provisional budget and reports
zero measurements; the measured figure is produced only when fed real nightly JUnit XML (the
human-owned live runs are the blocking input). Once enough runs exist, fold the observed max back
into ``per_scenario_budget_s`` and flip ``provisional: false`` in the budget config.

Split into a pure core (``parse_junit_xml`` / ``summarize`` / ``format_report`` — plain data in,
plain data out, unit-tested in Layer-A with no SITL and no pytest run) and a thin ``main`` that reads
the YAML budget + XML files. Mirrors the repo's ROS-free-core + thin-I/O-wrapper convention.

Usage:
    scripts/measure_sitl_budget.py RUN1.xml [RUN2.xml ...] [--budget tests/integration/sitl_budget.yaml]
    scripts/measure_sitl_budget.py --fail-on-quarantine RUN*.xml   # optional non-zero exit on quarantine
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_BUDGET = REPO_ROOT / "tests" / "integration" / "sitl_budget.yaml"


@dataclass(frozen=True)
class CaseResult:
    """One testcase from one run: its scenario id, wall-clock seconds, and pass/fail."""

    scenario: str
    seconds: float
    passed: bool


@dataclass(frozen=True)
class Budget:
    """The budget thresholds (from the config), externalized from design prose so this is data."""

    per_scenario_budget_s: float
    flake_rate_threshold: float
    budget_overrun_factor: float


@dataclass(frozen=True)
class ScenarioSummary:
    """The measured verdict for one scenario across all supplied runs."""

    scenario: str
    runs: int
    failures: int
    max_seconds: float
    mean_seconds: float
    flake_rate: float
    over_budget: bool
    quarantine: bool


def parse_junit_xml(xml_text: str) -> list[CaseResult]:
    """Parse pytest JUnit XML text into per-testcase results (skipped cases are excluded).

    A ``<testcase>`` with a ``<failure>`` or ``<error>`` child did not pass; ``<skipped>`` means the
    scenario did not run this time, so it is dropped rather than counted as a passing run.
    """
    root = ElementTree.fromstring(xml_text)
    results: list[CaseResult] = []
    for case in root.iter("testcase"):
        if case.find("skipped") is not None:
            continue
        classname = case.get("classname", "")
        name = case.get("name", "")
        scenario = f"{classname}::{name}" if classname else name
        passed = case.find("failure") is None and case.find("error") is None
        results.append(CaseResult(scenario, float(case.get("time", "0") or 0.0), passed))
    return results


def _summarize_one(scenario: str, cases: list[CaseResult], budget: Budget) -> ScenarioSummary:
    runs = len(cases)
    failures = sum(1 for c in cases if not c.passed)
    seconds = [c.seconds for c in cases]
    max_s = max(seconds)
    mean_s = sum(seconds) / runs
    flake_rate = failures / runs
    over_budget = max_s > budget.per_scenario_budget_s
    quarantine = (
        flake_rate > budget.flake_rate_threshold
        or max_s > budget.per_scenario_budget_s * budget.budget_overrun_factor
    )
    return ScenarioSummary(
        scenario, runs, failures, max_s, mean_s, flake_rate, over_budget, quarantine
    )


def summarize(results: list[CaseResult], budget: Budget) -> list[ScenarioSummary]:
    """Group per-scenario results across runs into measured summaries, sorted by scenario name."""
    by_scenario: dict[str, list[CaseResult]] = {}
    for r in results:
        by_scenario.setdefault(r.scenario, []).append(r)
    return [_summarize_one(s, cases, budget) for s, cases in sorted(by_scenario.items())]


def format_report(summaries: list[ScenarioSummary], budget: Budget) -> str:
    """A human/Markdown-friendly table plus the measured-budget line to fold back into the config."""
    lines = [
        f"SITL budget: {budget.per_scenario_budget_s:.0f}s/scenario "
        f"(quarantine: flake > {budget.flake_rate_threshold:.0%} "
        f"or max > {budget.budget_overrun_factor:.0f}x budget)",
        "",
        f"{'scenario':<64} {'runs':>4} {'fails':>5} {'flake':>6} {'max_s':>7} {'mean_s':>7}  verdict",
    ]
    if not summaries:
        lines.append("(no run reports supplied — no measurements yet; provisional budget stands)")
        return "\n".join(lines)
    for s in summaries:
        verdict = "QUARANTINE" if s.quarantine else ("OVER-BUDGET" if s.over_budget else "ok")
        lines.append(
            f"{s.scenario:<64} {s.runs:>4} {s.failures:>5} {s.flake_rate:>5.0%} "
            f"{s.max_seconds:>7.1f} {s.mean_seconds:>7.1f}  {verdict}"
        )
    observed_max = max(s.max_seconds for s in summaries)
    lines += [
        "",
        f"measured per_scenario_budget_s (observed max across scenarios): {observed_max:.1f}s "
        f"— fold this into tests/integration/sitl_budget.yaml + flip provisional:false once stable.",
    ]
    return "\n".join(lines)


def load_budget(path: Path) -> Budget:
    """Read the budget config (YAML) into a Budget. Only the thin I/O layer touches files/yaml; the
    pure core (parse/summarize/format) takes plain data so it stays trivially unit-testable."""
    data = yaml.safe_load(path.read_text())
    q = data.get("quarantine", {})
    return Budget(
        per_scenario_budget_s=float(data["per_scenario_budget_s"]),
        flake_rate_threshold=float(q.get("flake_rate_threshold", 0.2)),
        budget_overrun_factor=float(q.get("budget_overrun_factor", 2.0)),
    )


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "junit", nargs="*", type=Path, help="pytest JUnit XML report(s); one per run"
    )
    parser.add_argument("--budget", type=Path, default=DEFAULT_BUDGET, help="budget config YAML")
    parser.add_argument(
        "--fail-on-quarantine",
        action="store_true",
        help="exit non-zero if any scenario is quarantine-worthy (default: report only)",
    )
    args = parser.parse_args(argv)

    budget = load_budget(args.budget)
    results: list[CaseResult] = []
    for path in args.junit:
        results.extend(parse_junit_xml(path.read_text()))
    summaries = summarize(results, budget)
    print(format_report(summaries, budget))

    if args.fail_on_quarantine and any(s.quarantine for s in summaries):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
