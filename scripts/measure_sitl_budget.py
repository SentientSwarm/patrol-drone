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


def sample_size_note(summaries: list[ScenarioSummary], budget: Budget) -> list[str]:
    """Say plainly when the sample is too small for the flake column to mean anything (F-08).

    The quarantine rule trips on a flake rate above ``flake_rate_threshold`` (1-in-5 by default), a
    ratio that N runs simply cannot express for small N: fed a single report, every scenario reports
    ``runs 1, fails 0, flake 0%`` — which looks like a clean measurement and is in fact no
    measurement at all. That is exactly how this ran for months, because the workflow handed the
    harness only the current night's JUnit while its own guidance said multiple reports must
    accumulate. The number is now stated, and an under-powered sample is labeled as such.
    """
    if not summaries:
        return []
    # The WEAKEST scenario, not the best-sampled one: parse_junit_xml drops <skipped> cases and the
    # rolling window spans scenario-set changes, so a scenario that ran on 2 of 10 nights sits beside
    # a 10-night sibling. max() would report 10 and certify the newcomer off its sibling's evidence —
    # the same false confidence, one level up, that this note exists to remove.
    runs = min(s.runs for s in summaries)
    min_runs = _min_runs_for_flake(budget)
    if runs >= min_runs:
        return ["", f"sample size: {runs} runs — flake rate is meaningful at this threshold."]
    return [
        "",
        f"SAMPLE SIZE {runs} run(s) — NOT a flake measurement. The quarantine rule trips at "
        f"flake > {budget.flake_rate_threshold:.0%} (1 in {min_runs}), which {runs} run(s) cannot "
        f"express: every flake figure above is necessarily 0% or 100%. Accumulate >= {min_runs} "
        f"nightly reports before reading the flake column, or acting on the budget line below.",
    ]


def _min_runs_for_flake(budget: Budget) -> int:
    """Runs needed before the flake rate can express the quarantine threshold (1/threshold)."""
    if budget.flake_rate_threshold <= 0:
        return 1
    return max(1, round(1 / budget.flake_rate_threshold))


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
    lines += sample_size_note(summaries, budget)
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
