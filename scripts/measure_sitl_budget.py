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
import math
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
    # Appended with a default so the positional Budget(a, b, c) constructions in the Layer-A suite
    # keep working. Defaults TRUE: a config that omits the key must not silently claim to be
    # measured — the whole point of the flag is that unmeasured is the honest state.
    provisional: bool = True


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
    # The max over PASSING runs only, or None when the scenario never once completed. Appended with a
    # default so positional construction stays valid. max_seconds deliberately keeps counting
    # failures — over_budget/quarantine are CAPACITY questions, where a scenario that burns 300s
    # hitting PATROL_TIMEOUT_S genuinely consumed 300s of the lane. Only the fold-this-back
    # RECOMMENDATION is restricted, because that number is a claim about how long the scenario takes.
    max_passing_seconds: float | None = None


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
        scenario,
        runs,
        failures,
        max_s,
        mean_s,
        flake_rate,
        over_budget,
        quarantine,
        max((c.seconds for c in cases if c.passed), default=None),
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
    # Every rate this sample can express is a multiple of 1/runs, and 1/runs > threshold for EVERY
    # runs < min_runs (min_runs = ceil(1/threshold) >= 1/threshold, so runs <= min_runs - 1
    # < 1/threshold) — i.e. in this branch a SINGLE failure trips quarantine. Say exactly that, and
    # say it of the WEAKEST-sampled scenario, which is what `runs` is (min across summaries). The
    # old "cannot express / necessarily 0% or 100%" was true only at runs == 1 and was visibly false
    # one line under the table for any larger N; a warning caught lying about something the reader
    # can check is a warning they discount.
    step = 1 / runs
    return [
        "",
        f"SAMPLE SIZE {runs} run(s) — NOT a flake measurement. The quarantine rule trips at "
        f"flake > {budget.flake_rate_threshold:.0%} (1 in {min_runs}), but the weakest-sampled "
        f"scenario has only {runs} run(s), which resolve its rate to the nearest {step:.0%}: one "
        f"failure there reads as {step:.0%} and trips it, zero failures read as a clean 0%. "
        f"Accumulate >= {min_runs} nightly reports before reading the flake column, or acting on "
        f"the budget line below.",
    ]


def _min_runs_for_flake(budget: Budget) -> int:
    """Runs needed before ONE failure stops tripping quarantine: ``ceil(1 / threshold)``.

    ``ceil``, not ``round``: the bar this returns is the sample size at which the note above stops
    warning, so it must satisfy ``1 / min_runs <= threshold``. ``round`` rounds DOWN whenever
    ``1/threshold`` has a fractional part below .5 and certifies a sample where a single failure
    still quarantines — at threshold 0.3 it returns 3, and 1-in-3 is 33% > 30%; at 0.4 it returns 2,
    and 1-in-2 is 50% > 40%. Inert at the shipped 0.2 (1/5 = 20% is not > 0.2), live the moment
    SWM-31's stated purpose — tuning the threshold — is carried out.
    """
    if budget.flake_rate_threshold <= 0:
        return 1
    return max(1, math.ceil(1 / budget.flake_rate_threshold))


def _read_run(path: Path) -> tuple[list[CaseResult], str | None]:
    """One run report -> (results, skip note). A corrupt report is SKIPPED, not fatal.

    The asymmetry with :func:`load_budget` is deliberate — do not "unify" them. A bad *budget* is a
    bad rule and must fail loud (pinned by ``test_load_budget_fails_loudly_on_a_malformed_config``).
    A bad *run report* is one night of a rolling window the nightly already fetches best-effort: a
    truncated or 0-byte JUnit is what an interrupted upload leaves behind, and discarding the other
    nine nights over it reports nothing where it could report nine. A path that does not exist is a
    third thing — an argv error, not a window artifact — so it still raises.
    """
    if not path.is_file():
        raise FileNotFoundError(f"JUnit report not found: {path}")
    try:
        return parse_junit_xml(path.read_text()), None
    except (OSError, ValueError, ElementTree.ParseError) as exc:
        return [], f"{path.name}: {type(exc).__name__}: {exc}"


def _read_runs(paths: list[Path]) -> tuple[list[CaseResult], list[str]]:
    """Read every supplied run report into (results, skip notes) — see :func:`_read_run`."""
    results: list[CaseResult] = []
    skipped: list[str] = []
    for path in paths:
        run_results, note = _read_run(path)
        results.extend(run_results)
        if note:
            skipped.append(note)
    return results, skipped


def _skipped_note(skipped: list[str] | None) -> list[str]:
    """Name every report that could not be read. A SILENTLY dropped file would recreate the exact
    hazard :func:`sample_size_note` exists to remove: a report that looks like N nights of evidence
    while it measures fewer."""
    if not skipped:
        return []
    return [
        "",
        f"SKIPPED {len(skipped)} unreadable run report(s) — NOT counted in the sample below:",
        *[f"  - {s}" for s in skipped],
    ]


def _header_line(budget: Budget) -> str:
    """The budget header, carrying the provisional flag the config documents as THE mechanism.

    ``provisional:`` was read by nothing: flipping it changed no behaviour and no output, while five
    places (this module's docstring, the config's own comments, 02's design.md) advertise the flip as
    how a measured figure supersedes the provisional one. Worse, the only line that used the word at
    all was the EMPTY-table line — so the status was disclosed exactly when there was no measurement
    to confuse it with, and hidden the moment there was one. It belongs in the header, next to the
    number it qualifies.
    """
    status = "PROVISIONAL — not yet measured" if budget.provisional else "measured"
    return (
        f"SITL budget: {budget.per_scenario_budget_s:.0f}s/scenario ({status}; "
        f"quarantine: flake > {budget.flake_rate_threshold:.0%} "
        f"or max > {budget.budget_overrun_factor:.0f}x budget)"
    )


def _no_measurements_line(skipped: list[str] | None) -> str:
    """The empty-table line, which must not call an all-corrupt window an empty one."""
    if skipped:
        return (
            "(no READABLE run reports — every supplied report was skipped above; no measurements)"
        )
    # No longer says "provisional" itself: the header above is now the authoritative label and reads
    # the config, so a second hardcoded claim here would be wrong the day provisional flips false.
    return "(no run reports supplied — no measurements yet; the budget above stands)"


def _failed_runs_note(summaries: list[ScenarioSummary]) -> list[str]:
    """Say that max_s/mean_s include failed runs, where that could be misread.

    A scenario timing out at ``PATROL_TIMEOUT_S`` every night reads ``mean_s 300.0`` for a run that
    has never completed. The columns stay all-runs on purpose (see :class:`ScenarioSummary`), so they
    and the passing-only recommendation disagree BY DESIGN — name the disagreement rather than
    leaving the reader to discover it and distrust both numbers.
    """
    if not any(s.failures for s in summaries):
        return []
    return [
        "",
        "note: max_s/mean_s include FAILED runs (a timeout contributes its full wall clock); the "
        "measured-budget recommendation, where one is offered, is over passing runs only.",
    ]


def _measured_budget_line(summaries: list[ScenarioSummary]) -> list[str]:
    """The fold-this-back recommendation, over PASSING runs only.

    Taken over all cases, a scenario failing at its 300s timeout recommended 300.0s as the measured
    per-scenario budget — a measurement of the timeout constant, not of the scenario — and once the
    window clears ``min_runs`` the sample-size caveat correctly stops firing, so nothing on the page
    hedged it. The line now says what it excludes, and declines to print a number at all when there
    is no passing run anywhere to derive one from.
    """
    measured = [s.max_passing_seconds for s in summaries if s.max_passing_seconds is not None]
    if not measured:
        return ["", "no measured per_scenario_budget_s: no scenario has a passing run to measure."]
    no_pass = len(summaries) - len(measured)
    excluded = f" ({no_pass} scenario(s) had no passing run)" if no_pass else ""
    return [
        "",
        f"measured per_scenario_budget_s (observed max across PASSING runs only){excluded}: "
        f"{max(measured):.1f}s — fold this into tests/integration/sitl_budget.yaml + flip "
        f"provisional:false once stable.",
    ]


def format_report(
    summaries: list[ScenarioSummary], budget: Budget, skipped: list[str] | None = None
) -> str:
    """A human/Markdown-friendly table plus the measured-budget line to fold back into the config."""
    lines = [
        _header_line(budget),
        *_skipped_note(skipped),
        "",
        f"{'scenario':<64} {'runs':>4} {'fails':>5} {'flake':>6} {'max_s':>7} {'mean_s':>7}  verdict",
    ]
    if not summaries:
        lines.append(_no_measurements_line(skipped))
        return "\n".join(lines)
    for s in summaries:
        verdict = "QUARANTINE" if s.quarantine else ("OVER-BUDGET" if s.over_budget else "ok")
        lines.append(
            f"{s.scenario:<64} {s.runs:>4} {s.failures:>5} {s.flake_rate:>5.0%} "
            f"{s.max_seconds:>7.1f} {s.mean_seconds:>7.1f}  {verdict}"
        )
    lines += sample_size_note(summaries, budget)
    lines += _failed_runs_note(summaries)
    lines += _measured_budget_line(summaries)
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
        provisional=bool(data.get("provisional", True)),
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
    results, skipped = _read_runs(args.junit)
    summaries = summarize(results, budget)
    print(format_report(summaries, budget, skipped))

    if args.fail_on_quarantine and any(s.quarantine for s in summaries):
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
