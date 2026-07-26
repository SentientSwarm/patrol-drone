"""Unit tests for scripts/measure_sitl_budget.py — the SITL budget/flakiness harness (SWM-31).

Exercises the pure core (parse_junit_xml / summarize / format_report) with in-memory data — no SITL,
no pytest subprocess, no yaml/files — so it runs in the fast Layer-A tier. The module is loaded by
path because scripts/ is intentionally not a Python package (mirrors test_manifest_drift.py).
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from typing import Any

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
_MODULE_PATH = REPO_ROOT / "scripts" / "measure_sitl_budget.py"


def _load() -> Any:
    spec = importlib.util.spec_from_file_location("measure_sitl_budget", _MODULE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register before exec: the module's frozen dataclasses use `from __future__ import annotations`,
    # and dataclasses resolves __module__ via sys.modules during class creation (None -> crash).
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


msb = _load()

_JUNIT = """<?xml version="1.0" encoding="utf-8"?>
<testsuites><testsuite name="pytest" tests="3">
  <testcase classname="tests.integration.test_mission_patrol" name="test_nominal" time="42.5"/>
  <testcase classname="tests.integration.test_mission_patrol" name="test_abort" time="30.0">
    <failure message="boom"/>
  </testcase>
  <testcase classname="tests.integration.test_fmu_bridge_surface" name="test_surface" time="11.0">
    <skipped/>
  </testcase>
</testsuite></testsuites>"""


def _budget(budget_s: float = 100.0) -> Any:
    return msb.Budget(
        per_scenario_budget_s=budget_s, flake_rate_threshold=0.2, budget_overrun_factor=2.0
    )


def test_parse_junit_excludes_skipped_and_reads_pass_fail_time():
    results = msb.parse_junit_xml(_JUNIT)
    # the skipped case is dropped; the other two parse with their time + pass/fail
    assert len(results) == 2
    by_scenario = {r.scenario: r for r in results}
    nominal = by_scenario["tests.integration.test_mission_patrol::test_nominal"]
    abort = by_scenario["tests.integration.test_mission_patrol::test_abort"]
    assert nominal.passed is True
    assert nominal.seconds == pytest.approx(42.5)
    assert abort.passed is False
    assert abort.seconds == pytest.approx(30.0)


def test_summarize_aggregates_runs_max_mean_and_flake():
    # one scenario across 5 runs: 1 failure, times 10..50
    results = [
        msb.CaseResult("S", secs, passed)
        for secs, passed in [(10.0, True), (20.0, True), (30.0, True), (40.0, True), (50.0, False)]
    ]
    (summary,) = msb.summarize(results, _budget())
    assert summary.runs == 5
    assert summary.failures == 1
    assert summary.max_seconds == pytest.approx(50.0)
    assert summary.mean_seconds == pytest.approx(30.0)
    assert summary.flake_rate == pytest.approx(0.2)


# inputs (fail count over 5 runs, max_seconds, budget_s) -> expected (over_budget, quarantine)
_QUARANTINE_CASES = [
    pytest.param((1, 50.0, 100.0), (False, False), id="clean-within-budget"),
    pytest.param((2, 50.0, 100.0), (False, True), id="flaky-40pct->quarantine"),
    pytest.param((1, 150.0, 100.0), (True, False), id="over-budget-but-not-2x"),
    pytest.param((1, 250.0, 100.0), (True, True), id="over-2x-budget->quarantine"),
]


@pytest.mark.parametrize(("inputs", "expected"), _QUARANTINE_CASES)
def test_over_budget_and_quarantine_thresholds(inputs, expected):
    fails, max_s, budget_s = inputs
    over_budget, quarantine = expected
    # 5 runs for one scenario: `fails` failures among the first four, the fifth carries max_s (passing)
    results = [msb.CaseResult("S", 10.0, i >= fails) for i in range(4)]
    results.append(msb.CaseResult("S", max_s, True))
    (summary,) = msb.summarize(results, _budget(budget_s))
    assert summary.over_budget is over_budget
    assert summary.quarantine is quarantine


def test_format_report_empty_notes_no_measurements():
    report = msb.format_report([], _budget())
    assert "no run reports supplied" in report
    assert (
        "100s/scenario" in report
    )  # the header shows the config budget, not an invented measurement


def test_format_report_flags_quarantine_and_measured_max():
    results = msb.parse_junit_xml(_JUNIT)  # nominal ok (42.5), abort failed (30.0)
    summaries = msb.summarize(results, _budget(budget_s=100.0))
    report = msb.format_report(summaries, _budget(budget_s=100.0))
    assert "QUARANTINE" in report  # the single-run failure is a 100% flake rate
    assert "observed max across scenarios): 42.5s" in report


# --- sample-size honesty (F-08) -------------------------------------------------
# Fed one report, every scenario reads `runs 1, fails 0, flake 0%` — which looks like a clean
# measurement and is no measurement at all. The workflow handed the harness exactly one report for
# months while its own guidance said multiple nights must accumulate. The report now says so.


@pytest.mark.parametrize(
    ("runs", "under_powered"),
    [(1, True), (4, True), (5, False), (12, False)],
    ids=["single-run", "just-under", "at-threshold", "well-powered"],
)
def test_sample_size_note_labels_an_under_powered_sample(runs: int, under_powered: bool):
    # A clean scenario across `runs` runs; at a 0.2 threshold the flake rate needs 5 to be expressible.
    results = [msb.CaseResult("S", 10.0, True) for _ in range(runs)]
    summaries = msb.summarize(results, _budget())

    note = "\n".join(msb.sample_size_note(summaries, _budget()))

    assert ("NOT a flake measurement" in note) is under_powered
    assert f"{runs} run" in note


def test_format_report_carries_the_sample_size_warning_for_one_run():
    summaries = msb.summarize(msb.parse_junit_xml(_JUNIT), _budget())

    report = msb.format_report(summaries, _budget())

    # The exact failure mode from the live 2026-07-26 run: a 1-sample verdict presented as measured.
    assert "NOT a flake measurement" in report
    assert "1 in 5" in report


def test_min_runs_for_flake_tracks_the_configured_threshold():
    # The needed sample size is derived from the quarantine threshold, not hard-coded, so retuning
    # the rule keeps the warning honest.
    assert msb._min_runs_for_flake(msb.Budget(100.0, 0.2, 2.0)) == 5
    assert msb._min_runs_for_flake(msb.Budget(100.0, 0.1, 2.0)) == 10
    assert msb._min_runs_for_flake(msb.Budget(100.0, 0.0, 2.0)) == 1  # no division by zero
