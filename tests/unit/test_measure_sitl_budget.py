"""Unit tests for scripts/measure_sitl_budget.py — the SITL budget/flakiness harness (SWM-31).

Exercises the pure core (parse_junit_xml / summarize / format_report) with in-memory data, plus the
thin I/O shell (load_budget / main) against tmp_path files — no SITL and no pytest subprocess, so it
still runs in the fast Layer-A tier. The module is loaded by path because scripts/ is intentionally
not a Python package (mirrors test_manifest_drift.py).
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


# The headline sample size must be the WEAKEST scenario, not the best-sampled one. parse_junit_xml
# drops <skipped> cases and the rolling window spans scenario-set changes, so a brand-new scenario
# can sit at 2 nights beside a 10-night sibling. Reporting 10 would certify the newcomer off its
# sibling's evidence — exactly the false confidence F-08 exists to remove.
def test_sample_size_note_reports_the_weakest_scenario_not_the_best():
    results = [msb.CaseResult("well_sampled", 10.0, True) for _ in range(10)]
    results += [msb.CaseResult("brand_new", 10.0, True) for _ in range(2)]

    note = "\n".join(msb.sample_size_note(msb.summarize(results, _budget()), _budget()))

    assert "2 run(s)" in note, "the headline must be the weakest scenario's sample"
    assert "10 runs" not in note
    assert "NOT a flake measurement" in note


# sample_size_note is module-public and directly unit-tested, so it must be total rather than relying
# on format_report's own empty guard staying in place above the call.
def test_sample_size_note_on_no_summaries_returns_nothing_and_does_not_raise():
    assert msb.sample_size_note([], _budget()) == []


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


# --- the thin I/O shell (F-04) --------------------------------------------------
# The pure core above is thoroughly covered; the failure modes live in the layer that touches files.
# Shared builders (not copied literals) keep this block CodeScene-clean.

_BUDGET_YAML = """per_scenario_budget_s: 100.0
quarantine:
  flake_rate_threshold: 0.2
  budget_overrun_factor: 2.0
"""


def _write_budget_yaml(tmp_path: Path, yaml_text: str) -> Path:
    """The one place a budget config reaches disk (shared by the fixture and the load_budget tests)."""
    path = tmp_path / "sitl_budget.yaml"
    path.write_text(yaml_text)
    return path


@pytest.fixture
def budget_file(tmp_path: Path) -> Path:
    return _write_budget_yaml(tmp_path, _BUDGET_YAML)


def _one_case_junit(seconds: float, *, passed: bool) -> str:
    failure = "" if passed else '<failure message="boom"/>'
    return (
        '<?xml version="1.0" encoding="utf-8"?><testsuites><testsuite name="pytest">'
        f'<testcase classname="tests.integration.test_x" name="test_p" time="{seconds}">'
        f"{failure}</testcase></testsuite></testsuites>"
    )


def _write_runs(tmp_path: Path, outcomes: list[bool]) -> list[str]:
    """One JUnit XML per run (the harness's real argv shape); returns the paths."""
    paths = []
    for i, passed in enumerate(outcomes):
        path = tmp_path / f"run-{i}.xml"
        path.write_text(_one_case_junit(10.0, passed=passed))
        paths.append(str(path))
    return paths


def test_main_reads_the_files_reports_and_exits_zero(tmp_path: Path, budget_file: Path, capsys):
    rc = msb.main([*_write_runs(tmp_path, [True] * 5), "--budget", str(budget_file)])

    out = capsys.readouterr().out
    assert rc == 0
    assert "tests.integration.test_x::test_p" in out
    assert "flake rate is meaningful" in out  # 5 runs clears the 0.2 threshold
    assert "observed max across scenarios): 10.0s" in out


def test_main_with_no_reports_prints_the_provisional_budget(budget_file: Path, capsys):
    rc = msb.main(["--budget", str(budget_file)])

    assert rc == 0
    assert "no run reports supplied" in capsys.readouterr().out


# --fail-on-quarantine is the flag that would let this non-gating harness become a gate later;
# nothing proved it returns 1. Same sample, both dispositions — parametrized, not duplicated.
@pytest.mark.parametrize(
    ("flag", "expected_rc"),
    [([], 0), (["--fail-on-quarantine"], 1)],
    ids=["report-only-is-non-gating", "flag-gates"],
)
def test_main_exits_non_zero_on_quarantine_only_with_the_flag(
    tmp_path: Path, budget_file: Path, capsys, flag: list[str], expected_rc: int
):
    # 2 failures in 5 runs = 40% flake, over the 20% quarantine threshold.
    runs = _write_runs(tmp_path, [False, False, True, True, True])

    rc = msb.main([*flag, *runs, "--budget", str(budget_file)])

    assert rc == expected_rc
    assert "QUARANTINE" in capsys.readouterr().out


@pytest.mark.parametrize(
    ("yaml_text", "expected"),
    [
        pytest.param(_BUDGET_YAML, (100.0, 0.2, 2.0), id="explicit-quarantine-block"),
        pytest.param("per_scenario_budget_s: 480\n", (480.0, 0.2, 2.0), id="defaults-when-absent"),
        pytest.param(
            "per_scenario_budget_s: 480\nquarantine:\n  flake_rate_threshold: 0.1\n",
            (480.0, 0.1, 2.0),
            id="partial-block-keeps-the-other-default",
        ),
    ],
)
def test_load_budget_reads_thresholds_with_documented_defaults(
    tmp_path: Path, yaml_text: str, expected: tuple[float, float, float]
):
    budget = msb.load_budget(_write_budget_yaml(tmp_path, yaml_text))

    assert (
        budget.per_scenario_budget_s,
        budget.flake_rate_threshold,
        budget.budget_overrun_factor,
    ) == expected


# The nightly runs this harness under `|| true` (deliberately non-gating), so a SILENTLY broken
# harness would produce the same job-summary shape as a night with nothing to measure. The chosen
# behaviour is fail-loud — a traceback, which the workflow now routes into the job summary — not a
# swallowed error. Pin that choice so nobody "helpfully" adds a bare except later.
@pytest.mark.parametrize(
    ("yaml_text", "expected_exc", "match"),
    [
        pytest.param("quarantine: {}\n", KeyError, "per_scenario_budget_s", id="missing-key"),
        pytest.param(
            "per_scenario_budget_s: not-a-number\n", ValueError, "not-a-number", id="non-numeric"
        ),
    ],
)
def test_load_budget_fails_loudly_on_a_malformed_config(
    tmp_path: Path, yaml_text: str, expected_exc: type[Exception], match: str
):
    path = _write_budget_yaml(tmp_path, yaml_text)

    with pytest.raises(expected_exc, match=match):
        msb.load_budget(path)


def test_main_fails_loudly_on_a_missing_junit_path(tmp_path: Path, budget_file: Path):
    with pytest.raises(FileNotFoundError):
        msb.main([str(tmp_path / "absent.xml"), "--budget", str(budget_file)])


# --- F-01: the note must not contradict the table it annotates ------------------


def test_sample_size_note_does_not_contradict_the_table_it_annotates():
    # 4 runs, 1 failure: the table prints 25%, which CLEARS the 20% trip point. The old wording
    # ("cannot express" / "necessarily 0% or 100%") was true only at runs == 1 and was visibly false
    # one line below the table. The note must state the real hazard — at this sample a single
    # failure IS the whole rate — and quote the same figure the table shows.
    results = [msb.CaseResult("S", 10.0, i > 0) for i in range(4)]
    (summary,) = msb.summarize(results, _budget())
    assert summary.flake_rate == pytest.approx(0.25)

    note = "\n".join(msb.sample_size_note([summary], _budget()))

    assert "cannot express" not in note
    assert "0% or 100%" not in note
    assert "25%" in note
    assert "NOT a flake measurement" in note  # the warning itself still fires
