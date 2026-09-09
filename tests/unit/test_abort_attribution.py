"""Layer-A cover for the abort-attribution rules behind the SITL abort scenarios (F-09).

Both rules live beside the rclpy-importing ``patrol_acceptance`` module under ``tests/integration``;
they import nothing heavy, so this Layer-A test pulls them in via a path insert (the same bootstrap
``test_dwell_tracker`` uses).

Why they are pure: the two live abort guards fly an identical profile to the abort point, so
"an ABORT happened" is not evidence that *this* scenario's trigger caused it. The rules that turn
observations into attribution therefore have to be correct — and they were previously reachable only
through an SITL lane that cannot currently run.
"""

import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "tests" / "integration"))

from abort_attribution import (  # noqa: E402  (import after the path bootstrap)
    AbortAttribution,
    attribution_count_ok,
    latest_real_reason,
)


# The asymmetry is the whole point. EXACT at zero: the low-battery scenario publishes no
# /patrol/abort at all, so "none arrived" is the negative evidence that rules out the other live
# guard — relaxing it to `>=` would read `observed >= 0` and assert nothing. AT-LEAST above zero:
# a redelivered sample is not a defect, and strict equality on a count observed by the test's own
# subscriber is brittle without adding evidential weight.
@pytest.mark.parametrize(
    ("observed", "expected", "ok"),
    [
        pytest.param(0, 0, True, id="none-expected-none-seen"),
        pytest.param(1, 0, False, id="none-expected-one-seen-IS-the-negative-evidence"),
        pytest.param(3, 0, False, id="none-expected-several-seen"),
        pytest.param(1, 1, True, id="one-expected-one-seen"),
        pytest.param(2, 1, True, id="one-expected-redelivered-is-not-a-defect"),
        pytest.param(0, 1, False, id="one-expected-none-seen"),
    ],
)
def test_attribution_count_ok(observed: int, expected: int, ok: bool):
    assert attribution_count_ok(observed, expected) is ok


# The machine latches the cause with the ABORT transition and it sticks through RTH, so the last
# real reason is the answer — and a trailing "NONE" must never un-latch it.
@pytest.mark.parametrize(
    ("reasons", "expected"),
    [
        pytest.param([], "NONE", id="never-published"),
        pytest.param(["NONE"], "NONE", id="pre-abort-nominal-only"),
        pytest.param(["NONE", "LOW_BATTERY"], "LOW_BATTERY", id="latched-after-nominal"),
        pytest.param(
            ["NONE", "EXTERNAL_SIGNAL", "LOW_BATTERY"], "LOW_BATTERY", id="last-real-reason-wins"
        ),
        pytest.param(["LOW_BATTERY", "NONE"], "LOW_BATTERY", id="trailing-NONE-does-not-unlatch"),
    ],
)
def test_latest_real_reason(reasons: list[str], expected: str):
    assert latest_real_reason(reasons) == expected


# The scenarios construct this per-scenario; the default must be "this scenario published none",
# so a scenario that forgets to declare a count still asserts the strict negative direction.
def test_abort_attribution_defaults_to_expecting_no_external_commands():
    assert AbortAttribution(reason="LOW_BATTERY").external_cmds == 0
