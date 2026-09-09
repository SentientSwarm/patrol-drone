"""What a scenario expects the mission to say about *why* it aborted, and the pure rules behind it.

Split out of ``patrol_acceptance`` (which imports rclpy at module scope) so the decision rules are
Layer-A testable, matching ``dwell_tracker`` / ``home_settle_tracker``. ``patrol_acceptance``
re-exports :class:`AbortAttribution`, so both scenario files keep importing it from the old path.

Observing an ABORT is not evidence that YOUR trigger caused it: the external-signal and low-battery
guards are both live, both latch, and both fly an identical profile to the abort point (the two SITL
scenarios even take the same wall-clock). A scenario therefore has to pin the cause, not just the
recovery — and both rules below are how it does that without an SITL lane to run in.
"""

from __future__ import annotations

from dataclasses import dataclass

NO_ABORT_REASON = "NONE"


@dataclass(frozen=True)
class AbortAttribution:
    """What a scenario expects the mission to say about *why* it aborted (F-09)."""

    reason: str  # the AbortReason name expected on /patrol/abort_reason
    external_cmds: int = 0  # inbound /patrol/abort commands this scenario itself publishes


def attribution_count_ok(observed: int, expected: int) -> bool:
    """Whether the observed inbound-abort count is consistent with what this scenario published.

    EXACT when ``expected`` is zero — that is the negative evidence (the low-battery scenario
    publishes no /patrol/abort at all, so any inbound command means something else drove the
    transition) and loosening it to ``>=`` would make it vacuous. AT-LEAST otherwise: for the
    positive direction a redelivered sample is not a defect, and strict equality on a count observed
    by the test's own subscriber is brittle without adding evidential weight.
    """
    return observed == 0 if expected == 0 else observed >= expected


def latest_real_reason(reasons: list[str]) -> str:
    """The last non-``"NONE"`` abort reason in the observed sequence; ``"NONE"`` if there is none.

    The machine latches the cause with the ABORT transition and it sticks through RTH, so the last
    real reason is stable once an abort has fired. Pure, so the rule is Layer-A testable rather than
    reachable only through an SITL lane that cannot currently run.
    """
    real = [r for r in reasons if r != NO_ABORT_REASON]
    return real[-1] if real else NO_ABORT_REASON
