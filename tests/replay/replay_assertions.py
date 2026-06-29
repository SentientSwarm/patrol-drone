"""The ROS-free replay assertion comparator (docset 05-logging-replay, M8 / T8.8, SWM-80).

The replay regression test (LR-5) plays a reference bag, counts messages per topic over the play
window, then asserts the result against a curated subset + rate band (design §4.2.5, OQ-5). This
module holds the *comparison* — pure data in, pass/fail + reasons out — so it is host- and ROS-free
and unit-tested directly. The ``ros2 bag play`` + subscribe half that produces the observed counts
lives in ``test_replay_regression.py`` (the ROS CI lane).

Two checks per asserted topic:
  * presence/count — observed count must meet ``min_count`` (a missing topic is count 0 → fail; this
    is what makes the deliberate-break self-check bite).
  * rate (optional) — when ``expected_hz`` is set, observed mean rate must lie within ±``tol`` of it.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class AssertionSpec:
    """The expected shape of one asserted topic: a minimum count and an optional rate band."""

    topic: str
    min_count: int = 1
    expected_hz: float | None = None
    tol: float = 0.40  # ±40% default (OQ-5)


@dataclass(frozen=True)
class ObservedTopic:
    """What the replay actually saw for one topic: a message count over a play window."""

    topic: str
    count: int
    duration_s: float

    @property
    def hz(self) -> float:
        return self.count / self.duration_s if self.duration_s > 0 else 0.0


@dataclass(frozen=True)
class ReplayResult:
    """The verdict of evaluating a spec against the observed topics."""

    passed: bool
    failures: list[str] = field(default_factory=list)


def _check_topic(spec: AssertionSpec, observed: dict[str, ObservedTopic]) -> str | None:
    """Return a failure message for ``spec``, or None if it passes (presence + rate)."""
    seen = observed.get(spec.topic)
    if seen is None or seen.count < spec.min_count:
        got = "absent" if seen is None else f"count={seen.count}"
        return f"{spec.topic}: expected count >= {spec.min_count}, got {got}"

    if spec.expected_hz is not None:
        low = spec.expected_hz * (1 - spec.tol)
        high = spec.expected_hz * (1 + spec.tol)
        if not (low <= seen.hz <= high):
            return (
                f"{spec.topic}: rate {seen.hz:.2f} Hz outside "
                f"[{low:.2f}, {high:.2f}] (expected {spec.expected_hz} ±{spec.tol:.0%})"
            )
    return None


def evaluate(specs: list[AssertionSpec], observed: list[ObservedTopic]) -> ReplayResult:
    """Evaluate every asserted topic; pass iff none fails presence/count or its rate band."""
    by_topic = {o.topic: o for o in observed}
    failures = [msg for spec in specs if (msg := _check_topic(spec, by_topic)) is not None]
    return ReplayResult(passed=not failures, failures=failures)
