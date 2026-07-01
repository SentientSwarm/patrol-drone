"""RTF-robust rate analysis for the live-bag e2e witness (docset 05-logging-replay, M8 / LR-8).

The CI replay lane (``test_replay_regression.py``) trusts ``count / bag-info-duration`` because the
checked-in reference bag was recorded at RTF ≈ 1.0 with a clean, self-consistent MCAP. A *live* bag
recorded on a GUI-loaded host is not so trustworthy: the sim sags below real-time and the record path
can double-deliver rendered frames while the MCAP summary is written inconsistently (``ros2 bag info``
count ≠ the actual message-stream row count). On such a bag ``count / bag-info-duration`` over-reads
(observed: camera 30.3 Hz vs a true 15.15 Hz) and trips the rate band with a false failure.

This module is the ROS-free half of the fix (mirrors ``replay_assertions.py``): pure functions the
witness feeds with per-topic samples it read from the bag. It does two jobs:

  * measure the TRUE publish rate from a topic's own message timestamps **de-duplicated to unique
    stamps** — which is both RTF-invariant and immune to duplicate-frame inflation; and
  * a **consistency guard** (:func:`consistency_verdict`) that fails loud when a bag is demonstrably
    untrustworthy, so the witness never reports a rate PASS on a suspect artifact.

Two independent bad-bag signals are checked, because the offending GUI bag exhibits both and each on
its own is a different key (measured directly):

  * ``info_count != reader_rows`` — the ``ros2 bag info`` summary disagrees with how many messages
    the reader actually yields (an inconsistent / non-finalized MCAP). This is the reliable, type-
    agnostic signal (offending bag: camera 9037 vs 9335, mission 5957 vs 6185, all mismatch).
  * per-topic ``dup_factor`` on the **sim-time header stamp** — each rendered frame's sim-stamp
    reappearing with a distinct receive time (offending camera: 9335 rows / 4537 unique sim-stamps =
    2.06). Only meaningful for header-bearing types; header-less/px4 types pass ``sim_stamps_ns=[]``.

The true **rate** is measured from the recorder ``log_time`` (``rate_stamps_ns``) rather than the
sim stamp, because on a *consistent* bag log_time is clean and monotonic for every type (verified:
reference bag camera 15.23 / mission 10.05 / vlp 49.97 Hz) whereas per-type sim-stamp CDR parsing is
fragile (px4 has no std Header, ``/tf`` leads with an array, a downsampled camera has sparse stamps).

The band check itself is reused verbatim from ``replay_assertions.evaluate`` — this module never
re-implements the ±tol logic.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from replay_assertions import AssertionSpec, ObservedTopic, ReplayResult, evaluate

# rows ÷ unique-stamps above this = the record path duplicated messages → the bag is untrustworthy.
# A clean bag is 1.0; the offending GUI bag's camera was 2.06. 1.2 leaves slack for a rare boundary
# stamp collision without admitting a 2x duplication.
_MAX_DUP_FACTOR = 1.2


def unique_stamp_rate(stamps_ns: list[int]) -> float:
    """True rate from a topic's own message timestamps: unique-stamp count over their span.

    De-duplicating on the stamp value collapses the duplicate-frame inflation (each rendered frame's
    sim-timestamp reappearing with a distinct receive time), and dividing by the *stamp* span (not
    wall-clock) makes the result RTF-invariant. Returns 0.0 when there is no span to divide by (< 2
    distinct stamps), which fails a rate band loudly rather than dividing by zero.
    """
    unique = sorted(set(stamps_ns))
    if len(unique) < 2:
        return 0.0
    span_s = (unique[-1] - unique[0]) / 1e9
    return len(unique) / span_s if span_s > 0 else 0.0


def dup_factor(row_count: int, unique_count: int) -> float:
    """Message-stream rows ÷ unique stamps — 1.0 is clean; > 1 means duplicated delivery/recording."""
    if unique_count <= 0:
        return 0.0
    return row_count / unique_count


@dataclass(frozen=True)
class TopicSample:
    """What the witness read from the bag for one asserted topic.

    ``info_count`` is the ``ros2 bag info`` summary count; ``reader_rows`` is how many messages the
    SequentialReader actually yielded. ``rate_stamps_ns`` are the per-message keys used to measure the
    true rate (recorder ``log_time`` — clean on a consistent bag for every type). ``sim_stamps_ns``
    are the sim-time header stamps used only for the duplicate-frame guard, and are empty for types
    with no readable std_msgs/Header (header-less std_msgs, px4, ``/tf``).
    """

    topic: str
    info_count: int
    reader_rows: int
    rate_stamps_ns: list[int]
    sim_stamps_ns: list[int] = field(default_factory=list)

    @property
    def sim_dup_factor(self) -> float | None:
        """rows ÷ unique sim-stamps, or None when no sim stamps were readable for this type."""
        if not self.sim_stamps_ns:
            return None
        return dup_factor(self.reader_rows, len(set(self.sim_stamps_ns)))


def consistency_verdict(
    samples: list[TopicSample], *, max_dup: float = _MAX_DUP_FACTOR
) -> tuple[bool, list[str]]:
    """The hard-fail guard: is this bag trustworthy enough to rate at all?

    A bag is bad if, for any asserted topic, the ``ros2 bag info`` summary count disagrees with the
    message-stream row count (an inconsistent / non-finalized MCAP), or a topic's sim-stamp
    ``dup_factor`` exceeds ``max_dup`` (the record path double-delivered rendered frames). Returns
    ``(ok, reasons)`` — an empty ``reasons`` iff ``ok``.
    """
    reasons = [r for s in samples for r in _sample_reasons(s, max_dup)]
    return (not reasons, reasons)


def _sample_reasons(sample: TopicSample, max_dup: float) -> list[str]:
    """The consistency reasons a single topic sample fails on (empty if it is clean)."""
    reasons: list[str] = []
    if sample.info_count != sample.reader_rows:
        reasons.append(
            f"{sample.topic}: ros2 bag info count {sample.info_count} != "
            f"message-stream rows {sample.reader_rows} (inconsistent/non-finalized bag)"
        )
    factor = sample.sim_dup_factor
    if factor is not None and factor > max_dup:
        reasons.append(
            f"{sample.topic}: dup_factor {factor:.2f} > {max_dup} "
            f"({sample.reader_rows} rows / {len(set(sample.sim_stamps_ns))} unique sim-stamps — "
            "duplicated delivery/recording)"
        )
    return reasons


def observed_true_rates(samples: list[TopicSample]) -> list[ObservedTopic]:
    """Per-topic :class:`ObservedTopic` whose ``.hz`` is the RTF-robust true rate.

    Encodes the unique-stamp rate as ``ObservedTopic(count=unique, duration_s=stamp_span)`` so
    ``ObservedTopic.hz`` stays the single rate definition and ``replay_assertions.evaluate`` needs no
    change. A topic with no usable span becomes count/0 → 0.0 Hz (fails its band loudly).
    """
    return [_observed(s) for s in samples]


def _observed(sample: TopicSample) -> ObservedTopic:
    """One ObservedTopic carrying the unique-stamp rate (count = unique stamps, dur = stamp span)."""
    unique = sorted(set(sample.rate_stamps_ns))
    if len(unique) < 2:
        return ObservedTopic(sample.topic, count=len(unique), duration_s=0.0)
    span_s = (unique[-1] - unique[0]) / 1e9
    return ObservedTopic(sample.topic, count=len(unique), duration_s=span_s)


def evaluate_true_rates(specs: list[AssertionSpec], samples: list[TopicSample]) -> ReplayResult:
    """RTF-robust band check: reuse ``replay_assertions.evaluate`` on the true (unique-stamp) rates."""
    return evaluate(specs, observed_true_rates(samples))
