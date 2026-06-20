"""Pure dwell tracker for the M4 patrol acceptance harness — ROS-free (Hermes High).

The "how many waypoints were actually reached and dwelled for their configured time?" decision lives
here, isolated from ``rclpy``, so the harness's verdict is Layer-A unit-testable without a live bridge
(mirrors the codebase's decision/mechanism split — see :class:`home_settle_tracker.HomeSettleTracker`).
The watcher (:class:`patrol_acceptance.PatrolWatcher`) is the mechanism: it feeds the timestamped
``mission_state`` stream here.

Why this counts the state stream **alone**. The node publishes ``mission_state`` and
``current_waypoint`` on two independent ``/patrol`` DDS topics (OQ-3 fixes them to plain
``std_msgs``, so they carry no shared sequence/timestamp to correlate). DDS guarantees per-topic
ordering but NOT an atomic cross-topic snapshot, so the two streams can interleave arbitrarily: a
``current_waypoint=i+1`` sample can be delivered while a still-pending ``DWELL(i)`` state sample
from waypoint ``i`` has yet to arrive. Any attribution that reads the cached waypoint index when a
``DWELL`` state is observed can therefore credit the *next* waypoint a dwell it never earned —
false-passing the patrol gate one leg early, including the final waypoint (Hermes High).

So this tracker never reads ``current_waypoint``. It tracks **DWELL episodes** — each rising edge
into the ``DWELL`` state — in the ``mission_state`` stream, which DDS keeps per-topic-ordered. The
patrol dwells exactly once per waypoint, in strict index order: the state machine enters
``DWELL(i)`` only from ``WAYPOINT(i)``, and ``_advance_from_dwell`` only ever *increments* the index
and never re-enters ``DWELL`` for the same one. So the k-th episode is waypoint k, and because
``current_waypoint`` never participates, no cross-topic reorder can change the count.

Why **duration** matters (Hermes High). Counting an episode on its rising edge alone proves only
that the machine *entered* ``DWELL`` — a single ``DWELL`` sample would satisfy the gate. But AC-2/AC-9
require each waypoint to be *held* for its configured ``dwell_s`` (the state machine holds ``DWELL(i)``
for ``waypoints[i].dwell_s`` via ``_elapsed_since_entry`` before advancing). So an episode counts as a
real dwell only once it has spanned its configured ``dwell_s``, measured from the first ``DWELL(i)``
sample to the sample that ends the episode (the node republishes ``DWELL`` every 10 Hz tick for the
whole hold, then publishes the next state on the tick the hold completes — so the entry→exit span is
``>= dwell_s``). A transient one-tick ``DWELL`` spans ~0 s and is correctly rejected.

Why **observation gaps** must not count (Hermes Medium). The span is measured in wall clock, so two
sparse ``DWELL`` samples delivered far apart (a starved executor, a telemetry blackout) could
otherwise credit a hold that was never continuously observed — making the oracle *less* conservative
than the node, which pauses progression and ``reset_timing()``s the same window on stale telemetry. A
gap wider than ``max_gap_s`` between consecutive ``DWELL`` samples therefore restarts the episode
clock (mirrors :class:`home_settle_tracker.HomeSettleTracker`), so each credited dwell is backed by a
continuously observed span.
"""

from __future__ import annotations

from dataclasses import dataclass, field

_DWELL = "DWELL"


@dataclass
class DwellTracker:
    """Which waypoints were reached AND held for their configured ``dwell_s`` (AC-2/AC-9/OQ-7).

    Counted race-free from the single per-topic-ordered ``mission_state`` stream, and duration-aware:
    a waypoint counts only once its DWELL episode has spanned ``dwell_required_s[index]``.
    """

    dwell_required_s: tuple[float, ...]  # index-aligned required dwell per waypoint, from the route
    max_gap_s: float = 1.0  # a gap between DWELL samples wider than this breaks the continuous hold
    _episode_index: int = -1  # index of the current/last DWELL episode (-1 before any)
    _in_dwell: bool = False
    _entered_s: float = 0.0  # timestamp of the rising edge (or post-gap restart) of the episode
    _last_sample_s: float | None = None  # timestamp of the previous DWELL sample in this episode
    _dwelled: set[int] = field(default_factory=set)  # indices whose episode met its required dwell

    def on_state(self, state: str, now_s: float) -> None:
        """Fold one timestamped ``mission_state`` sample in.

        A rising edge into ``DWELL`` opens a new episode for the next waypoint index; while in (and on
        leaving) the episode, credit the waypoint once its span has reached the configured dwell. An
        observation gap wider than ``max_gap_s`` between two ``DWELL`` samples breaks the continuous
        hold and restarts the episode clock, so a starved/blackout span cannot be credited as dwell
        time — keeping the oracle at least as conservative as the node, which pauses progression and
        ``reset_timing()``s the same window on stale telemetry (Hermes Medium).
        """
        if state != _DWELL:
            if self._in_dwell:  # falling edge: finalize the episode with the exit-sample time
                self._credit_if_held(now_s)
                self._in_dwell = False
            return
        gap_broke = (
            self._in_dwell
            and self._last_sample_s is not None
            and (now_s - self._last_sample_s) > self.max_gap_s
        )
        self._last_sample_s = now_s
        if not self._in_dwell:  # rising edge: a new episode for waypoint _episode_index + 1
            self._episode_index += 1
            self._entered_s = now_s
            self._in_dwell = True
        elif (
            gap_broke
        ):  # mid-episode observation gap: the continuous hold is broken, restart the clock
            self._entered_s = now_s
        self._credit_if_held(now_s)

    def _credit_if_held(self, now_s: float) -> None:
        """Credit the current episode's waypoint iff it has now spanned its configured ``dwell_s``."""
        i = self._episode_index
        if (
            0 <= i < len(self.dwell_required_s)
            and now_s - self._entered_s >= self.dwell_required_s[i]
        ):
            self._dwelled.add(i)

    @property
    def episodes(self) -> int:
        """DWELL episodes *started* (waypoints reached) — may exceed ``dwelled`` if a hold was short."""
        return self._episode_index + 1

    @property
    def dwelled(self) -> set[int]:
        """Waypoint indices proven reached AND held for the configured dwell: episode k <-> waypoint k."""
        return set(self._dwelled)
