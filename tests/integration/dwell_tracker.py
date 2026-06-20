"""Pure dwell tracker for the M4 patrol acceptance harness — ROS-free (Hermes High).

The "how many waypoints were actually reached and dwelled?" decision lives here, isolated from
``rclpy``, so the harness's verdict is Layer-A unit-testable without a live bridge (mirrors the
codebase's decision/mechanism split — see :class:`home_settle_tracker.HomeSettleTracker`). The
watcher (:class:`patrol_acceptance.PatrolWatcher`) is the mechanism: it feeds the ``mission_state``
stream here.

Why this counts the state stream **alone**. The node publishes ``mission_state`` and
``current_waypoint`` on two independent ``/patrol`` DDS topics (OQ-3 fixes them to plain
``std_msgs``, so they carry no shared sequence/timestamp to correlate). DDS guarantees per-topic
ordering but NOT an atomic cross-topic snapshot, so the two streams can interleave arbitrarily: a
``current_waypoint=i+1`` sample can be delivered while a still-pending ``DWELL(i)`` state sample
from waypoint ``i`` has yet to arrive. Any attribution that reads the cached waypoint index when a
``DWELL`` state is observed can therefore credit the *next* waypoint a dwell it never earned —
false-passing the patrol gate one leg early, including the final waypoint (Hermes High).

So this tracker never reads ``current_waypoint``. It counts **DWELL episodes** — each rising edge
into the ``DWELL`` state — in the ``mission_state`` stream, which DDS keeps per-topic-ordered. The
patrol dwells exactly once per waypoint, in strict index order: the state machine enters
``DWELL(i)`` only from ``WAYPOINT(i)``, and ``_advance_from_dwell`` only ever *increments* the index
and never re-enters ``DWELL`` for the same one. So the number of DWELL episodes is exactly the
number of waypoints reached + dwelled, and the k-th episode is waypoint k. Because the
``current_waypoint`` topic never participates, no cross-topic reorder can change the count — the
race is structurally impossible rather than merely unlikely.
"""

from __future__ import annotations

from dataclasses import dataclass

_DWELL = "DWELL"


@dataclass
class DwellTracker:
    """How many waypoints were reached + dwelled (AC-2 / OQ-7), counted race-free from one topic."""

    _episodes: int = 0
    _in_dwell: bool = False

    def on_state(self, state: str) -> None:
        """Count one dwell per rising edge into DWELL (consecutive DWELL samples are one episode)."""
        if state == _DWELL and not self._in_dwell:
            self._episodes += 1
        self._in_dwell = state == _DWELL

    @property
    def episodes(self) -> int:
        """Distinct DWELL episodes observed = waypoints reached + dwelled, in index order."""
        return self._episodes

    @property
    def dwelled(self) -> set[int]:
        """Waypoint indices proven reached + dwelled: episode k <-> waypoint k (in-order patrol)."""
        return set(range(self._episodes))
