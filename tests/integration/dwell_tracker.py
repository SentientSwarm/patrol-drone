"""Pure dwell tracker for the M4 patrol acceptance harness — ROS-free (Hermes High).

The "which waypoints were actually reached and dwelled?" decision lives here, isolated from
``rclpy``, so the harness's split-topic correlation is Layer-A unit-testable without a live bridge
(mirrors the codebase's decision/mechanism split — see
:class:`home_settle_tracker.HomeSettleTracker`). The watcher
(:class:`patrol_acceptance.PatrolWatcher`) is the mechanism: it pulls ``mission_state`` and
``current_waypoint`` off the two independent ``/patrol`` topics and feeds them here.

The node publishes ``mission_state`` and ``current_waypoint`` on two separate DDS topics each tick.
DDS guarantees per-topic ordering, NOT an atomic cross-topic snapshot. So on a ``DWELL(i) ->
WAYPOINT(i+1)`` transition a reordered ``current_waypoint=i+1`` can be delivered while the cached
state is still ``DWELL``; attributing a dwell from that waypoint update would mark waypoint ``i+1``
reached before the vehicle ever flew there, false-passing the patrol gate one leg early — including
the final waypoint (Hermes High).

This tracker attributes a dwell **only from the authoritative state signal**: a waypoint counts as
dwelled only when ``DWELL`` is observed (:meth:`on_state`) with a non-negative active index. A
waypoint-index update alone (:meth:`on_waypoint`) only caches the index — it never counts a dwell.
Because the node publishes ``current_waypoint=i`` throughout *both* WAYPOINT(i) and DWELL(i), the
cached index is already ``i`` by the time DWELL(i) is observed, so every genuine dwell is still
counted; and because the state stream is monotonic (DWELL(i) then WAYPOINT(i+1), with no DWELL after
the index advances), the next waypoint is never attributed a dwell from a stale cached state.
"""

from __future__ import annotations

from dataclasses import dataclass, field

_DWELL = "DWELL"


@dataclass
class DwellTracker:
    """Which waypoint indices were observed reached + dwelled (AC-2 / OQ-7), race-free across topics."""

    cur_waypoint: int = -1
    dwelled: set[int] = field(default_factory=set)

    def on_waypoint(self, index: int) -> None:
        """Cache the active waypoint index. Never counts a dwell — that is the state signal's job."""
        self.cur_waypoint = index

    def on_state(self, state: str) -> None:
        """Count the active waypoint dwelled iff the authoritative state is DWELL with a live index."""
        if state == _DWELL and self.cur_waypoint >= 0:
            self.dwelled.add(self.cur_waypoint)
