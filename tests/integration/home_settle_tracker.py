"""Pure home-settle tracker for the M4 patrol acceptance harness — ROS-free (Hermes High).

The "did the vehicle return home and hold there?" decision lives here, isolated from ``rclpy``, so the
harness's subtlest bit of logic is Layer-A unit-testable without a live bridge (mirrors the codebase's
decision/mechanism split — see :class:`settle_tracker.SettleTracker`). The watcher
(:class:`patrol_acceptance.PatrolWatcher`) is the mechanism: it pulls the vehicle position off
``/fmu/out/vehicle_local_position`` and feeds ``(position, now_s, rth_started, valid)`` here.

A settle counts **only once RTH has begun**. The vehicle climbs straight up at home x/y during
takeoff, so it passes *through* the configured home altitude (home ENU z, e.g. 2 m) before any
waypoint is visited. A sample taken then sits at ``home_ned`` within tolerance and — if it were
allowed to latch — would permanently and falsely mark "returned home" before RTH ever runs, letting
the acceptance gate pass on a patrol that never actually returned (Hermes High). Gating on
``rth_started`` means only a position fix within tolerance *after* the RTH state was published counts.
An RTH targeting the wrong home never settles here, so a mis-aimed return still fails (Hermes Medium).
Only a valid EKF fix is trusted (``valid``).

A settle also requires a **continuous hold**, not a single in-tolerance fix (Hermes High). The
production state machine leaves RTH for LANDING only after ``_within_tolerance_for_hold()`` — i.e. the
vehicle stays within ``tolerance_m`` of home for ``hold_time_s`` continuously. A one-sample crossing of
the home ball (the vehicle flying *through* home, overshooting, or a transient EKF blip) does not prove
a real return; this tracker mirrors the production rule exactly, latching ``settled`` only after the
position has stayed in tolerance for ``hold_time_s``, and resetting the hold clock the moment it leaves.

The hold must also be **continuously observed** (PR #8 post-mortem C — sparse samples must not pass
as a continuous hold). A silence between consecutive post-RTH fixes longer than ``max_gap_s`` means
observation was lost: we cannot claim the vehicle stayed within tolerance across the gap, so the hold
restarts. Without this, one in-tolerance fix before a telemetry blackout and one after — spanning
``hold_time_s`` with nothing in between — would falsely latch a settle (this mirrors the M3
:class:`settle_tracker.SettleTracker`, which already guards its hover window the same way).
"""

from __future__ import annotations

from dataclasses import dataclass

Point = tuple[float, float, float]


@dataclass
class HomeSettleTracker:
    """Did the vehicle hold within ``tolerance_m`` of ``home_ned`` for ``hold_time_s`` after RTH began?

    Mirrors the state machine's ``_within_tolerance_for_hold`` RTH->LANDING criterion (AC-2/AC-6/MC-5).
    """

    home_ned: Point
    tolerance_m: float
    hold_time_s: float
    max_gap_s: float = 1.0  # a sample gap longer than this breaks the continuous-observation hold
    settled: bool = False
    min_distance_m: float = float("inf")  # closest post-RTH approach to home (diagnostic detail)
    _inside_since_s: float | None = (
        None  # when the current in-tolerance hold began (None = outside)
    )
    _last_sample_s: float | None = None  # time of the previous usable (post-RTH, valid) fix

    def update(
        self, position_ned: Point, now_s: float, *, rth_started: bool, valid: bool = True
    ) -> None:
        """Fold one timestamped position sample in. Ignores any sample before RTH starts or on a bad fix.

        While within tolerance, track when the hold began and latch ``settled`` once it has lasted
        ``hold_time_s``; leaving the tolerance ball — or a silence longer than ``max_gap_s`` since the
        previous usable fix — resets the hold clock (a real settle is continuous AND continuously
        observed).
        """
        if not rth_started or not valid:
            return  # pre-RTH (e.g. the takeoff climb through home altitude) and bad fixes don't count
        hx, hy, hz = self.home_ned
        x, y, z = position_ned
        distance = ((x - hx) ** 2 + (y - hy) ** 2 + (z - hz) ** 2) ** 0.5
        self.min_distance_m = min(self.min_distance_m, distance)
        # An observation gap wider than max_gap_s breaks the hold even across two in-tolerance fixes:
        # a telemetry blackout spanning the home ball must not count as a continuous hold (post-mortem
        # C). Measured against the previous usable fix, in- or out-of-tolerance.
        gap_broke = (
            self._last_sample_s is not None and (now_s - self._last_sample_s) > self.max_gap_s
        )
        self._last_sample_s = now_s
        if distance > self.tolerance_m:
            self._inside_since_s = None  # left the tolerance ball; the hold must start over
            return
        if gap_broke or self._inside_since_s is None:
            self._inside_since_s = (
                now_s  # first in-tolerance fix, or restart after an observation gap
            )
        if now_s - self._inside_since_s >= self.hold_time_s:
            self.settled = True
