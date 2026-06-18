"""Pure home-settle tracker for the M4 patrol acceptance harness — ROS-free (Hermes High).

The "did the vehicle return home?" decision lives here, isolated from ``rclpy``, so the harness's
subtlest bit of logic is Layer-A unit-testable without a live bridge (mirrors the codebase's
decision/mechanism split — see :class:`settle_tracker.SettleTracker`). The watcher
(:class:`patrol_acceptance.PatrolWatcher`) is the mechanism: it pulls the vehicle position off
``/fmu/out/vehicle_local_position`` and feeds ``(position, rth_started, valid)`` here.

A settle counts **only once RTH has begun**. The vehicle climbs straight up at home x/y during
takeoff, so it passes *through* the configured home altitude (home ENU z, e.g. 2 m) before any
waypoint is visited. A sample taken then sits at ``home_ned`` within tolerance and — if it were
allowed to latch — would permanently and falsely mark "returned home" before RTH ever runs, letting
the acceptance gate pass on a patrol that never actually returned (Hermes High). Gating on
``rth_started`` means only a position fix within tolerance *after* the RTH state was published
counts, which is what truly proves return-home. An RTH targeting the wrong home never settles here,
so a mis-aimed return still fails (Hermes Medium). Only a valid EKF fix is trusted (``valid``).
"""

from __future__ import annotations

from dataclasses import dataclass

Point = tuple[float, float, float]


@dataclass
class HomeSettleTracker:
    """Did the vehicle settle within ``tolerance_m`` of ``home_ned`` after RTH began? (AC-2/AC-6)."""

    home_ned: Point
    tolerance_m: float
    settled: bool = False
    min_distance_m: float = float("inf")  # closest post-RTH approach to home (diagnostic detail)

    def update(self, position_ned: Point, *, rth_started: bool, valid: bool = True) -> None:
        """Fold one position sample in. Ignores any sample before RTH starts or on an invalid fix."""
        if not rth_started or not valid:
            return  # pre-RTH (e.g. the takeoff climb through home altitude) and bad fixes don't count
        hx, hy, hz = self.home_ned
        x, y, z = position_ned
        distance = ((x - hx) ** 2 + (y - hy) ** 2 + (z - hz) ** 2) ** 0.5
        self.min_distance_m = min(self.min_distance_m, distance)
        if distance <= self.tolerance_m:
            self.settled = True
