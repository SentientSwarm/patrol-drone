"""Pure settled-hover tracker for the M3 acceptance harness — ROS-free, clock-injected.

The acceptance "settled_hover" decision lives here, isolated from ``rclpy``, so the harness's hardest
bit of logic — *how long did the vehicle hold within tolerance of the takeoff altitude?* — is
Layer-A unit-testable without a live bridge (mirrors the codebase's decision/mechanism split). The
watcher (:class:`mission_acceptance.MissionAcceptanceWatcher`) is the mechanism: it pulls ``z`` off
``/fmu/out/vehicle_local_position`` and feeds ``(z, now_s)`` here.

A window of "settled" time counts only while the vehicle stays within ``+/- tolerance`` of the target
altitude **and** is *continuously observed*: a gap between consecutive samples longer than
``max_gap_s`` breaks the window. Without that gap break a telemetry blackout spanning the band would
be silently counted as continuous settled-hover time (one in-band sample before an 8 s silence and
one after would "satisfy" the hold), producing a false PASS in the nightly verifier (review #2).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SettleTracker:
    """Longest uninterrupted, continuously-observed window within +/- tolerance of the target."""

    target_z_ned: float
    tolerance_m: float
    max_gap_s: float  # a sample gap longer than this breaks the continuous-observation window
    _window_start_s: float | None = None
    _last_sample_s: float | None = None
    _max_hold_s: float = 0.0

    def update(self, z: float, now_s: float) -> None:
        """Fold one ``(z, now_s)`` sample into the running longest-settled-window measurement."""
        in_band = abs(z - self.target_z_ned) <= self.tolerance_m
        gap_broke = (
            self._last_sample_s is not None and (now_s - self._last_sample_s) > self.max_gap_s
        )
        self._last_sample_s = now_s
        if not in_band:
            self._window_start_s = None  # left the band — break the continuous-hold window
            return
        if gap_broke or self._window_start_s is None:
            self._window_start_s = (
                now_s  # first in-band sample, or restart after an observation gap
            )
            return
        self._max_hold_s = max(self._max_hold_s, now_s - self._window_start_s)

    @property
    def max_hold_s(self) -> float:
        """Longest continuous in-tolerance, continuously-observed window seen so far."""
        return self._max_hold_s
