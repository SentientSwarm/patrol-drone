"""Shared M3 basic-mission acceptance criteria — the single source of PASS/FAIL truth.

Both consumers import this module so the definition of "the basic mission flew" lives in exactly
one place and the host path and CI can't drift (UAT design — Verifier layer):

  * the nightly SITL integration test  -> ``tests/integration/test_mission_basic.py``
    (launches the node via ``mission_basic.launch.py`` and asserts each check)
  * the host-side verifier              -> ``scripts/verify_mission.py``
    (subscribes to an already-running stack and prints each check PASS/FAIL)

This is a **Layer-B** module: it imports ``rclpy`` + ``px4_msgs``, so it is excluded from the
Layer-A unit runner (pyproject ``norecursedirs``) and from mypy (pyproject ``exclude``). It ships
to the nightly container via ``docker cp tests`` (.github/workflows/sitl-nightly.yml). It defines
no ``test_*`` functions, so neither pytest tier collects it as a test.

Acceptance criteria (mission_basic.yaml: ``takeoff_alt_m=5``, ``hover 10 s``, home at origin):

  1. ARMED          — ``arming_state == 2`` observed.
  2. OFFBOARD       — ``nav_state == 14`` observed.
  3. reached_alt    — climbed to within tolerance of (or above) the takeoff altitude.
  4. settled_hover  — held the takeoff altitude **+/- tolerance** continuously (the *settled*
                      altitude, not the peak: a transient climb overshoot is expected step
                      response, not a failure).
  5. landed_disarmed — disarmed again after arming + climbing (the landing).

Topic names come from ``patrol_mission.topics`` (PX4 v1.17: ``/fmu/out/*`` outputs are ``_v1``).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from ament_index_python.packages import get_package_share_directory
from patrol_mission.config import load_mission_config
from patrol_mission.frames import Point, takeoff_target_ned, to_ned_from_origin
from patrol_mission.qos import px4_qos
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from settle_tracker import SettleTracker

from patrol_mission import topics

# SITL VehicleLocalPosition is already EKF-origin-relative NED, so the configured home is converted
# with the same zero origin the node uses (node.py _EKF_ORIGIN_NED, mirrored by patrol_acceptance).
_EKF_ORIGIN_NED: Point = (0.0, 0.0, 0.0)

# The settled-hover window need only cover MOST of the hover, not all of it: the climb must first
# settle into the band, and /fmu/out is sampled at ~2 Hz here. Sized below hover_time_s so a real
# hover passes while a climb-then-immediately-land (or a mid-hover dip) cannot accumulate a window.
SETTLE_MARGIN_S = 2.0
# A position-sample gap longer than this breaks the settled-hover window: /fmu/out is sampled at
# ~2 Hz here, so a >1 s silence means observation was lost and we cannot claim the vehicle stayed in
# the band through it. Without this, a telemetry blackout spanning the band would be counted as
# continuous settled-hover time (review #2). Sized above the nominal sampling interval so ordinary
# jitter never trips it.
MAX_SETTLE_SAMPLE_GAP_S = 1.0
# Upper bound on how long to watch a single mission before giving up (covers climb + 10 s hover +
# land with generous slack); the verifier/test stop early the instant all criteria are met.
MISSION_TIMEOUT_S = 120.0


@dataclass(frozen=True)
class AcceptanceThresholds:
    """Numbers the checks compare against, derived once from the flown mission YAML."""

    takeoff_alt_m: float
    hover_time_s: float
    tolerance_m: float
    target_z_ned: float  # the takeoff-target down coord the state machine flies to (home-relative)
    min_settled_hold_s: float  # required continuous time in the settle band
    mission_timeout_s: float


def _default_mission_yaml() -> str:
    """The same checked-in YAML mission_basic.launch.py feeds the node (via the installed share)."""
    share = get_package_share_directory("patrol_bringup")
    return f"{share}/config/mission_basic.yaml"


def load_thresholds(mission_yaml: str | None = None) -> AcceptanceThresholds:
    """Derive the acceptance thresholds from the flown mission config (never hardcoded here).

    The settle-band target is the same point the state machine flies to: ``takeoff_alt_m`` above the
    *configured home*, derived by converting home through the single MC-7 frame boundary and reusing
    ``frames.takeoff_target_ned``. Hardcoding ``-takeoff_alt_m`` here would silently drift the moment
    home is not at z=0 — and the shipped mission sits home at 2 m ENU, so the band would land 2 m off
    the vehicle's actual hover altitude and the settled-hover check could never pass (Hermes).
    """
    cfg = load_mission_config(mission_yaml or _default_mission_yaml())
    home_ned = to_ned_from_origin(cfg.home_position, cfg.home_frame, _EKF_ORIGIN_NED)
    _, _, target_z_ned = takeoff_target_ned(home_ned, cfg.takeoff_alt_m)
    return AcceptanceThresholds(
        takeoff_alt_m=cfg.takeoff_alt_m,
        hover_time_s=cfg.hover_time_s,
        tolerance_m=cfg.completion.tolerance_m,
        target_z_ned=target_z_ned,
        min_settled_hold_s=max(0.0, cfg.hover_time_s - SETTLE_MARGIN_S),
        mission_timeout_s=MISSION_TIMEOUT_S,
    )


class MissionAcceptanceWatcher(Node):
    """Records the arm/offboard/settled-altitude/disarm milestones observed on ``/fmu/out/*``."""

    def __init__(
        self,
        thresholds: AcceptanceThresholds,
        *,
        node_name: str = "mission_acceptance_watcher",
    ) -> None:
        super().__init__(node_name)
        self._t = thresholds
        self.was_armed = False
        self.saw_offboard = False
        self.reached_altitude = False
        self.disarmed_after_arm = False
        # Settled-hover measurement (longest continuously-observed in-tolerance window) lives in a
        # pure, Layer-A-tested tracker; the watcher just feeds it (z, now) samples off /fmu/out.
        self._settle = SettleTracker(
            target_z_ned=thresholds.target_z_ned,
            tolerance_m=thresholds.tolerance_m,
            max_gap_s=MAX_SETTLE_SAMPLE_GAP_S,
        )
        qos = px4_qos()
        self.create_subscription(VehicleStatus, topics.VEHICLE_STATUS, self._on_status, qos)
        self.create_subscription(
            VehicleLocalPosition, topics.VEHICLE_LOCAL_POSITION, self._on_pos, qos
        )

    def _on_status(self, msg: VehicleStatus) -> None:
        if msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.saw_offboard = True
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.was_armed = True
        elif self.was_armed and self.reached_altitude:
            self.disarmed_after_arm = True

    def _on_pos(self, msg: VehicleLocalPosition) -> None:
        if not (msg.xy_valid and msg.z_valid):
            return  # don't judge altitude on an unconverged EKF estimate (mirrors the node's gate)
        z = float(msg.z)
        if z <= self._t.target_z_ned + self._t.tolerance_m:
            self.reached_altitude = True  # climbed to >= (takeoff_alt - tolerance) at least once
        self._settle.update(z, time.monotonic())

    @property
    def settled_hold_s(self) -> float:
        """Longest uninterrupted window observed within +/- tolerance of the takeoff altitude."""
        return self._settle.max_hold_s

    @property
    def mission_complete(self) -> bool:
        """All acceptance criteria observed — the spin loop stops early once this is true."""
        return (
            self.was_armed
            and self.saw_offboard
            and self.reached_altitude
            and self.disarmed_after_arm
            and self.settled_hold_s >= self._t.min_settled_hold_s
        )


@dataclass(frozen=True)
class Check:
    """One acceptance criterion's verdict, with a human-readable detail for the report/assert."""

    name: str
    passed: bool
    detail: str


def evaluate(watcher: MissionAcceptanceWatcher, t: AcceptanceThresholds) -> list[Check]:
    """Turn the watcher's observations into the ordered list of acceptance checks."""
    reached_floor = t.takeoff_alt_m - t.tolerance_m
    return [
        Check("armed", watcher.was_armed, "vehicle reported ARMED (arming_state == 2)"),
        Check("offboard", watcher.saw_offboard, "vehicle entered OFFBOARD (nav_state == 14)"),
        Check(
            "reached_altitude",
            watcher.reached_altitude,
            f"climbed to >= {reached_floor:.1f} m AGL",
        ),
        Check(
            "settled_hover",
            watcher.settled_hold_s >= t.min_settled_hold_s,
            f"held {t.takeoff_alt_m:.1f} +/- {t.tolerance_m:.1f} m for "
            f"{watcher.settled_hold_s:.1f} s (need >= {t.min_settled_hold_s:.1f} s)",
        ),
        Check(
            "landed_disarmed",
            watcher.disarmed_after_arm,
            "disarmed after arming + climb (landing completed)",
        ),
    ]


def spin_until_complete(
    watcher: MissionAcceptanceWatcher,
    t: AcceptanceThresholds,
    *,
    timeout_s: float | None = None,
) -> None:
    """Spin the watcher until every criterion is met or the timeout elapses (caller owns rclpy)."""

    deadline = time.monotonic() + (timeout_s if timeout_s is not None else t.mission_timeout_s)
    while time.monotonic() < deadline and not watcher.mission_complete:
        rclpy.spin_once(watcher, timeout_sec=0.5)
