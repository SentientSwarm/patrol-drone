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
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from patrol_mission import topics

# The settled-hover window need only cover MOST of the hover, not all of it: the climb must first
# settle into the band, and /fmu/out is sampled at ~2 Hz here. Sized below hover_time_s so a real
# hover passes while a climb-then-immediately-land (or a mid-hover dip) cannot accumulate a window.
SETTLE_MARGIN_S = 2.0
# Upper bound on how long to watch a single mission before giving up (covers climb + 10 s hover +
# land with generous slack); the verifier/test stop early the instant all criteria are met.
MISSION_TIMEOUT_S = 120.0


@dataclass(frozen=True)
class AcceptanceThresholds:
    """Numbers the checks compare against, derived once from the flown mission YAML."""

    takeoff_alt_m: float
    hover_time_s: float
    tolerance_m: float
    target_z_ned: float  # -takeoff_alt_m (NED down is negative-up)
    min_settled_hold_s: float  # required continuous time in the settle band
    mission_timeout_s: float


def _default_mission_yaml() -> str:
    """The same checked-in YAML mission_basic.launch.py feeds the node (via the installed share)."""
    share = get_package_share_directory("patrol_bringup")
    return f"{share}/config/mission_basic.yaml"


def load_thresholds(mission_yaml: str | None = None) -> AcceptanceThresholds:
    """Derive the acceptance thresholds from the flown mission config (never hardcoded here)."""
    cfg = load_mission_config(mission_yaml or _default_mission_yaml())
    return AcceptanceThresholds(
        takeoff_alt_m=cfg.takeoff_alt_m,
        hover_time_s=cfg.hover_time_s,
        tolerance_m=cfg.completion.tolerance_m,
        target_z_ned=-cfg.takeoff_alt_m,
        min_settled_hold_s=max(0.0, cfg.hover_time_s - SETTLE_MARGIN_S),
        mission_timeout_s=MISSION_TIMEOUT_S,
    )


def _px4_qos() -> QoSProfile:
    """The /fmu/* QoS PX4's uXRCE-DDS bridge publishes with (matches the node + px4_ros_com)."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
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
        # Longest *uninterrupted* window observed within the settle band; resets on any sample that
        # leaves the band, so a transient overshoot/dip cannot accumulate into a passing hover.
        self._settle_start_s: float | None = None
        self._max_settled_hold_s = 0.0
        qos = _px4_qos()
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
        self._track_settle(z)

    def _track_settle(self, z: float) -> None:
        """Accumulate the continuous time spent within +/- tolerance of the takeoff altitude."""
        lo = self._t.target_z_ned - self._t.tolerance_m
        hi = self._t.target_z_ned + self._t.tolerance_m
        if not (lo <= z <= hi):
            self._settle_start_s = None  # left the band — break the continuous-hold window
            return
        now = time.monotonic()
        if self._settle_start_s is None:
            self._settle_start_s = now
        self._max_settled_hold_s = max(self._max_settled_hold_s, now - self._settle_start_s)

    @property
    def settled_hold_s(self) -> float:
        """Longest uninterrupted window observed within +/- tolerance of the takeoff altitude."""
        return self._max_settled_hold_s

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
