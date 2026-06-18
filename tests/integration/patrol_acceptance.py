"""Shared patrol-mission acceptance criteria (AC-2 / AC-6) — single PASS/FAIL truth for CI + host.

Mirrors :mod:`mission_acceptance` (the M3 basic-mission harness) for the M4 multi-waypoint patrol.
Both consumers import this so "the patrol flew" / "the abort was observable" is defined in exactly
one place and the nightly SITL test and the host verifier can't drift:

  * the nightly SITL integration test -> ``tests/integration/test_mission_patrol.py``
  * the host-side verifier              -> ``scripts/verify_patrol.py`` (M4 UAT slice, SWM-40)

Layer-B: imports ``rclpy`` + ``px4_msgs`` + ``std_msgs``, so it is excluded from the Layer-A unit
runner and from mypy (pyproject), and ships to the nightly container via ``docker cp tests``. It
defines no ``test_*`` functions, so neither pytest tier collects it as a test.

The observable patrol surface is ``/patrol/*`` (OQ-3): ``mission_state`` (the MissionState name) and
``current_waypoint`` (the active index). Arm/disarm comes from ``/fmu/out/vehicle_status`` (the same
``_v1`` output the basic harness reads). The acceptance criteria:

  AC-2 (nominal patrol): armed -> every configured waypoint index observed in DWELL (reached and
        dwelled, not merely targeted) -> RTH observed -> disarmed after arming.
  AC-6 (external abort): an external ``/patrol/abort`` published mid-patrol drives an observable
        ABORT then RTH, then disarm (asserted by the abort scenario in the test, using this watcher).
"""

from __future__ import annotations

import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from home_settle_tracker import HomeSettleTracker
from mission_acceptance import Check  # reuse the one Check verdict shape
from patrol_mission.frames import Point, to_ned_from_origin
from patrol_mission.qos import patrol_state_qos, px4_qos
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import Int32, String

from patrol_mission import topics

# SITL VehicleLocalPosition is already EKF-origin-relative NED, so the watcher converts the
# configured home to NED with the same zero origin the node uses (node.py _EKF_ORIGIN_NED). The
# home-settle check then compares the vehicle's reported position against the *same* home_ned the
# state machine flies to before LANDING — catching an RTH that targets the wrong coordinate (Hermes).
_EKF_ORIGIN_NED: Point = (0.0, 0.0, 0.0)

# Generous upper bound on a single patrol run: takeoff + hover + 4*(fly + dwell) + RTH + land, with
# slack. The verifier/test stop early the instant the criteria are met. Within the OQ-5 ≤8 min/
# scenario provisional budget; MZ.1 re-measures.
PATROL_TIMEOUT_S = 300.0


def _patrol_mission_yaml() -> str:
    """The same checked-in YAML mission_patrol.launch.py feeds the node (via the installed share)."""
    return f"{get_package_share_directory('patrol_bringup')}/config/patrol_mission.yaml"


def _mission_raw(mission_yaml: str | None = None) -> dict:
    """The raw mission YAML — read directly (no checkpoint_id resolution), so the acceptance process
    never depends on 03's checkpoints file being reachable just to read waypoint count / home."""
    with open(mission_yaml or _patrol_mission_yaml()) as fh:
        return yaml.safe_load(fh)


def expected_waypoint_count(mission_yaml: str | None = None) -> int:
    """Number of waypoints the patrol must visit — counted from the route YAML, never hardcoded."""
    return len(_mission_raw(mission_yaml)["waypoints"])


def home_target_ned(mission_yaml: str | None = None) -> Point:
    """The configured home, converted to the EKF-origin-relative NED the vehicle reports (MC-7).

    Mirrors the node's own home_ned derivation (``to_ned_from_origin`` with a zero EKF origin), so
    the home-settle check compares the vehicle against the exact coordinate RTH flies to.
    """
    home = _mission_raw(mission_yaml)["home"]
    p = home["position"]
    return to_ned_from_origin((p["x"], p["y"], p["z"]), home["frame"], _EKF_ORIGIN_NED)


def home_tolerance_m(mission_yaml: str | None = None) -> float:
    """The completion-tolerance ball (m) for the home settle — the same radius the state machine
    uses to leave RTH for LANDING. Reads ``completion.tolerance_m`` (OQ-4 default 0.5)."""
    completion = _mission_raw(mission_yaml).get("completion") or {}
    return float(completion.get("tolerance_m", 0.5))


# States that only occur once the vehicle is airborne. A disarm after any of these is a landing —
# whether the mission flew the full patrol OR aborted early (an abort can fire during HOVER, before
# any waypoint is visited), so this is what gates "disarmed after arming" rather than a waypoint.
_AIRBORNE_STATES = frozenset({"TAKEOFF", "HOVER", "WAYPOINT", "DWELL", "RTH", "LANDING"})


class PatrolWatcher(Node):
    """Records the patrol's observable surface: states seen (ordered), waypoint indices visited,
    and arm/disarm — off ``/patrol/*`` + ``/fmu/out/vehicle_status``."""

    def __init__(
        self,
        expected_waypoints: int,
        *,
        home_ned: Point | None = None,
        home_tol_m: float | None = None,
        node_name: str = "patrol_acceptance_watcher",
    ):
        super().__init__(node_name)
        self._expected = expected_waypoints
        # The home coordinate RTH must settle at, and the tolerance ball (default: read from the same
        # checked-in mission YAML the launch uses), so the home-settle check is config-driven.
        self.home_ned = home_ned if home_ned is not None else home_target_ned()
        self.home_tol_m = home_tol_m if home_tol_m is not None else home_tolerance_m()
        self.states_seen: list[str] = []  # deduped-consecutive ordered mission_state history
        self.waypoints_visited: set[int] = (
            set()
        )  # active targets seen (current_waypoint>=0): underway
        self.waypoints_dwelled: set[int] = (
            set()
        )  # indices observed in DWELL — reached, not just targeted
        self.was_armed = False
        self.disarmed_after_arm = False
        # Return-home decision lives in a pure, Layer-A-tested tracker; the watcher feeds it position
        # samples and gates them on RTH having started, so the takeoff climb through the home altitude
        # can't falsely latch "returned home" before RTH runs (see HomeSettleTracker / Hermes High).
        self._home_settle = HomeSettleTracker(self.home_ned, self.home_tol_m)
        # Latest sample on each /patrol topic, correlated in _note_dwell so a waypoint counts as
        # *reached* only once DWELL is observed with its index active (not merely targeted).
        self._cur_state = ""
        self._cur_wp = -1
        pqos = patrol_state_qos()
        self.create_subscription(String, topics.PATROL_MISSION_STATE, self._on_state, pqos)
        self.create_subscription(Int32, topics.PATROL_CURRENT_WAYPOINT, self._on_wp, pqos)
        self.create_subscription(VehicleStatus, topics.VEHICLE_STATUS, self._on_status, px4_qos())
        self.create_subscription(
            VehicleLocalPosition, topics.VEHICLE_LOCAL_POSITION, self._on_local_pos, px4_qos()
        )

    def _on_state(self, msg: String) -> None:
        if not self.states_seen or self.states_seen[-1] != msg.data:
            self.states_seen.append(msg.data)
        self._cur_state = msg.data
        self._note_dwell()

    def _on_wp(self, msg: Int32) -> None:
        if msg.data >= 0:
            self.waypoints_visited.add(msg.data)
        self._cur_wp = msg.data
        self._note_dwell()

    def _note_dwell(self) -> None:
        """Count a waypoint as reached only when observed in DWELL with its index active (OQ-7).

        The node publishes ``current_waypoint=i`` while still *flying toward* waypoint i (WAYPOINT
        state), so counting any non-negative index as "visited" would pass the patrol gate on
        approach, before arrival/dwell (Hermes High). ``current_waypoint`` stays ``i`` across both
        WAYPOINT(i) and DWELL(i), so correlating the two /patrol topics — DWELL + index i — is sound
        evidence that waypoint i was actually reached and dwelled, which is the AC-2 / OQ-7 contract.
        """
        if self._cur_state == "DWELL" and self._cur_wp >= 0:
            self.waypoints_dwelled.add(self._cur_wp)

    def _on_status(self, msg: VehicleStatus) -> None:
        if msg.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.was_armed = True
        elif self.was_armed and self._flew:
            self.disarmed_after_arm = True  # disarmed after arming + getting airborne (a landing)

    def _on_local_pos(self, msg: VehicleLocalPosition) -> None:
        """Feed the vehicle position to the home-settle tracker, gated on RTH having begun (AC-2/AC-6).

        The settle counts only *after* RTH starts: the takeoff climb passes through the configured
        home altitude at home x/y, so a pre-RTH sample sits at home_ned within tolerance and — if it
        latched — would falsely mark "returned home" before RTH ever runs (Hermes High). The pure
        HomeSettleTracker owns that rule, the RTH->LANDING-style tolerance check, and the
        closest-approach diagnostic; the watcher just supplies (position, rth_started, valid).
        """
        self._home_settle.update(
            (float(msg.x), float(msg.y), float(msg.z)),
            rth_started=self.saw_rth,
            valid=bool(msg.xy_valid and msg.z_valid),
        )

    @property
    def _flew(self) -> bool:
        """True once the vehicle has been airborne, so a later disarm is a landing not pre-flight.

        Holds for both the nominal patrol and an early abort (abort can fire during HOVER, before any
        waypoint), so the disarm gate works for both scenarios.
        """
        return any(s in _AIRBORNE_STATES for s in self.states_seen)

    @property
    def all_waypoints_dwelled(self) -> bool:
        """Every configured waypoint index was observed in DWELL — reached + dwelled, not just targeted."""
        return self.waypoints_dwelled >= set(range(self._expected))

    @property
    def saw_rth(self) -> bool:
        return "RTH" in self.states_seen

    @property
    def abort_then_rth(self) -> bool:
        """ABORT was observed and an RTH followed it (the AC-6 observable return-home)."""
        if "ABORT" not in self.states_seen or "RTH" not in self.states_seen:
            return False
        return self.states_seen.index("RTH") > self.states_seen.index("ABORT")

    @property
    def settled_near_home(self) -> bool:
        """The vehicle reached the configured home_ned within tolerance *after* RTH began (Hermes)."""
        return self._home_settle.settled

    @property
    def min_home_distance_m(self) -> float:
        """Closest post-RTH approach to home_ned (diagnostic; inf until RTH starts being observed)."""
        return self._home_settle.min_distance_m

    @property
    def returned_home(self) -> bool:
        """RTH observed AND the vehicle settled at the configured home_ned (not just the state)."""
        return self.saw_rth and self.settled_near_home

    @property
    def nominal_complete(self) -> bool:
        """Every nominal-patrol criterion observed — the spin loop stops early once true (AC-2)."""
        return (
            self.was_armed
            and self.all_waypoints_dwelled
            and self.returned_home
            and self.disarmed_after_arm
        )


def evaluate_nominal(watcher: PatrolWatcher, expected_waypoints: int) -> list[Check]:
    """The AC-2 nominal-patrol checks: armed, every waypoint reached + dwelled, RTH, land."""
    return [
        Check("armed", watcher.was_armed, "vehicle reported ARMED"),
        Check(
            "all_waypoints_dwelled",
            watcher.all_waypoints_dwelled,
            f"dwelled at waypoint indices {sorted(watcher.waypoints_dwelled)} "
            f"(need 0..{expected_waypoints - 1}); active targets seen "
            f"{sorted(watcher.waypoints_visited)}",
        ),
        Check(
            "returned_home",
            watcher.returned_home,
            f"RTH observed and vehicle settled within {watcher.home_tol_m} m of home_ned "
            f"{watcher.home_ned}; closest approach {watcher.min_home_distance_m:.2f} m",
        ),
        Check(
            "landed_disarmed",
            watcher.disarmed_after_arm,
            "disarmed after arming + visiting waypoints (landing completed)",
        ),
    ]


def spin_until(watcher: PatrolWatcher, predicate, *, timeout_s: float = PATROL_TIMEOUT_S) -> None:
    """Spin the watcher until ``predicate(watcher)`` is true or the timeout elapses (caller owns rclpy)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate(watcher):
        rclpy.spin_once(watcher, timeout_sec=0.5)


def wait_for_subscription(node: Node, publisher, *, timeout_s: float = 10.0) -> bool:
    """Spin ``node`` until ``publisher`` has a matched subscription (DDS discovery), or timeout.

    Returns whether a subscriber was discovered. A *volatile* command publisher (e.g. /patrol/abort)
    drops samples published before discovery completes, so a test must wait for the node's subscriber
    to be matched rather than assume it — otherwise the abort can be silently lost (Hermes Medium).
    """
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and publisher.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    return publisher.get_subscription_count() > 0
