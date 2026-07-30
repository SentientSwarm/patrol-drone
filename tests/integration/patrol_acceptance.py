"""Shared patrol-mission acceptance criteria (AC-2 / AC-6) — single PASS/FAIL truth for CI + host.

Mirrors :mod:`mission_acceptance` (the M3 basic-mission harness) for the M4 multi-waypoint patrol.
Both consumers import this so "the patrol flew" / "the abort was observable" is defined in exactly
one place and the nightly SITL test and the host verifier can't drift:

  * the nightly SITL integration test -> ``tests/integration/test_mission_patrol.py``
  * the host-side verifier              -> ``scripts/verify_patrol.py`` (M4 UAT slice, SWM-40)

Layer-B: imports ``rclpy`` + ``px4_msgs`` + ``std_msgs``, so it is excluded from the Layer-A unit
runner and from mypy (pyproject), and ships to the nightly container via ``docker cp tests``. It
defines no ``test_*`` functions, so neither pytest tier collects it as a test.

The observable patrol surface is ``/patrol/*`` (OQ-3): ``mission_state`` (the MissionState name),
``current_waypoint`` (the active index), and ``dwell`` (the atomic OQ-7 capture trigger — one Int32
per DWELL entry carrying the dwelled waypoint identity). Arm/disarm comes from
``/fmu/out/vehicle_status`` (the same ``_v1`` output the basic harness reads). The acceptance criteria:

  AC-2 (nominal patrol): armed -> every configured waypoint index observed in DWELL (reached and
        dwelled for its configured dwell_s, not merely targeted) AND its atomic /patrol/dwell event
        fired -> RTH observed -> disarmed after arming.
  AC-6 (external abort): an external ``/patrol/abort`` published mid-patrol drives an observable
        ABORT then RTH, then disarm (asserted by the abort scenario in the test, using this watcher).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
import yaml

# AbortAttribution is re-exported: both scenario files import it from here, and it is used in this
# module's own signatures below, so it is a live reference rather than a bare re-export.
from abort_attribution import AbortAttribution, attribution_count_ok, latest_real_reason
from ament_index_python.packages import get_package_share_directory
from dwell_tracker import DwellTracker
from home_settle_tracker import HomeSettleTracker
from mission_acceptance import Check  # reuse the one Check verdict shape
from patrol_mission.frames import Point, to_ned_from_origin
from patrol_mission.qos import patrol_abort_qos, patrol_event_qos, patrol_state_qos, px4_qos
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from subscription_match import has_subscriber

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

# An observation gap longer than this breaks a continuous-hold window — both the home-settle hold AND
# a DWELL episode: a silence this long means observation was lost, so we cannot claim the vehicle
# stayed within tolerance of home (or held a waypoint) through it, and the hold must restart (mirrors
# mission_acceptance.MAX_SETTLE_SAMPLE_GAP_S / PR #8 post-mortem C). One constant for both trackers so
# the oracle never drifts. Sized above the nominal /fmu/out + /patrol sampling interval so ordinary
# jitter never trips it.
MAX_PATROL_SAMPLE_GAP_S = 1.0

# How long to wait for the patrol to get underway (arm + climb + reach the first leg) before
# injecting a mid-patrol abort trigger. Well under PATROL_TIMEOUT_S; the abort then drives the
# shorter return home. Shared by the external-abort and low-battery scenarios.
UNDERWAY_TIMEOUT_S = 150.0


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


def home_hold_time_s(mission_yaml: str | None = None) -> float:
    """The continuous in-tolerance hold (s) RTH must sustain at home before LANDING — the same
    ``completion.hold_time_s`` the state machine enforces via ``_within_tolerance_for_hold`` (OQ-4
    default 2.0). Config-driven so the return-home oracle proves the real hold, not a transient
    crossing (Hermes High), and never drifts from the flown criterion."""
    completion = _mission_raw(mission_yaml).get("completion") or {}
    return float(completion.get("hold_time_s", 2.0))


def dwell_required_s(mission_yaml: str | None = None) -> tuple[float, ...]:
    """Per-waypoint required dwell (s), index-aligned — what the patrol must HOLD at each waypoint
    before advancing (the state machine's ``waypoints[i].dwell_s``, AC-2/AC-9). Read from the same
    route YAML so the dwell oracle proves the configured hold, not mere DWELL entry (Hermes High)."""
    waypoints = _mission_raw(mission_yaml)["waypoints"]
    return tuple(float(w["dwell_s"]) for w in waypoints)


@dataclass(frozen=True)
class PatrolThresholds:
    """Config-derived thresholds the acceptance trackers compare against (AC-2/AC-6), grouped so the
    watcher's constructor stays narrow. Defaults read from the same checked-in route YAML the launch
    feeds the node, so the oracle never drifts from the flown criteria; tests can inject explicit values.
    """

    home_ned: Point  # the home coordinate RTH must settle at (EKF-origin-relative NED)
    home_tol_m: float  # completion-tolerance ball (m) for the home settle
    home_hold_s: float  # continuous in-tolerance hold (s) RTH must sustain before LANDING
    dwell_req_s: tuple[float, ...]  # per-waypoint required dwell (s), index-aligned

    @classmethod
    def from_mission_yaml(cls, mission_yaml: str | None = None) -> PatrolThresholds:
        """Read every threshold from one mission YAML (the launch's checked-in route, by default)."""
        return cls(
            home_ned=home_target_ned(mission_yaml),
            home_tol_m=home_tolerance_m(mission_yaml),
            home_hold_s=home_hold_time_s(mission_yaml),
            dwell_req_s=dwell_required_s(mission_yaml),
        )


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
        thresholds: PatrolThresholds | None = None,
        node_name: str = "patrol_acceptance_watcher",
    ):
        super().__init__(node_name)
        self._expected = expected_waypoints
        # The home coordinate RTH must settle at, the tolerance ball, the continuous hold it must sustain
        # there, and the per-waypoint required dwell — grouped in PatrolThresholds and read from the same
        # checked-in mission YAML the launch uses, so the oracle matches the flown criteria exactly.
        th = thresholds if thresholds is not None else PatrolThresholds.from_mission_yaml()
        self.home_ned = th.home_ned
        self.home_tol_m = th.home_tol_m
        self.home_hold_s = th.home_hold_s
        self.states_seen: list[str] = []  # deduped-consecutive ordered mission_state history
        self.waypoints_visited: set[int] = set()  # active targets seen (current_waypoint>=0)
        self.dwell_events: list[int] = []  # waypoint indices from the atomic /patrol/dwell event
        self.was_armed = False
        self.disarmed_after_arm = False
        # Abort ATTRIBUTION (F-09). mission_state says ABORT but not why, and the external and
        # low-battery guards fly an identical profile to the abort point — so a scenario that merely
        # observes an abort cannot claim its own trigger caused it. Two independent records:
        #   abort_reasons_seen  the mission's own latched answer, off /patrol/abort_reason
        #   external_abort_cmds inbound /patrol/abort commands, so a scenario that injects NO
        #                       external abort can assert the count is zero (the negative evidence
        #                       its docstring previously only argued for in prose)
        self.abort_reasons_seen: list[str] = []
        self.external_abort_cmds = 0
        # Return-home decision lives in a pure, Layer-A-tested tracker; the watcher feeds it timestamped
        # position samples gated on RTH having started, so the takeoff climb through the home altitude
        # can't falsely latch "returned home" before RTH runs, and only a continuous hold_time_s hold
        # within tolerance counts — not a transient crossing (see HomeSettleTracker / Hermes High).
        self._home_settle = HomeSettleTracker(
            th.home_ned, th.home_tol_m, th.home_hold_s, max_gap_s=MAX_PATROL_SAMPLE_GAP_S
        )
        # Dwell DURATION lives in a pure, Layer-A-tested tracker that counts DWELL *episodes* in the
        # per-topic-ordered mission_state stream ALONE — it never reads current_waypoint, so a
        # cross-topic reorder cannot false-count — credits a waypoint only once its episode has spanned
        # the configured dwell_s (not mere DWELL entry), and restarts that span on an observation gap
        # wider than max_gap_s so a blackout is never credited as dwell (see DwellTracker / Hermes).
        self._dwell = DwellTracker(th.dwell_req_s, max_gap_s=MAX_PATROL_SAMPLE_GAP_S)
        pqos = patrol_state_qos()
        self.create_subscription(String, topics.PATROL_MISSION_STATE, self._on_state, pqos)
        self.create_subscription(Int32, topics.PATROL_CURRENT_WAYPOINT, self._on_wp, pqos)
        # Dwell IDENTITY comes from the atomic /patrol/dwell event (one Int32 per DWELL entry): a
        # single message carries the dwelled waypoint, so attribution is race-free without correlating
        # the two non-atomic state topics. Acceptance requires both this and the duration proof above.
        self.create_subscription(Int32, topics.PATROL_DWELL, self._on_dwell, patrol_event_qos())
        # Latched, same profile as mission_state, so the reason is available even if the watcher
        # matched after the abort fired.
        self.create_subscription(String, topics.PATROL_ABORT_REASON, self._on_abort_reason, pqos)
        # The inbound command, on its own reliable+volatile profile (matching what a `ros2 topic pub`
        # or an injector node publishes with) — counted, never published, by the watcher.
        self.create_subscription(Bool, topics.PATROL_ABORT, self._on_abort_cmd, patrol_abort_qos())
        self.create_subscription(VehicleStatus, topics.VEHICLE_STATUS, self._on_status, px4_qos())
        self.create_subscription(
            VehicleLocalPosition, topics.VEHICLE_LOCAL_POSITION, self._on_local_pos, px4_qos()
        )

    def _on_state(self, msg: String) -> None:
        if not self.states_seen or self.states_seen[-1] != msg.data:
            self.states_seen.append(msg.data)
        # Dwell attribution counts DWELL episodes from this (ordered) state stream alone, timestamped so
        # the tracker can require each episode to span its configured dwell_s; see DwellTracker for why
        # current_waypoint must NOT participate (cross-topic reorder, Hermes High).
        self._dwell.on_state(msg.data, time.monotonic())

    def _on_wp(self, msg: Int32) -> None:
        if msg.data >= 0:
            self.waypoints_visited.add(msg.data)  # active targets (underway) — diagnostic only

    def _on_dwell(self, msg: Int32) -> None:
        # The atomic OQ-7 capture trigger: one event per DWELL entry carrying the dwelled waypoint
        # index. Race-free identity (a single message, no two-topic correlation), recorded in order.
        self.dwell_events.append(int(msg.data))

    def _on_abort_reason(self, msg: String) -> None:
        # Deduped-consecutive, like the mission_state history: the node republishes the latched
        # reason every progressing tick, so raw appends would be thousands of identical strings.
        if not self.abort_reasons_seen or self.abort_reasons_seen[-1] != msg.data:
            self.abort_reasons_seen.append(msg.data)

    def _on_abort_cmd(self, msg: Bool) -> None:
        # Only True is an abort command; a False sample is not a request and must not be counted.
        if msg.data:
            self.external_abort_cmds += 1

    @property
    def latched_abort_reason(self) -> str:
        """The mission's own answer for why it aborted; ``"NONE"`` if it never did.

        The last non-NONE reason observed — the machine latches the cause with the ABORT transition
        and it sticks through RTH, so this is stable once an abort has fired. The rule itself lives
        in the ROS-free ``abort_attribution`` module so it is Layer-A testable rather than reachable
        only through an SITL lane that cannot currently run.
        """
        return latest_real_reason(self.abort_reasons_seen)

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
        HomeSettleTracker owns that rule, the RTH->LANDING continuous-hold tolerance check, and the
        closest-approach diagnostic; the watcher just supplies (position, now_s, rth_started, valid).
        """
        self._home_settle.update(
            (float(msg.x), float(msg.y), float(msg.z)),
            time.monotonic(),
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
    def waypoints_dwelled(self) -> set[int]:
        """Indices reached AND held for the configured dwell_s — not just targeted (delegates to the
        tracker, which is duration-aware so a transient DWELL entry does not count, Hermes High)."""
        return self._dwell.dwelled

    @property
    def dwell_episodes(self) -> int:
        """DWELL episodes *started* (waypoints reached) — a diagnostic: if this exceeds the count of
        ``waypoints_dwelled``, some leg entered DWELL but did not hold its configured dwell_s."""
        return self._dwell.episodes

    @property
    def all_waypoints_dwelled(self) -> bool:
        """Every configured waypoint index was observed in DWELL — reached + dwelled, not just targeted."""
        return self.waypoints_dwelled >= set(range(self._expected))

    @property
    def all_checkpoint_events_seen(self) -> bool:
        """Every configured waypoint emitted its atomic /patrol/dwell event — race-free identity proof.

        The duration proof (``all_waypoints_dwelled``) is order-inferred from the state stream; this
        cross-checks that each waypoint *identity* 0..N-1 actually fired its one-shot capture trigger,
        so acceptance never passes on the state stream alone (Hermes High)."""
        return set(self.dwell_events) >= set(range(self._expected))

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
            and self.all_checkpoint_events_seen
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
            f"held configured dwell at waypoint indices {sorted(watcher.waypoints_dwelled)} "
            f"(need 0..{expected_waypoints - 1}); {watcher.dwell_episodes} DWELL episode(s) entered; "
            f"active targets seen {sorted(watcher.waypoints_visited)}",
        ),
        Check(
            "checkpoint_events",
            watcher.all_checkpoint_events_seen,
            f"atomic /patrol/dwell event fired for waypoint indices {watcher.dwell_events} "
            f"(need 0..{expected_waypoints - 1}) — race-free per-checkpoint capture trigger (OQ-7)",
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


def wait_until_underway(watcher: PatrolWatcher, *, timeout_s: float = UNDERWAY_TIMEOUT_S) -> None:
    """Spin until the patrol is underway (armed + past takeoff / a waypoint active), or timeout.

    The shared precondition for both mid-patrol abort scenarios: a trigger raised before the vehicle
    is airborne would not exercise the observable ABORT -> RTH -> land recovery.
    """
    spin_until(
        watcher,
        lambda w: w.was_armed and ("HOVER" in w.states_seen or bool(w.waypoints_visited)),
        timeout_s=timeout_s,
    )


def abort_recovery_checks(watcher: PatrolWatcher) -> list[Check]:
    """The shared observable-recovery checks after a mid-patrol abort (AC-6): an observable
    ABORT -> RTH, a settle within tolerance of the configured home, and a disarm after arming.

    Reused verbatim by the external-abort (AC-6) and the low-battery scenarios so "the abort was
    observable and the vehicle came home and landed" is defined in exactly one place — the abort
    *trigger* differs between scenarios, the observable recovery does not.
    """
    return [
        Check(
            "abort_then_rth",
            watcher.abort_then_rth,
            f"observable ABORT -> RTH; states seen: {watcher.states_seen}",
        ),
        Check(
            "settled_near_home",
            watcher.settled_near_home,
            f"abort-driven RTH settled within {watcher.home_tol_m} m of home_ned "
            f"{watcher.home_ned}; closest approach {watcher.min_home_distance_m:.2f} m",
        ),
        Check(
            "disarmed_after_arm",
            watcher.disarmed_after_arm,
            "vehicle disarmed after the abort-driven return home",
        ),
    ]


def abort_attribution_checks(watcher: PatrolWatcher, attribution: AbortAttribution) -> list[Check]:
    """The shared causal-attribution checks for a mid-patrol abort (F-09).

    Two independent lines of evidence, so neither alone has to carry the claim:

    * **positive** — the mission's own latched reason names the guard the scenario triggered;
    * **negative** — the count of inbound external-abort commands is exactly what this scenario
      published (zero for the low-battery scenario), so an external signal cannot be the cause.

    The negative half is what the low-battery test's docstring used to argue in prose: "the only
    live guards are external and low-battery, and this test injects no /patrol/abort, so the abort
    is attributable to the battery reading." A fair argument — but it was never an assertion, so
    the test would have passed had the abort come from anywhere else.
    """
    return [
        Check(
            "abort_reason_attributed",
            watcher.latched_abort_reason == attribution.reason,
            f"mission reported abort reason {watcher.latched_abort_reason!r}, expected "
            f"{attribution.reason!r}; reasons seen: {watcher.abort_reasons_seen}",
        ),
        Check(
            "external_abort_command_count",
            attribution_count_ok(watcher.external_abort_cmds, attribution.external_cmds),
            f"observed {watcher.external_abort_cmds} inbound /patrol/abort command(s), expected "
            f"{attribution.external_cmds} — a different count means the abort cannot be "
            f"attributed to this scenario's trigger",
        ),
    ]


def run_mid_patrol_abort_scenario(
    injector_name: str, inject, attribution: AbortAttribution
) -> None:
    """Run a mid-patrol abort scenario end to end: the shared rclpy lifecycle + watcher + underway
    wait + observable-recovery assertions, defined once so the external-abort and low-battery
    scenario files share everything but the *trigger* (and can't drift / duplicate scaffolding).

    ``inject(watcher, injector)`` is the scenario's only difference: it publishes the abort trigger
    on the ``injector`` node (an external ``/patrol/abort`` Bool, or a sub-threshold BatteryStatus on
    ``/fmu/out/battery_status``) and spins the watcher until the recovery predicate holds. On return,
    the observable ABORT -> RTH -> settle-at-home -> disarm is asserted via ``abort_recovery_checks``,
    and ``attribution`` pins that the abort had the *cause* this scenario triggered (F-09) rather
    than merely that some abort occurred.
    """
    rclpy.init()
    watcher = PatrolWatcher(expected_waypoint_count())
    injector = rclpy.create_node(injector_name)
    try:
        wait_until_underway(watcher)
        assert watcher.was_armed, "patrol never armed — cannot exercise the mid-patrol abort"
        inject(watcher, injector)
        checks = [*abort_recovery_checks(watcher), *abort_attribution_checks(watcher, attribution)]
        for check in checks:
            assert check.passed, f"{check.name}: {check.detail}"
    finally:
        injector.destroy_node()
        watcher.destroy_node()
        rclpy.shutdown()


def wait_for_subscription(node: Node, publisher, *, timeout_s: float = 10.0) -> bool:
    """Spin ``node`` until the MISSION NODE holds a discovered subscriber on ``publisher``'s topic
    AND ``publisher`` has matched at least one subscription, or timeout.

    Both halves are required and neither is sufficient alone:

    * **counting alone is what broke.** Since F-09 the watcher itself subscribes to /patrol/abort to
      count inbound commands, and it lives in THIS process while the mission node is separate — so it
      matches first and ``get_subscription_count() > 0`` is satisfied while the mission node is still
      undiscovered. The volatile Bool then publishes into nothing, no abort fires, and the scenario
      fails as an opaque PATROL_TIMEOUT_S timeout.
    * **graph presence alone is weaker than what it replaces.** An endpoint appears here as soon as
      discovery reports it, which is not the instant this publisher matches it, and it says nothing
      about QoS compatibility — precisely the guarantee an earlier review round added this wait for.

    Matching by node name asks the question the test actually means, and stays correct however many
    observers subscribe later — unlike an expected-count parameter, which encodes "exactly one other
    subscriber exists today" and silently breaks on the next one.

    The topic is read off ``publisher`` rather than passed alongside it, so the two can never
    disagree, and the mission node is the only subscriber this helper is ever asked about (both call
    sites) — so it is named from the shared constant rather than taken as an argument.
    """

    def ready() -> bool:
        return (
            has_subscriber(
                node.get_subscriptions_info_by_topic(publisher.topic_name),
                topics.MISSION_NODE_NAME,
            )
            and publisher.get_subscription_count() > 0
        )

    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not ready():
        rclpy.spin_once(node, timeout_sec=0.1)
    return ready()
