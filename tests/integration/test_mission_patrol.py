"""SITL integration test for the multi-waypoint patrol (AC-2, AC-6, MC-10).

Spins the mission node via ``mission_patrol.launch.py`` against a **real** PX4 SITL drone (Gazebo
Harmonic) over the uXRCE-DDS bridge, and asserts the observable patrol on ``/patrol/*``:

  * Nominal (AC-2): arm -> takeoff -> visit every configured waypoint (with dwell) -> RTH -> land.
    Run with ``record:=true`` passed explicitly (the launch default is ``false`` until 05 lands):
    since 05 (patrol_logging) is absent in CI, this also exercises the launch's resilient-include
    skip (TS-I3) — the patrol must still come up.
  * External abort (AC-6): an abort published on ``/patrol/abort`` mid-patrol drives an observable
    ABORT -> RTH (return home) then disarm. Run with ``record:=false`` for determinism.

The simulator is never mocked (tests/README): real SITL + the agent are brought up by the nightly
job before pytest runs; this test launches only the mission node and observes ``/patrol/*`` +
``/fmu/out/vehicle_status``. The launch wiring, the underway-wait, and the abort-recovery PASS/FAIL
are shared verbatim with the low-battery scenario (:mod:`patrol_launch`, :mod:`patrol_acceptance`)
so the scenarios can't drift — only the abort *trigger* differs.

Nightly SITL tier only — never a required per-PR check (OQ-5). Marked ``ros`` so the Layer-A unit
runner (no ROS) skips it.
"""

import launch_pytest
import pytest
import rclpy
from patrol_acceptance import (
    AbortAttribution,
    PatrolWatcher,
    evaluate_nominal,
    expected_waypoint_count,
    run_mid_patrol_abort_scenario,
    spin_until,
    wait_for_subscription,
)
from patrol_launch import patrol_launch_description
from patrol_mission.qos import patrol_abort_qos
from std_msgs.msg import Bool

from patrol_mission import topics

pytestmark = pytest.mark.ros


@launch_pytest.fixture
def patrol_launch():
    # Pass record:=true explicitly (launch default is false); 05 absent in CI -> resilient skip
    # (TS-I3), patrol still comes up.
    return patrol_launch_description("true")


@launch_pytest.fixture
def patrol_launch_no_record():
    return patrol_launch_description("false")


@pytest.mark.launch(fixture=patrol_launch)
def test_patrol_visits_all_waypoints_then_returns_home() -> None:
    rclpy.init()
    expected = expected_waypoint_count()
    watcher = PatrolWatcher(expected)
    try:
        spin_until(watcher, lambda w: w.nominal_complete)
        for check in evaluate_nominal(watcher, expected):
            assert check.passed, f"{check.name}: {check.detail}"
    finally:
        watcher.destroy_node()
        rclpy.shutdown()


@pytest.mark.launch(fixture=patrol_launch_no_record)
def test_external_abort_mid_patrol_drives_observable_rth() -> None:
    def inject(watcher: PatrolWatcher, injector) -> None:
        # patrol_abort_qos is reliable + *volatile*: a sample published before the node's subscriber
        # is discovered would be dropped, so wait for DDS matching first (Hermes Medium). Once
        # delivered the abort "sticks" through RTH via the state machine's latch.
        #
        # The wait establishes BOTH that the MISSION NODE specifically holds a subscriber on this
        # topic and that this publisher has matched one. Counting matched subscriptions alone is not
        # enough any more: since F-09 the watcher subscribes to /patrol/abort too, and it shares this
        # process with the publisher, so a count-only wait returns as soon as the WATCHER matches —
        # while the mission node may still be undiscovered (High #1).
        abort_pub = injector.create_publisher(Bool, topics.PATROL_ABORT, patrol_abort_qos())
        assert wait_for_subscription(injector, abort_pub), (
            "mission node's /patrol/abort subscriber was not discovered; the volatile abort would "
            "be dropped"
        )
        msg = Bool()
        msg.data = True
        abort_pub.publish(msg)
        # The mission must transition to an observable ABORT, then RTH, settle at home, then disarm.
        spin_until(
            watcher,
            lambda w: w.abort_then_rth and w.settled_near_home and w.disarmed_after_arm,
        )

    # Attribution (F-09): the mirror of the low-battery scenario — here exactly ONE external abort
    # command is published, and the mission must name EXTERNAL_SIGNAL as the cause. Pinning both
    # directions is what makes the pair meaningful: the two scenarios are otherwise behaviorally
    # indistinguishable from outside (identical profile, identical wall-clock).
    run_mid_patrol_abort_scenario(
        "abort_injector",
        inject,
        AbortAttribution(reason="EXTERNAL_SIGNAL", external_cmds=1),
    )
