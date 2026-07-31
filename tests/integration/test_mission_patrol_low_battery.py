"""SITL integration test: a mid-patrol LOW-BATTERY abort drives an observable RTH (exit item 12).

Grows the nightly SITL coverage beyond the two canonical missions (SWM-32): the external-signal
abort is already exercised in SITL (``test_mission_patrol.py``), but the **low-battery** abort guard
— the other *live* guard (state machine ``battery_low`` / AbortReason.LOW_BATTERY, config default
``abort.low_battery_threshold = 0.20``) — has until now been covered only by Layer-A unit tests.
Exit-checklist item 12 asks for mission abort to be "observable in a SITL run"; this makes the
low-battery half observable too, closing the SITL-observation gap the DoD flagged for AC-7.

**Trigger (in-band, no new deps, no PX4 param poking).** The node reads
``/fmu/out/battery_status`` (``BatteryStatus.remaining``, a 0..1 fraction; -1/disconnected =
unknown, which must NOT abort — Hermes High). We republish a fresh ``remaining = 0.05`` (well below
0.20), ``connected = True`` sample at spin cadence on that topic, with the *same* px4_qos the node
subscribes with (best-effort + transient-local). PX4's own ~full sample interleaves, but the node
caches the latest reading and checks the guard every tick, so a fresh sub-threshold sample makes the
guard fire; the abort then latches (sticks through RTH) exactly like the external abort.

**Attribution (F-09).** This used to be argued in prose — "the only live guards are external-signal
and low-battery, and this test injects no ``/patrol/abort``, so an observed ABORT is attributable to
the battery reading alone." A fair argument, but it was never an *assertion*: the test would still
have passed had the abort come from somewhere else, and the external and low-battery scenarios are
behaviorally indistinguishable from outside (identical flight profile, identical 24.4 s wall-clock).

It is now asserted, from two independent directions via ``AbortAttribution``:

* **positive** — the mission publishes its latched ``AbortReason`` on ``/patrol/abort_reason``, and
  this scenario requires it to read ``LOW_BATTERY``. ``mission_state`` carries the state but not the
  cause; that gap was the real defect behind the weak assertion, so it was closed rather than
  worked around.
* **negative** — the inbound ``/patrol/abort`` command count must be exactly zero, so an external
  signal is positively excluded rather than assumed absent.

The launch wiring, the underway-wait, and the ABORT -> RTH -> land recovery PASS/FAIL are shared
verbatim with the external-abort scenario (:mod:`patrol_launch`, :mod:`patrol_acceptance`) — only
the abort *trigger* differs.

Nightly SITL tier only — never a required per-PR check (OQ-5). Marked ``ros`` so the Layer-A unit
runner (no ROS) skips it. **Live-run status: PASSED 2026-07-26 on its first execution anywhere
(24.4 s) — the injection path works end to end and produces abort -> RTH -> settle -> disarm. Until
then this scenario had never run in CI, locally, or in the nightly (which had never been green), so
it shipped written-against-the-design and unproven.**
"""

import time

import launch_pytest
import pytest
import rclpy
from patrol_acceptance import (
    PATROL_TIMEOUT_S,
    AbortAttribution,
    PatrolWatcher,
    run_mid_patrol_abort_scenario,
    wait_for_subscription,
)
from patrol_launch import patrol_launch_description
from patrol_mission.qos import px4_qos
from px4_msgs.msg import BatteryStatus
from rclpy.node import Node

from patrol_mission import topics

pytestmark = pytest.mark.ros

# Injected remaining-capacity fraction — well below the config default low_battery_threshold (0.20,
# OQ-6). Comfortably clear of the threshold so ordinary SITL battery jitter can't mask the abort.
_LOW_BATTERY_FRACTION = 0.05


def _low_battery_msg() -> BatteryStatus:
    """A BatteryStatus reporting a valid, connected, sub-threshold charge.

    ``connected`` must be True: the node maps a disconnected battery to the -1 "unknown" sentinel
    that ``battery_low`` deliberately ignores (so a valid mission is not aborted before PX4 has an
    estimate), which would never fire the guard.
    """
    msg = BatteryStatus()
    msg.connected = True
    msg.remaining = _LOW_BATTERY_FRACTION
    return msg


def _spin_injecting_low_battery(
    watcher: PatrolWatcher,
    bat_pub,
    predicate,
    *,
    timeout_s: float = PATROL_TIMEOUT_S,
) -> None:
    """Spin the watcher, republishing a fresh low BatteryStatus each iteration, until ``predicate``.

    Republishing every iteration keeps the node's cached reading both low and unstale (the node
    forwards "unknown" once a reading ages past its 10 s budget), so the guard is guaranteed a fresh
    sub-threshold sample to fire on; once the abort latches, continued injection is harmless. Only the
    watcher is spun — the injector node just publishes (a publisher needs no spin; it has no
    subscriptions), so it is not passed here.
    """
    msg = _low_battery_msg()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not predicate(watcher):
        bat_pub.publish(msg)
        rclpy.spin_once(watcher, timeout_sec=0.1)


@launch_pytest.fixture
def low_battery_launch():
    # record:=false for determinism (no recorder in this scenario).
    return patrol_launch_description("false")


@pytest.mark.launch(fixture=low_battery_launch)
def test_low_battery_mid_patrol_drives_observable_rth() -> None:
    def inject(watcher: PatrolWatcher, injector: Node) -> None:
        # Confirm DDS matching before injecting — the node's battery subscriber must be discovered or
        # the sample is dropped (mirrors the external-abort test). The required set is the DEFAULT
        # (mission node only) and must stay that way: unlike /patrol/abort, this topic has exactly one
        # subscriber — PatrolWatcher takes vehicle_status + vehicle_local_position, NOT battery_status
        # — so requiring the watcher here would spin the full timeout and fail deterministically.
        bat_pub = injector.create_publisher(BatteryStatus, topics.BATTERY_STATUS, px4_qos())
        assert wait_for_subscription(injector, bat_pub), (
            "mission node's /fmu/out/battery_status subscriber was not discovered; the reading "
            "would be dropped"
        )
        # Inject the low battery until the full observable recovery is seen: an ABORT -> RTH, a
        # settle at home, and a disarm after arming.
        _spin_injecting_low_battery(
            watcher,
            bat_pub,
            lambda w: w.abort_then_rth and w.settled_near_home and w.disarmed_after_arm,
        )

    # Attribution (F-09): the mission must name LOW_BATTERY as the cause, AND no external abort
    # command may have been published — this scenario injects only a BatteryStatus sample, so a
    # non-zero /patrol/abort count would mean something else drove the transition.
    run_mid_patrol_abort_scenario(
        "low_battery_injector",
        inject,
        AbortAttribution(reason="LOW_BATTERY", external_cmds=0),
    )
