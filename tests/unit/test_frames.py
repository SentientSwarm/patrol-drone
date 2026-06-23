"""Unit tests for the single ENU<->NED conversion boundary (design §4.2.4, MC-7).

Layer-A: ROS-free, deterministic. No simulator, no rclpy.
"""

import math

import pytest
from patrol_mission.frames import (
    enu_yaw_to_ned,
    takeoff_target_ned,
    to_enu_from_ned,
    to_ned_from_origin,
)


# TS-8: ENU -> NED axis map for a known input, with a zero origin.
def test_enu_to_ned_axis_map_zero_origin():
    # ENU (east=1, north=2, up=3) -> NED (north=2, east=1, down=-3)
    assert to_ned_from_origin((1.0, 2.0, 3.0), "enu", (0.0, 0.0, 0.0)) == (2.0, 1.0, -3.0)


# TS-8: ENU -> NED adds the EKF-origin NED offset.
def test_enu_to_ned_applies_origin_offset():
    # origin NED offset (10, 20, 30) is added after the axis map
    assert to_ned_from_origin((1.0, 2.0, 3.0), "enu", (10.0, 20.0, 30.0)) == (12.0, 21.0, 27.0)


# TS-8: a typical "up" ENU waypoint becomes negative "down" in NED.
def test_enu_up_becomes_negative_down():
    _, _, down = to_ned_from_origin((0.0, 0.0, 5.0), "enu", (0.0, 0.0, 0.0))
    assert down == -5.0


# TS-8: NED frame is a passthrough plus origin offset (no axis swap).
def test_ned_passthrough_plus_origin():
    assert to_ned_from_origin((1.0, 2.0, 3.0), "ned", (0.0, 0.0, 0.0)) == (1.0, 2.0, 3.0)
    assert to_ned_from_origin((1.0, 2.0, 3.0), "ned", (10.0, 20.0, 30.0)) == (11.0, 22.0, 33.0)


# TS-8: unknown frame fails loud (Tenet 4 — no silent default).
def test_unknown_frame_raises():
    with pytest.raises(ValueError, match="unknown frame"):
        to_ned_from_origin((0.0, 0.0, 0.0), "lla", (0.0, 0.0, 0.0))


# TS-8: frame string is case-/typo-sensitive — only exact 'enu'/'ned' accepted.
def test_frame_is_exact_match():
    with pytest.raises(ValueError, match="unknown frame"):
        to_ned_from_origin((0.0, 0.0, 0.0), "ENU", (0.0, 0.0, 0.0))


def test_returns_plain_float_tuple():
    result = to_ned_from_origin((1, 2, 3), "enu", (0, 0, 0))
    assert all(isinstance(c, float) for c in result)
    assert not any(math.isnan(c) for c in result)


# --- to_enu_from_ned: the inverse axis map at the single MC-7 site (T B.2, PoseSampler/ADR-B) ---


# NED (north, east, down) -> ENU (east, north, up); zero origin.
def test_ned_to_enu_axis_map_zero_origin():
    # NED (north=2, east=1, down=-3) -> ENU (east=1, north=2, up=3)
    assert to_enu_from_ned((2.0, 1.0, -3.0), (0.0, 0.0, 0.0)) == (1.0, 2.0, 3.0)


# The EKF-origin NED offset is subtracted before the axis swap (origin-relative -> world ENU).
def test_ned_to_enu_subtracts_origin_offset():
    # origin NED (10, 20, 30): NED (12, 21, 27) -> rel NED (2, 1, -3) -> ENU (1, 2, 3)
    assert to_enu_from_ned((12.0, 21.0, 27.0), (10.0, 20.0, 30.0)) == (1.0, 2.0, 3.0)


# NED "down" becomes positive ENU "up".
def test_ned_down_becomes_positive_up():
    _, _, up = to_enu_from_ned((0.0, 0.0, -5.0), (0.0, 0.0, 0.0))
    assert up == 5.0


# Round-trips exactly with to_ned_from_origin (the two halves of the one MC-7 boundary).
@pytest.mark.parametrize(
    "enu",
    [(0.0, 0.0, 0.0), (1.0, 2.0, 3.0), (-4.5, 6.0, -2.0), (10.0, -10.0, 5.0)],
)
@pytest.mark.parametrize("origin_ned", [(0.0, 0.0, 0.0), (10.0, 20.0, 30.0)])
def test_enu_ned_round_trip(enu, origin_ned):
    ned = to_ned_from_origin(enu, "enu", origin_ned)
    assert to_enu_from_ned(ned, origin_ned) == pytest.approx(enu)


def test_to_enu_returns_plain_float_tuple():
    result = to_enu_from_ned((1, 2, 3), (0, 0, 0))
    assert all(isinstance(c, float) for c in result)


# takeoff_target_ned keeps home x/y and climbs takeoff_alt_m above home (NED down decreases).
@pytest.mark.parametrize(
    ("home_ned", "alt", "expected"),
    [
        ((0.0, 0.0, 0.0), 5.0, (0.0, 0.0, -5.0)),  # home on the ground plane -> -alt
        ((0.0, 0.0, -2.0), 5.0, (0.0, 0.0, -7.0)),  # home 2 m up (shipped config) -> -7, NOT -alt
        ((1.0, 2.0, -3.0), 4.0, (1.0, 2.0, -7.0)),  # x/y preserved; down is home_down - alt
    ],
    ids=["home_at_origin", "home_above_origin", "home_offset_xy"],
)
def test_takeoff_target_is_alt_above_home(home_ned, alt, expected):
    assert takeoff_target_ned(home_ned, alt) == expected


# Returns a plain float tuple even for int inputs (mirrors to_ned_from_origin's contract).
def test_takeoff_target_returns_plain_float_tuple():
    result = takeoff_target_ned((1, 2, 3), 4)
    assert all(isinstance(c, float) for c in result)


# TS-SIM4: ENU yaw (CCW from East) -> NED yaw (CW from North): yaw_ned = pi/2 - yaw_enu, wrapped.
@pytest.mark.parametrize(
    ("yaw_enu", "expected_ned"),
    [
        (0.0, math.pi / 2),  # ENU East -> NED faces East (90 deg CW from North)
        (math.pi / 2, 0.0),  # ENU North -> NED faces North (0)
        (-math.pi / 2, math.pi),  # ENU South -> NED faces South (180)
        (math.pi, -math.pi / 2),  # ENU West -> NED faces West (-90)
    ],
    ids=["east", "north", "south", "west"],
)
def test_enu_yaw_to_ned_cardinals(yaw_enu, expected_ned):
    assert enu_yaw_to_ned(yaw_enu) == pytest.approx(expected_ned)


# The result is always normalized to the half-open (-pi, pi] regardless of input magnitude.
@pytest.mark.parametrize("yaw_enu", [-100.0, -10.0, -math.pi, 0.0, math.pi, 7.5, 100.0])
def test_enu_yaw_to_ned_wraps_to_pi(yaw_enu):
    result = enu_yaw_to_ned(yaw_enu)
    assert -math.pi < result <= math.pi
