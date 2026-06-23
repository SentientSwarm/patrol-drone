"""The single ENU<->NED coordinate-frame conversion site (design §4.2.4, MC-7).

This is the ONLY place world/YAML waypoints are converted to PX4 NED. A second
conversion site anywhere in the codebase is review-rejectable (Tenet 4): silent,
infuriating frame bugs come from conversions sprinkled across the call graph.

ROS-free and pure: no rclpy, no I/O. Exercised entirely by Layer-A unit tests.
"""

from __future__ import annotations

import math

Point = tuple[float, float, float]


def to_ned_from_origin(point: Point, frame: str, ekf_origin_ned: Point) -> Point:
    """Convert a world/YAML waypoint to PX4 NED relative to the EKF origin (MC-7).

    Args:
        point: the source point in ``frame`` coordinates (meters).
        frame: ``"enu"`` (East-North-Up, the world/YAML convention) or ``"ned"``
            (already North-East-Down relative to the origin).
        ekf_origin_ned: the EKF origin expressed as a NED offset to add, so the
            result is origin-relative.

    Returns:
        The point in NED relative to the EKF origin, as a plain ``float`` tuple.

    Raises:
        ValueError: on any frame other than ``"enu"`` / ``"ned"`` (fail loud —
            no silent default frame).

    ENU->NED axis map: ``(x_e, y_n, z_u) -> (y_n, x_e, -z_u)``, then add the
    EKF-origin NED offset.
    """
    ox, oy, oz = (float(c) for c in ekf_origin_ned)
    if frame == "ned":
        x, y, z = (float(c) for c in point)
        return (x + ox, y + oy, z + oz)
    if frame == "enu":
        xe, ye, ze = (float(c) for c in point)
        return (ye + ox, xe + oy, -ze + oz)
    raise ValueError(f"unknown frame {frame!r}: expected 'ned' or 'enu'")


def takeoff_target_ned(home_ned: Point, takeoff_alt_m: float) -> Point:
    """The takeoff/hover setpoint: ``takeoff_alt_m`` above home, in EKF-origin NED.

    NED "down" increases downward, so "altitude above home" subtracts from home's own down
    coordinate — correct for *any* home altitude, not just home sitting on the EKF-origin ground
    plane. The state machine flies to this point, and the basic-mission acceptance harness derives
    its settle band from it, so both share this one derivation rather than each hardcoding
    ``-takeoff_alt_m`` (which silently drifts the moment home is not at z=0; Hermes).
    """
    hx, hy, hz = (float(c) for c in home_ned)
    return (hx, hy, hz - float(takeoff_alt_m))


def _wrap_to_pi(angle: float) -> float:
    """Normalize a radian angle to (-pi, pi]."""
    wrapped = math.remainder(angle, 2.0 * math.pi)  # IEEE remainder -> [-pi, pi]
    return math.pi if wrapped == -math.pi else wrapped  # fold the open end up to +pi


def enu_yaw_to_ned(yaw_enu: float) -> float:
    """Convert an ENU yaw (CCW from East about +Up) to PX4 NED yaw (CW from North about +Down).

    The rotational analogue of :func:`to_ned_from_origin`'s axis map, kept at this single MC-7
    boundary so heading conversions never sprinkle across the call graph (Tenet 4). ENU yaw 0 faces
    East; NED yaw 0 faces North; the two run in opposite senses, so ``yaw_ned = pi/2 - yaw_enu``,
    normalized to (-pi, pi].
    """
    return _wrap_to_pi(math.pi / 2.0 - yaw_enu)
