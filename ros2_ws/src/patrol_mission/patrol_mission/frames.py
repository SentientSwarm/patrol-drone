"""The single ENU<->NED coordinate-frame conversion site (design §4.2.4, MC-7).

This is the ONLY place world/YAML waypoints are converted to PX4 NED. A second
conversion site anywhere in the codebase is review-rejectable (Tenet 4): silent,
infuriating frame bugs come from conversions sprinkled across the call graph.

ROS-free and pure: no rclpy, no I/O. Exercised entirely by Layer-A unit tests.
"""

from __future__ import annotations

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
