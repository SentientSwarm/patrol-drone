"""Pure-geometry visibility oracle for the SIM-4 checkpoint approach pose (F-01 lock).

Layer-A: ROS-free, stdlib-only, deterministic. For every canonical checkpoint it projects the
AprilTag's four readable-face corners through the camera at the *resolved hover pose* and asserts they
fall inside the camera's field of view. This is the capability the geometry must deliver — the drone
has to actually "see" the tag it flies to — which the asset-contract tests never assert; it is what
locks the approach-pose fix so a regression back to "hover on top of the tag" fails CI.

It composes the REAL code under test, so there is no second source of truth: the hover pose + yaw come
from ``config._approach_pose``, the tag size from ``gen_apriltag_models``, the canonical checkpoints
from the World Composer, and the camera intrinsics + mount are parsed from the shipped airframe SDF.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import pytest
from patrol_mission.config import Approach, _approach_pose

import compose_world as cw
import gen_apriltag_models as tags

Vec3 = tuple[float, float, float]

_REPO_ROOT = Path(__file__).resolve().parents[2]
_AIRFRAME_SDF = _REPO_ROOT / "sim" / "px4_sitl_overrides" / "gz_x500_patrol" / "model.sdf"
_STANDOFF_M = Approach().standoff_m


def _require(elem: ET.Element | None, what: str) -> ET.Element:
    if elem is None:
        raise AssertionError(f"airframe SDF missing element {what!r}")
    return elem


def _text(elem: ET.Element | None, what: str) -> str:
    found = _require(elem, what)
    if found.text is None:
        raise AssertionError(f"airframe SDF element {what!r} has no text")
    return found.text


@dataclass(frozen=True)
class Camera:
    """The camera contract parsed from the airframe SDF (single source of truth)."""

    mount: Vec3  # camera_link offset from base_link, in body frame (x fwd, y left, z up), meters
    pitch: float  # downward pitch, radians (rotation about body +Y)
    hfov: float  # horizontal field of view, radians
    width: int
    height: int
    near: float  # near clip plane, meters

    @property
    def half_hfov(self) -> float:
        return self.hfov / 2.0

    @property
    def half_vfov(self) -> float:
        # VFOV from HFOV and the image aspect (symmetric rectilinear projection, square pixels).
        return math.atan(math.tan(self.half_hfov) * self.height / self.width)


def _load_camera() -> Camera:
    root = ET.parse(_AIRFRAME_SDF).getroot()
    model = _require(root.find("model"), "model")
    link = _require(model.find("link[@name='camera_link']"), "link[camera_link]")
    pose = _text(link.find("pose"), "camera_link pose").split()
    mx, my, mz, _roll, pitch, _yaw = (float(v) for v in pose)
    cam = _require(link.find("sensor[@name='camera']/camera"), "sensor camera")
    return Camera(
        mount=(mx, my, mz),
        pitch=pitch,
        hfov=float(_text(cam.find("horizontal_fov"), "horizontal_fov")),
        width=int(_text(cam.find("image/width"), "image/width")),
        height=int(_text(cam.find("image/height"), "image/height")),
        near=float(_text(cam.find("clip/near"), "clip/near")),
    )


_CAMERA = _load_camera()
_CHECKPOINTS = cw.load_checkpoints()


def _dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _camera_basis(yaw: float, pitch: float) -> tuple[Vec3, Vec3, Vec3]:
    """Camera (forward, left, up) world axes for a level drone at ENU heading ``yaw`` (CCW from East)
    carrying a camera pitched ``pitch`` rad down about body +Y. gz cameras view down link +X, and a
    +pitch about +Y tilts +X downward — matching the ~20-deg-down mount.
    """
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    forward = (cy * cp, sy * cp, -sp)
    left = (-sy, cy, 0.0)
    up = (cy * sp, sy * sp, cp)
    return forward, left, up


def _camera_position(hover: Vec3, yaw: float, mount: Vec3) -> Vec3:
    """World camera position = hover + body->world(mount). Body x fwd, y left, z up; yaw about +Up."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    mx, my, mz = mount
    return (hover[0] + mx * cy - my * sy, hover[1] + mx * sy + my * cy, hover[2] + mz)


def _tag_corners(tag: Vec3, size: float) -> list[Vec3]:
    """The four readable-face corners of a zero-yaw tag (thin in Y, faces in the world XZ plane)."""
    tx, ty, tz = tag
    h = size / 2.0
    return [(tx + dx, ty, tz + dz) for dx in (-h, h) for dz in (-h, h)]


@pytest.mark.parametrize("cp", _CHECKPOINTS, ids=[c.checkpoint_id for c in _CHECKPOINTS])
def test_tag_in_frame_at_resolved_hover_pose(cp):
    tag: Vec3 = (cp.x, cp.y, cp.z)
    hover, yaw = _approach_pose(tag, _STANDOFF_M)
    cam_pos = _camera_position(hover, yaw, _CAMERA.mount)
    forward, left, up = _camera_basis(yaw, _CAMERA.pitch)

    for corner in _tag_corners(tag, tags.TAG_SIZE_M):
        v = (corner[0] - cam_pos[0], corner[1] - cam_pos[1], corner[2] - cam_pos[2])
        depth = _dot(v, forward)
        assert depth > _CAMERA.near  # in front of the camera, past the near clip
        az = math.atan2(_dot(v, left), depth)
        el = math.atan2(_dot(v, up), depth)
        # Strict in-frame containment. Worst case is the top corners: el ~= 25 deg vs ~27.2 deg
        # half-VFOV (the +20 deg mount pitch dominates the elevation, ~2 deg to spare). If a
        # mount / FOV / stand-off change pushes a corner past an edge, this fails — the F-01 lock.
        assert abs(az) < _CAMERA.half_hfov
        assert abs(el) < _CAMERA.half_vfov
