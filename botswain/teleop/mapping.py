"""Phone motion -> robot target. Pure math, no threads, no robot.

On hold-down we store a phone anchor and a robot anchor (the measured pose), so the first target is exactly
where the robot is. After that: target = robot anchor + R_remap * (phone - phone anchor) * scale.
WebXR gives positions in RUB (x right, y up, z back); we turn them into FLU (x forward, y left, z up) first.
Rotation is off by default (scale_rot = 0) for first runs, like Phone2Act's first-run config.
"""

from __future__ import annotations

import numpy as np
from transforms3d.euler import euler2mat, mat2euler
from transforms3d.quaternions import quat2mat

from botswain.core.types import Pose

RUB_TO_FLU = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def remap_about_z(deg: float) -> np.ndarray:
    """Which way the operator faces: 0 = phone forward is robot +X, 90 = robot +Y, ..."""
    c, s = np.cos(np.radians(deg)), np.sin(np.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class TeleopMapper:
    def __init__(self, scale_pos: float = 0.3, scale_rot: float = 0.0, remap: np.ndarray | None = None) -> None:
        self.scale_pos = scale_pos
        self.scale_rot = scale_rot
        self.remap = np.eye(3) if remap is None else remap
        self._anchor: tuple[np.ndarray, np.ndarray, Pose] | None = None  # (phone pos FLU, phone rot FLU, robot)
        self._reanchor = False

    def request_reanchor(self) -> None:
        """Called after a teleport (tracking glitch): the next sample becomes the new anchor."""
        self._reanchor = True

    def update(self, position_m: tuple[float, float, float], quat_wxyz: tuple[float, float, float, float],
               hold: bool, phone_scale: float, robot_pose: Pose) -> Pose | None:
        """Target pose in mm/deg, or None while hold-to-move is up."""
        if not hold:
            self._anchor = None
            return None
        p = RUB_TO_FLU @ np.asarray(position_m, dtype=float)
        r = RUB_TO_FLU @ quat2mat(quat_wxyz) @ RUB_TO_FLU.T
        if self._anchor is None or self._reanchor:
            self._anchor = (p, r, robot_pose)
            self._reanchor = False
            return robot_pose
        p0, r0, robot0 = self._anchor
        d_mm = self.remap @ (p - p0) * 1000.0 * self.scale_pos * phone_scale
        x, y, z = np.array(robot0.xyz) + d_mm
        if self.scale_rot <= 0.0:
            return Pose(x, y, z, *robot0.rpy)
        rel = self.remap @ (r @ r0.T) @ self.remap.T  # phone rotation since the anchor, in robot axes
        angle, axis = _axis_angle(rel)
        rel_scaled = _from_axis_angle(axis, angle * self.scale_rot)
        robot_rot = rel_scaled @ euler2mat(*np.radians(robot0.rpy), axes="sxyz")
        rpy = np.degrees(mat2euler(robot_rot, axes="sxyz"))
        return Pose(x, y, z, *rpy)


def _axis_angle(m: np.ndarray) -> tuple[float, np.ndarray]:
    angle = float(np.arccos(np.clip((np.trace(m) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-9:
        return 0.0, np.array([1.0, 0.0, 0.0])
    axis = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]]) / (2.0 * np.sin(angle))
    return angle, axis / np.linalg.norm(axis)


def _from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)
