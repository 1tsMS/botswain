"""Forward and inverse kinematics from a URDF (yourdfpy). Units: mm and degrees outside, radians inside.

Pose convention: TCP = the last link's frame, Rx/Ry/Rz as static XYZ Euler angles ('sxyz').
IK is damped least squares on the geometric Jacobian: fine for a fake robot and for checks, not a planner.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np
from transforms3d.euler import euler2mat, mat2euler

from botswain.core.types import Pose

ROT_WEIGHT_MM = 200.0  # 1 rad of orientation error counts like 200 mm of position error


def pose_to_matrix(p: Pose) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = euler2mat(*np.radians(p.rpy), axes="sxyz")
    t[:3, 3] = p.xyz
    return t


def matrix_to_pose(t: np.ndarray) -> Pose:
    return Pose(*t[:3, 3], *np.degrees(mat2euler(t[:3, :3], axes="sxyz")))


def rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation matrix -> axis * angle (rad).

    atan2(sin, cos) instead of arccos(cos): arccos cannot see angles below ~1e-8 rad, which stalls IK.
    """
    s = 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])  # sin(angle) * axis
    sin_a = float(np.linalg.norm(s))
    cos_a = (float(np.trace(r)) - 1.0) / 2.0
    angle = float(np.arctan2(sin_a, cos_a))
    if sin_a > 1e-6 or cos_a > 0.0:
        return s * (angle / sin_a) if sin_a > 0.0 else np.zeros(3)
    # Near 180 deg the skew part vanishes: take the axis from the symmetric part.
    axis = np.sqrt(np.maximum((np.diag(r) + 1.0) / 2.0, 0.0))
    i = int(np.argmax(axis))
    axis = (r[:, i] + np.eye(3)[:, i]) / (2.0 * axis[i])
    return axis / np.linalg.norm(axis) * angle


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    k = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


class Kinematics:
    """Not thread-safe: fk() updates the shared URDF state. Use one instance per thread or a lock."""

    def __init__(self, urdf: Path) -> None:
        import yourdfpy

        self._u = yourdfpy.URDF.load(str(urdf), load_meshes=False, build_scene_graph=True)
        self.joint_names = list(self._u.actuated_joint_names)
        self._children = [self._u.joint_map[j].child for j in self.joint_names]
        self._axes = [np.asarray(self._u.joint_map[j].axis, dtype=float) for j in self.joint_names]

    def _frames(self, q_deg: Sequence[float]) -> list[np.ndarray]:
        self._u.update_cfg(np.radians(np.asarray(q_deg, dtype=float)))
        return [np.array(self._u.get_transform(c)) for c in self._children]

    def fk(self, q_deg: Sequence[float]) -> np.ndarray:
        """Tip transform with the translation in mm."""
        t = self._frames(q_deg)[-1]
        t[:3, 3] *= 1000.0
        return t

    def jacobian(self, q_deg: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
        frames = self._frames(q_deg)
        tip = frames[-1].copy()
        tip[:3, 3] *= 1000.0
        jac = np.zeros((6, len(frames)))
        for i, (f, axis) in enumerate(zip(frames, self._axes, strict=True)):
            z = f[:3, :3] @ axis
            jac[:3, i] = np.cross(z, tip[:3, 3] - f[:3, 3] * 1000.0)  # mm per rad
            jac[3:, i] = z
        return tip, jac

    def ik(self, target: np.ndarray, seed_deg: Sequence[float], max_iters: int = 40,
           tol_mm: float = 1e-5, tol_rad: float = 1e-7) -> np.ndarray | None:
        """Joint angles (deg) that put the tip at `target` (4x4, mm), starting from `seed_deg`. None if not found."""
        q = np.radians(np.asarray(seed_deg, dtype=float))
        for _ in range(max_iters):
            tip, jac = self.jacobian(np.degrees(q))
            e_p = target[:3, 3] - tip[:3, 3]
            e_r = rotvec(target[:3, :3] @ tip[:3, :3].T)
            if np.linalg.norm(e_p) < tol_mm and np.linalg.norm(e_r) < tol_rad:
                return np.degrees(q)
            err = np.concatenate([e_p, e_r * ROT_WEIGHT_MM])
            jac[3:] *= ROT_WEIGHT_MM
            lam = 0.5 * min(1.0, float(np.linalg.norm(err)) / 10.0)  # damping fades out near the solution
            dq = jac.T @ np.linalg.solve(jac @ jac.T + lam**2 * np.eye(6), err)
            biggest = float(np.max(np.abs(dq)))
            if biggest > 0.2:
                dq *= 0.2 / biggest
            q += dq
        tip = self.fk(np.degrees(q))
        if np.linalg.norm(target[:3, 3] - tip[:3, 3]) < 1e-4 and np.linalg.norm(
                rotvec(target[:3, :3] @ tip[:3, :3].T)) < 1e-6:
            return np.degrees(q)
        return None
