"""Saved points: name + TCP pose + joints, stored as JSON next to the robot config (<robot>.points.json).

A point remembers the pose and joints it was taught with. Edits in the Points table are limited to EDIT_LIMIT
from those taught values, so many small edits can never walk a point somewhere it was never taught. A point
also says how to get there when the points are run in order (joint or linear move) and how long to wait.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path

from botswain.core.types import Joints, Pose

EDIT_LIMIT_MM = 10.0  # an edited X/Y/Z may differ this much from the taught one
EDIT_LIMIT_DEG = 10.0  # an edited angle (Rx..Rz, J1..J6) may differ this much from the taught one
MOVE_KINDS = ("joint", "linear")


@dataclass(frozen=True)
class Point:
    name: str
    pose: Pose
    joints: Joints
    t_wall: str
    move: str = "joint"  # how Run all reaches it: "joint" (joint move to the joints) or "linear" (to the pose)
    wait_s: float = 0.0  # pause after arriving, when run in order
    taught_pose: Pose | None = None  # what was taught (edit limits are measured from these)
    taught_joints: Joints | None = None

    @property
    def origin_pose(self) -> Pose:
        return self.taught_pose or self.pose

    @property
    def origin_joints(self) -> Joints:
        return self.taught_joints or self.joints


class PointStore:
    """In memory, saved on every change. path=None keeps points in memory only (used when the file is bad)."""

    def __init__(self, path: Path | None) -> None:
        self.path = path
        self._points: list[Point] = []
        if path is not None and path.exists():
            self._load(path)

    def all(self) -> list[Point]:
        return list(self._points)

    def get(self, name: str) -> Point:
        for p in self._points:
            if p.name == name:
                return p
        raise KeyError(name)

    def next_name(self) -> str:
        n = len(self._points) + 1
        while self._exists(f"P{n}"):
            n += 1
        return f"P{n}"

    def add(self, name: str, pose: Pose, joints: Joints) -> Point:
        point = Point(self._unique(name), pose, joints, datetime.now().isoformat(timespec="seconds"),
                      taught_pose=pose, taught_joints=joints)
        self._points.append(point)
        self.save()
        return point

    def remove(self, name: str) -> None:
        self._points = [p for p in self._points if p.name != name]
        self.save()

    def rename(self, old: str, new: str) -> Point:
        new = new.strip()
        if not new:
            raise ValueError("a point needs a name")
        if " " in new:
            raise ValueError("no spaces in a point name (Space is STOP)")
        point = self.get(old)
        if new == old:
            return point
        return self._replace(point, replace(point, name=self._unique(new)))

    def update(self, name: str, *, pose: Pose | None = None, joints: Joints | None = None,
               move: str | None = None, wait_s: float | None = None) -> Point:
        """Change a point. The caller checks the edit limits (it knows the kinematics); this checks the rest."""
        point = self.get(name)
        if move is not None and move not in MOVE_KINDS:
            raise ValueError(f"move must be one of {MOVE_KINDS}, got {move!r}")
        if wait_s is not None and not 0.0 <= wait_s <= 600.0:
            raise ValueError(f"wait must be 0..600 s, got {wait_s}")
        changes = {k: v for k, v in (("pose", pose), ("joints", joints), ("move", move), ("wait_s", wait_s))
                   if v is not None}
        return self._replace(point, replace(point, **changes))

    def move_to_index(self, name: str, index: int) -> None:
        point = self.get(name)
        self._points.remove(point)
        self._points.insert(max(0, min(index, len(self._points))), point)
        self.save()

    def save(self) -> None:
        if self.path is None:
            return
        data = {"points": [_to_json(p) for p in self._points]}
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
        os.replace(tmp, self.path)  # atomic: a crash mid-write never leaves half a file

    def _replace(self, old: Point, new: Point) -> Point:
        self._points[self._points.index(old)] = new
        self.save()
        return new

    def _load(self, path: Path) -> None:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self._points = [_from_json(d) for d in data["points"]]
        except (OSError, ValueError, KeyError, TypeError) as e:
            raise ValueError(f"cannot read saved points from {path}: {e}") from e

    def _exists(self, name: str) -> bool:
        return any(p.name == name for p in self._points)

    def _unique(self, name: str) -> str:
        if not self._exists(name):
            return name
        n = 2
        while self._exists(f"{name}_{n}"):
            n += 1
        return f"{name}_{n}"


def _to_json(p: Point) -> dict:
    return {"name": p.name, "pose": p.pose.as_list(), "joints": p.joints.as_list(), "t_wall": p.t_wall,
            "move": p.move, "wait_s": p.wait_s, "taught_pose": p.origin_pose.as_list(),
            "taught_joints": p.origin_joints.as_list()}


def _from_json(d: dict) -> Point:
    """Older files have no move / wait / taught values: the saved pose and joints are what was taught."""
    pose, joints = Pose(*d["pose"]), Joints(d["joints"])
    move = d.get("move", "joint")
    if move not in MOVE_KINDS:
        raise ValueError(f"point {d['name']}: move {move!r}")
    return Point(d["name"], pose, joints, d.get("t_wall", ""), move=move, wait_s=float(d.get("wait_s", 0.0)),
                 taught_pose=Pose(*d["taught_pose"]) if "taught_pose" in d else pose,
                 taught_joints=Joints(d["taught_joints"]) if "taught_joints" in d else joints)
