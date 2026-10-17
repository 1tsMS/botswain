"""Saved points: name + TCP pose + joints, stored as JSON next to the robot YAML (<robot>.points.json)."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from botswain.core.types import Joints, Pose


@dataclass(frozen=True)
class Point:
    name: str
    pose: Pose
    joints: Joints
    t_wall: str


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
        point = Point(self._unique(name), pose, joints, datetime.now().isoformat(timespec="seconds"))
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
        point = self.get(old)
        if new == old:
            return point
        renamed = Point(self._unique(new), point.pose, point.joints, point.t_wall)
        self._points[self._points.index(point)] = renamed
        self.save()
        return renamed

    def save(self) -> None:
        if self.path is None:
            return
        data = {"points": [{"name": p.name, "pose": p.pose.as_list(), "joints": p.joints.as_list(),
                            "t_wall": p.t_wall} for p in self._points]}
        tmp = self.path.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, indent=2), encoding="utf-8")
        os.replace(tmp, self.path)  # atomic: a crash mid-write never leaves half a file

    def _load(self, path: Path) -> None:
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            self._points = [Point(d["name"], Pose(*d["pose"]), Joints(d["joints"]), d.get("t_wall", ""))
                            for d in data["points"]]
        except (OSError, ValueError, KeyError, TypeError) as e:
            raise ValueError(f"cannot read saved points from {path}: {e}") from e

    def _exists(self, name: str) -> bool:
        return any(p.name == name for p in self._points)

    def _unique(self, name: str) -> str:
        if not self._exists(name):
            return name
        n = 2
        while self._exists(f"{name} ({n})"):
            n += 1
        return f"{name} ({n})"
