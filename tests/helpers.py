"""Shared test helpers (imported directly; pytest puts tests/ on sys.path)."""

from __future__ import annotations

import time
from collections.abc import Callable
from pathlib import Path

from botswain.core.clock import now_ns
from botswain.core.types import Joints, Mode, Pose, RobotState
from botswain.drivers.fake import FakeDriver

ROOT = Path(__file__).resolve().parents[1]
ROBOTS = ROOT / "robots"

SPIED = ("connect", "disconnect", "enable", "disable", "clear_error", "stop", "set_speed", "jog_step",
         "move_to", "jog_hold", "jog_release", "stream_target", "gripper")


def wait_for(pred: Callable[[], bool], timeout: float = 1.0, step: float = 0.002) -> bool:
    end = time.perf_counter() + timeout
    while time.perf_counter() < end:
        if pred():
            return True
        time.sleep(step)
    return pred()


def make_state(mode: Mode = Mode.IDLE, link_ok: bool = True, tcp: Pose | None = None,
               joints: tuple[float, ...] = (0, 0, 90, 0, -90, 0)) -> RobotState:
    tcp = tcp or Pose(-482, -136, 484, 180, 0, 0)
    j = Joints(joints)
    return RobotState(now_ns(), mode, j, tcp, j, tcp, 20, link_ok=link_ok, feedback_hz=125.0)


class SpyDriver(FakeDriver):
    """FakeDriver that records every contract call with its time, and can fail one call on demand."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.calls: list[tuple[str, int]] = []
        self.fail_next: dict[str, Exception] = {}

    def names(self) -> list[str]:
        return [name for name, _ in self.calls]

    def call_time(self, name: str) -> int:
        return next(t for n, t in self.calls if n == name)


def _spy(name: str):
    def method(self, *args, **kwargs):
        self.calls.append((name, now_ns()))
        if name in self.fail_next:
            raise self.fail_next.pop(name)
        return getattr(FakeDriver, name)(self, *args, **kwargs)

    method.__name__ = name
    return method


for _name in SPIED:
    setattr(SpyDriver, _name, _spy(_name))
