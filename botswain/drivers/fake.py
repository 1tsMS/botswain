"""Fake robot: pure Python, no network. Used for GUI work and most tests.

It keeps joint angles and a TCP pose and moves each toward its target at a capped speed every time
read_state() is called. Joints and pose are NOT kinematically linked (no IK): it is a stand-in, not a
simulator. The mode machine copies Dobot: starts DISABLED, enable -> IDLE, move -> MOVING -> IDLE,
alarm -> ERROR. Clearing an alarm leaves it DISABLED (the conservative guess; the real CR5 is checked Monday).
"""

from __future__ import annotations

import threading
import time

import numpy as np

from botswain.core.clock import now_ns
from botswain.core.config import RobotConfig
from botswain.core.types import (
    Capabilities,
    Joints,
    Mode,
    Pose,
    RobotState,
    cart_index,
    is_joint_axis,
    joint_index,
)
from botswain.drivers.base import Driver, DriverError

ROT_SPEED_FACTOR = 0.5  # rx/ry/rz move at this many deg/s per mm/s of linear speed


class FakeDriver(Driver):
    def __init__(self, cfg: RobotConfig, lin_speed_mm_s: float = 100.0, joint_speed_deg_s: float = 30.0) -> None:
        self.cfg = cfg
        self.lin_speed_mm_s = lin_speed_mm_s
        self.joint_speed_deg_s = joint_speed_deg_s
        self.capabilities = Capabilities(
            name=cfg.name, dof=cfg.dof, step_jog=True, hold_jog=True, streaming=True,
            stream_rate_hz=1.0 / cfg.stream.period_s, gripper=True, drag_mode=False,
            frames=("base", "tool"), max_tcp_speed_mm_s=lin_speed_mm_s,
        )
        # Start in the middle of the safety box, so jogging works right away.
        center = [(lo + hi) / 2 for lo, hi in cfg.limits.box_mm]
        self._tcp = np.array([*center, 180.0, 0.0, 0.0])
        self._joints = np.array(cfg.home_joints_deg or (0.0,) * cfg.dof, dtype=float)
        self._tcp_target = self._tcp.copy()
        self._joints_target = self._joints.copy()
        self._mode = Mode.DISCONNECTED
        self._speed_pct = 10
        self._errors: tuple[int, ...] = ()
        self._gripper: float | None = None
        self._hold: tuple[str, int] | None = None  # (axis, direction) while hold-jogging
        self._link_ok = False
        self._latency_s = 0.0
        self._last_ns = now_ns()
        self._calls_in_window = 0
        self._window_start_ns = now_ns()
        self._hz = 0.0
        # Test hooks are called from other threads than the worker.
        self._lock = threading.Lock()

    # ---- test hooks ----

    def inject_error(self, code: int) -> None:
        with self._lock:
            self._errors = (*self._errors, code)
            self._mode = Mode.ERROR
            self._halt()

    def drop_link(self) -> None:
        with self._lock:
            self._link_ok = False

    def restore_link(self) -> None:
        with self._lock:
            self._link_ok = self._mode is not Mode.DISCONNECTED

    def set_latency(self, ms: float) -> None:
        self._latency_s = ms / 1000.0

    # ---- contract ----

    def connect(self) -> None:
        self._wait_latency()
        with self._lock:
            if self._mode is Mode.DISCONNECTED:
                self._mode = Mode.DISABLED
            self._link_ok = True
            self._last_ns = now_ns()

    def disconnect(self) -> None:
        self.stop()
        with self._lock:
            self._mode = Mode.DISCONNECTED
            self._link_ok = False

    def read_state(self) -> RobotState:
        with self._lock:
            now = now_ns()
            if self._link_ok:
                self._advance(now)
            self._count_rate(now)
            j = Joints(tuple(self._joints))
            jt = Joints(tuple(self._joints_target))
            return RobotState(
                t_ns=now, mode=self._mode, joints=j, tcp=Pose(*self._tcp), joints_target=jt,
                tcp_target=Pose(*self._tcp_target), speed_pct=self._speed_pct, errors=self._errors,
                gripper=self._gripper, raw_mode=-1, link_ok=self._link_ok,
                feedback_hz=self._hz if self._link_ok else 0.0,
            )

    def enable(self) -> None:
        self._command()
        with self._lock:
            if self._mode is Mode.ERROR:
                raise DriverError(-2, "robot in alarm", "clear the error first")
            if self._mode in (Mode.DISABLED, Mode.POWERED_OFF):
                self._mode = Mode.IDLE

    def disable(self) -> None:
        self._command()
        with self._lock:
            self._halt()
            if self._mode is not Mode.ERROR:
                self._mode = Mode.DISABLED

    def clear_error(self) -> None:
        self._command()
        with self._lock:
            self._errors = ()
            if self._mode is Mode.ERROR:
                self._mode = Mode.DISABLED

    def stop(self) -> None:
        # No latency and no link check: stop must always work and never raise.
        with self._lock:
            self._halt()
            if self._mode in (Mode.MOVING, Mode.JOGGING, Mode.STREAMING):
                self._mode = Mode.IDLE

    def set_speed(self, pct: int) -> None:
        self._command()
        with self._lock:
            self._speed_pct = int(pct)

    def jog_step(self, axis: str, delta: float, frame: str) -> None:
        # Frame is ignored: without kinematics the fake has no tool frame, a tool step moves like a base step.
        self._command()
        with self._lock:
            self._require_mode(Mode.IDLE, Mode.MOVING)
            if is_joint_axis(axis):
                self._joints_target[joint_index(axis)] += delta
            else:
                self._tcp_target[cart_index(axis)] += delta
            self._mode = Mode.MOVING

    def move_to(self, target: Pose | Joints, kind: str) -> None:
        self._command()
        with self._lock:
            self._require_mode(Mode.IDLE)
            if isinstance(target, Pose):
                self._tcp_target = np.array(target.as_list())
            else:
                if len(target) != self.cfg.dof:
                    raise DriverError(-20000, f"expected {self.cfg.dof} joints, got {len(target)}")
                self._joints_target = np.array(target.as_list())
            self._mode = Mode.MOVING

    def jog_hold(self, axis: str, direction: int, frame: str) -> None:
        self._command()
        with self._lock:
            self._require_mode(Mode.IDLE)
            self._hold = (axis, direction)
            self._mode = Mode.JOGGING

    def jog_release(self) -> None:
        self._command()
        with self._lock:
            self._hold = None
            if self._mode is Mode.JOGGING:
                self._halt()
                self._mode = Mode.IDLE

    def stream_target(self, pose: Pose) -> None:
        self._command()
        with self._lock:
            self._require_mode(Mode.IDLE, Mode.STREAMING)
            self._tcp_target = np.array(pose.as_list())
            self._mode = Mode.STREAMING

    def gripper(self, open: bool) -> None:
        self._command()
        with self._lock:
            self._gripper = 1.0 if open else 0.0

    # ---- internals (call with the lock held) ----

    def _command(self) -> None:
        # Latency is outside the lock, like a network round trip that doesn't freeze the feedback.
        if self._mode is Mode.DISCONNECTED:
            raise DriverError(-100, "not connected", "press Connect")
        if not self._link_ok:
            raise DriverError(-100, "link lost", "check the cable")
        self._wait_latency()

    def _wait_latency(self) -> None:
        if self._latency_s > 0:
            time.sleep(self._latency_s)

    def _require_mode(self, *modes: Mode) -> None:
        if self._mode is Mode.ERROR:
            raise DriverError(-2, "robot in alarm", "clear the error first")
        if self._mode not in modes:
            raise DriverError(-1, f"robot is {self._mode.value}, command refused")

    def _halt(self) -> None:
        self._hold = None
        self._tcp_target = self._tcp.copy()
        self._joints_target = self._joints.copy()

    def _advance(self, now: int) -> None:
        dt = (now - self._last_ns) / 1e9
        self._last_ns = now
        if self._mode not in (Mode.MOVING, Mode.JOGGING, Mode.STREAMING):
            return
        # The controller's own speed factor does not slow streaming down (same as Dobot ServoP).
        scale = 1.0 if self._mode is Mode.STREAMING else self._speed_pct / 100.0
        lin = self.lin_speed_mm_s * scale * dt
        rot = self.lin_speed_mm_s * ROT_SPEED_FACTOR * scale * dt
        jnt = self.joint_speed_deg_s * scale * dt

        if self._hold is not None:
            axis, direction = self._hold
            if is_joint_axis(axis):
                self._joints[joint_index(axis)] += direction * jnt
            else:
                i = cart_index(axis)
                self._tcp[i] += direction * (lin if i < 3 else rot)
            self._tcp_target = self._tcp.copy()
            self._joints_target = self._joints.copy()
            return

        self._tcp[:3] = _step_toward(self._tcp[:3], self._tcp_target[:3], lin)
        self._tcp[3:] = _step_toward(self._tcp[3:], self._tcp_target[3:], rot)
        self._joints = _step_toward(self._joints, self._joints_target, jnt)
        arrived = np.array_equal(self._tcp, self._tcp_target) and np.array_equal(self._joints, self._joints_target)
        if arrived and self._mode is Mode.MOVING:
            self._mode = Mode.IDLE

    def _count_rate(self, now: int) -> None:
        self._calls_in_window += 1
        elapsed = now - self._window_start_ns
        if elapsed >= 1_000_000_000:
            self._hz = self._calls_in_window * 1e9 / elapsed
            self._calls_in_window = 0
            self._window_start_ns = now


def _step_toward(cur: np.ndarray, target: np.ndarray, max_step: float) -> np.ndarray:
    """Straight line from cur toward target, at most max_step long. Lands exactly on the target."""
    d = target - cur
    dist = float(np.linalg.norm(d))
    if dist <= max_step:
        return target.copy()
    return cur + d * (max_step / dist)
