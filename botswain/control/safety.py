"""SafetyGate: every command passes here, inside the worker, right before the driver.

check() looks at a command and the newest state and returns Allowed (maybe changed, plus the rules that
changed it) or Rejected (rule + a reason a human can read). It never talks to the robot.
Unknown command types are rejected: a new motion command must get its own rule before it can move anything.

Rules: ready check, speed cap, workspace box (clamp steps, reject moves), joint limits (clamp steps, reject
moves), hold-jog limits, and for teleop streaming: hold-to-move, stale input, teleport guard, step clamp.
Streaming is the one stateful part: the gate remembers the last target it let through.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace

import numpy as np
from transforms3d.euler import euler2mat

from botswain.core.clock import now_ns
from botswain.core.config import LimitsCfg
from botswain.core.types import (
    ClearError,
    Command,
    Connect,
    Diagnose,
    Disable,
    Disconnect,
    Enable,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Joints,
    Mode,
    MoveTo,
    Pose,
    RequestControl,
    RobotState,
    SetSpeed,
    Stop,
    StreamTarget,
    cart_index,
    is_joint_axis,
    joint_index,
)


@dataclass(frozen=True)
class Allowed:
    cmd: Command
    clamped: tuple[str, ...] = ()


@dataclass(frozen=True)
class Rejected:
    rule: str
    reason: str


Verdict = Allowed | Rejected

# Commands that can't move the arm. Release/stop-type commands must always get through.
NON_MOTION = (Connect, Disconnect, Enable, Disable, ClearError, Stop, JogTick, JogRelease, Diagnose,
              RequestControl)

# Modes each motion command may start from. Idle means idle: MOVING and JOGGING are not idle
# (the old app treated "not error" as idle and re-sent moves on top of running ones).
READY_MODES: dict[type, frozenset[Mode]] = {
    JogStep: frozenset({Mode.IDLE}),
    JogHold: frozenset({Mode.IDLE}),
    MoveTo: frozenset({Mode.IDLE}),
    StreamTarget: frozenset({Mode.IDLE, Mode.STREAMING, Mode.MOVING}),
}


def not_ready_reason(state: RobotState, modes: frozenset[Mode]) -> str | None:
    """Why motion can't start in this state, or None if it can. The GUI uses it for button tooltips."""
    if state.mode is Mode.DISCONNECTED:
        return "not connected"
    if not state.link_ok:
        return "no live feedback from the robot"
    if state.mode in modes:
        return None
    if state.mode in (Mode.DISABLED, Mode.POWERED_OFF):
        return "robot not enabled"
    if state.mode in (Mode.ERROR, Mode.COLLISION):
        return "robot in error, clear it first"
    return f"robot busy ({state.mode.value})"


class SafetyGate:
    def __init__(self, limits: LimitsCfg, period_s: float) -> None:
        self.limits = limits
        self.period_s = period_s
        self.counts: dict[str, int] = {}
        self.box_enabled = limits.box_enabled  # switched from the GUI; joint limits are always on
        self._last_sent: Pose | None = None  # last streamed target that went to the robot
        self._last_raw: np.ndarray | None = None  # last streamed input position (for the teleport guard)

    def check(self, cmd: Command, state: RobotState) -> Verdict:
        verdict = self._check(cmd, state)
        if isinstance(verdict, Rejected):
            self._count(verdict.rule)
        else:
            for rule in verdict.clamped:
                self._count(rule)
        return verdict

    def _check(self, cmd: Command, state: RobotState) -> Verdict:
        if isinstance(cmd, NON_MOTION):
            return Allowed(cmd)
        if isinstance(cmd, SetSpeed):
            return self._speed(cmd)
        if isinstance(cmd, Gripper):
            # The gripper may be tested while the arm is disabled, but needs a live link.
            if state.mode is Mode.DISCONNECTED or not state.link_ok:
                return Rejected("ready", "not connected")
            return Allowed(cmd)
        bad = _non_finite(cmd)
        if bad:
            return Rejected("invalid", bad)
        modes = READY_MODES.get(type(cmd))
        if modes is None:
            return Rejected("unknown", f"no safety rule for {type(cmd).__name__}")
        reason = not_ready_reason(state, modes)
        if reason is not None:
            return Rejected("ready", reason)
        if isinstance(cmd, StreamTarget):
            return self._stream(cmd, state)
        if isinstance(cmd, JogStep):
            return self._jog_step(cmd, state)
        if isinstance(cmd, MoveTo):
            return self._move_to(cmd)
        if isinstance(cmd, JogHold):
            why = self.hold_violation(cmd, state, margin_mm=0.0, margin_deg=0.0)
            return Rejected(*why) if why else Allowed(cmd)
        return Allowed(cmd)

    # ---- streaming (teleop) ----

    def reset_stream(self, seed: Pose | None) -> None:
        """Start (seed = measured pose) or end (None) a stream. The first target can then never jump."""
        self._last_sent = seed
        self._last_raw = None

    def _stream(self, cmd: StreamTarget, state: RobotState) -> Verdict:
        if not cmd.hold:
            return Rejected("hold", "hold-to-move released")
        age_ms = (now_ns() - cmd.t_ns) / 1e6
        if age_ms > self.limits.stale_ms:
            return Rejected("stale", f"input sample is {age_ms:.0f} ms old")
        raw = np.array(cmd.pose.xyz)
        if self._last_raw is not None:
            jump = float(np.linalg.norm(raw - self._last_raw))
            if jump > self.limits.teleport_mm:
                self._last_raw = None  # the input re-anchors; the next sample starts a fresh comparison
                return Rejected("teleport", f"input jumped {jump:.0f} mm between two samples (tracking glitch)")
        self._last_raw = raw
        last = self._last_sent or state.tcp
        rules = []
        target, was = clamp_step(last, cmd.pose, self.limits.max_tcp_speed_mm_s * self.period_s,
                                 self.limits.max_step_deg)
        if was:
            rules.append("step")
        target, was = self._clamp_box(target)
        if was:
            rules.append("box")
        self._last_sent = target
        return Allowed(replace(cmd, pose=target), tuple(rules))

    def _clamp_box(self, pose: Pose) -> tuple[Pose, bool]:
        if not self.box_enabled:
            return pose, False
        xyz = [min(max(v, lo), hi) for v, (lo, hi) in zip(pose.xyz, self.limits.box_mm, strict=True)]
        if xyz == list(pose.xyz):
            return pose, False
        return replace(pose, x=xyz[0], y=xyz[1], z=xyz[2]), True

    # ---- workspace box + joint limits ----

    def _jog_step(self, cmd: JogStep, state: RobotState) -> Verdict:
        if is_joint_axis(cmd.axis):
            i = joint_index(cmd.axis)
            if i >= len(self.limits.joints_deg):
                return Rejected("joint_limit", f"{cmd.axis} does not exist on this robot")
            lo, hi = self.limits.joints_deg[i]
            cur = state.joints.values[i]
            target = min(max(cur + cmd.delta, lo), hi)
            if abs(target - cur) < 1e-9:
                return Rejected("joint_limit", f"{cmd.axis} is at its limit ({cur:.1f} in [{lo:g}, {hi:g}])")
            if abs(target - (cur + cmd.delta)) > 1e-9:
                return Allowed(replace(cmd, delta=target - cur), ("joint_limit",))
            return Allowed(cmd)
        i = cart_index(cmd.axis)
        if i >= 3 or not self.box_enabled:
            return Allowed(cmd)  # rotating about the TCP does not move the TCP
        d = np.zeros(3)
        d[i] = cmd.delta
        if cmd.frame == "tool":
            d = tool_rotation(state.tcp) @ d
        start = np.array(state.tcp.xyz)
        if not self._inside(start):
            # Started outside (e.g. the box was edited): only allow steps that bring the TCP back toward it.
            if self._distance_out(start + d) < self._distance_out(start):
                return Allowed(cmd)
            return Rejected("box", "TCP is outside the safety box; only moves back toward it are allowed")
        s = self._fraction_inside(start, d)
        if s <= 1e-6:
            return Rejected("box", f"already at the safety box wall ({cmd.axis} {'+' if cmd.delta > 0 else '-'})")
        if s < 1.0:
            return Allowed(replace(cmd, delta=cmd.delta * s), ("box",))
        return Allowed(cmd)

    def _move_to(self, cmd: MoveTo) -> Verdict:
        """Never silently go somewhere else: a target outside the limits is rejected with the reason."""
        if isinstance(cmd.target, Pose):
            if not self.box_enabled:
                return Allowed(cmd)
            for axis, v, (lo, hi) in zip("XYZ", cmd.target.xyz, self.limits.box_mm, strict=True):
                if not lo <= v <= hi:
                    return Rejected("box", f"target {axis} {v:.1f} mm is outside the safety box [{lo:g}, {hi:g}]")
            return Allowed(cmd)
        return self._joints_outside(cmd.target) or Allowed(cmd)

    def _joints_outside(self, joints: Joints) -> Rejected | None:
        if len(joints) != len(self.limits.joints_deg):
            return Rejected("joint_limit", f"expected {len(self.limits.joints_deg)} joints, got {len(joints)}")
        for i, (v, (lo, hi)) in enumerate(zip(joints.values, self.limits.joints_deg, strict=True)):
            if not lo <= v <= hi:
                return Rejected("joint_limit", f"J{i + 1} {v:.1f} deg is outside its limit [{lo:g}, {hi:g}]")
        return None

    def hold_violation(self, cmd: JogHold, state: RobotState, margin_mm: float = 1.0,
                       margin_deg: float = 0.5) -> tuple[str, str] | None:
        """(rule, reason) if this hold-jog must not start, or must be released now; else None.

        The worker calls it every tick while a hold-jog runs, with a small margin so the arm stops at the
        wall, not past it. Base-frame X/Y/Z and joints know their direction; a tool-frame move is stopped
        as soon as the TCP leaves the box.
        """
        if is_joint_axis(cmd.axis):
            lo, hi = self.limits.joints_deg[joint_index(cmd.axis)]
            v = state.joints.axis(cmd.axis)
            if (cmd.direction > 0 and v >= hi - margin_deg) or (cmd.direction < 0 and v <= lo + margin_deg):
                return ("joint_limit", f"{cmd.axis} reached its limit")
            return None
        i = cart_index(cmd.axis)
        if i >= 3 or not self.box_enabled:
            return None
        xyz = np.array(state.tcp.xyz)
        if cmd.frame == "base":
            lo, hi = self.limits.box_mm[i]
            if (cmd.direction > 0 and xyz[i] >= hi - margin_mm) or (cmd.direction < 0 and xyz[i] <= lo + margin_mm):
                return ("box", f"reached the safety box wall ({cmd.axis})")
            return None
        if not self._inside(xyz):
            return ("box", "TCP left the safety box")
        return None

    def _inside(self, xyz: np.ndarray) -> bool:
        return all(lo <= v <= hi for v, (lo, hi) in zip(xyz, self.limits.box_mm, strict=True))

    def _distance_out(self, xyz: np.ndarray) -> float:
        return float(sum(max(lo - v, 0.0, v - hi) for v, (lo, hi) in zip(xyz, self.limits.box_mm, strict=True)))

    def _fraction_inside(self, start: np.ndarray, d: np.ndarray) -> float:
        """Largest s in [0, 1] so that start + s*d stays in the box (start is inside)."""
        s = 1.0
        for v, dv, (lo, hi) in zip(start, d, self.limits.box_mm, strict=True):
            if dv > 0:
                s = min(s, (hi - v) / dv)
            elif dv < 0:
                s = min(s, (lo - v) / dv)
        return max(0.0, s)

    def _speed(self, cmd: SetSpeed) -> Verdict:
        cap = self.limits.speed_cap_pct
        if cmd.pct > cap:
            return Allowed(SetSpeed(cap), ("speed_cap",))
        if cmd.pct < 1:
            return Allowed(SetSpeed(1), ("speed_min",))
        return Allowed(cmd)

    def _count(self, rule: str) -> None:
        self.counts[rule] = self.counts.get(rule, 0) + 1


def tool_rotation(pose: Pose) -> np.ndarray:
    """Tool orientation as a 3x3 matrix: Dobot Rx/Ry/Rz as static XYZ Euler angles ('sxyz', like Phone2Act).

    Verify on the CR5 (SPEC 10): a tool-frame step must move along the tool axis DobotStudio shows.
    """
    return euler2mat(*np.radians(pose.rpy), axes="sxyz")


def clamp_step(last: Pose, target: Pose, max_mm: float, max_deg: float) -> tuple[Pose, bool]:
    """Move from `last` toward `target` by at most max_mm (straight line) and max_deg per rotation axis.

    Clamp, never drop: a fast phone move makes the arm follow slower, not freeze (the Phone2Act lesson).
    """
    d = np.array(target.xyz) - np.array(last.xyz)
    dist = float(np.linalg.norm(d))
    xyz = np.array(target.xyz) if dist <= max_mm else np.array(last.xyz) + d * (max_mm / dist)
    rpy = []
    for lo_v, hi_v in zip(last.rpy, target.rpy, strict=True):
        delta = (hi_v - lo_v + 180.0) % 360.0 - 180.0  # shortest way round
        rpy.append(lo_v + max(-max_deg, min(max_deg, delta)) if abs(delta) > max_deg else hi_v)
    clamped = dist > max_mm or any(r != t for r, t in zip(rpy, target.rpy, strict=True))
    return Pose(*xyz, *rpy), clamped


def _non_finite(cmd: Command) -> str | None:
    """NaN fails every comparison, so it would slip past the box and step limits: refuse it up front."""
    if isinstance(cmd, JogStep):
        values = [cmd.delta]
    elif isinstance(cmd, MoveTo):
        values = cmd.target.as_list()
    elif isinstance(cmd, StreamTarget):
        values = cmd.pose.as_list()
    else:
        return None
    return None if all(math.isfinite(v) for v in values) else f"{type(cmd).__name__} contains NaN or infinity"
