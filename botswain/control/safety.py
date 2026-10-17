"""SafetyGate: every command passes here, inside the worker, right before the driver.

check() looks at a command and the newest state and returns Allowed (maybe changed, plus the rules that
changed it) or Rejected (rule + a reason a human can read). It never talks to the robot.
Unknown command types are rejected: a new motion command must get its own rule before it can move anything.

Rules so far: ready check, speed cap. Box/joint limits (T4) and streaming rules (T5) plug in here.
"""

from __future__ import annotations

from dataclasses import dataclass

from botswain.core.config import LimitsCfg
from botswain.core.types import (
    ClearError,
    Command,
    Connect,
    Disable,
    Disconnect,
    Enable,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Mode,
    MoveTo,
    RobotState,
    SetSpeed,
    Stop,
    StreamTarget,
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
NON_MOTION = (Connect, Disconnect, Enable, Disable, ClearError, Stop, JogTick, JogRelease)

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
        modes = READY_MODES.get(type(cmd))
        if modes is None:
            return Rejected("unknown", f"no safety rule for {type(cmd).__name__}")
        reason = not_ready_reason(state, modes)
        if reason is not None:
            return Rejected("ready", reason)
        return Allowed(cmd)

    def _speed(self, cmd: SetSpeed) -> Verdict:
        cap = self.limits.speed_cap_pct
        if cmd.pct > cap:
            return Allowed(SetSpeed(cap), ("speed_cap",))
        if cmd.pct < 1:
            return Allowed(SetSpeed(1), ("speed_min",))
        return Allowed(cmd)

    def _count(self, rule: str) -> None:
        self.counts[rule] = self.counts.get(rule, 0) + 1
