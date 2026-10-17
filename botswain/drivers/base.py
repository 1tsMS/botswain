"""The driver contract. A driver is a travel adapter: every robot brand answers the same buttons.

Only the worker thread calls a driver, and only after the SafetyGate said yes. Drivers raise DriverError
with a hint for the operator; they never swallow errors.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal

from botswain.core.types import Capabilities, Joints, Pose, RobotState


class DriverError(Exception):
    def __init__(self, code: int, message: str, hint: str = "") -> None:
        super().__init__(f"[{code}] {message}" + (f" ({hint})" if hint else ""))
        self.code = code
        self.message = message
        self.hint = hint


@dataclass(frozen=True)
class Check:
    """One Doctor result."""

    name: str
    status: Literal["ok", "warn", "fail"]
    value: str
    hint: str = ""


class Driver(ABC):
    capabilities: Capabilities

    @abstractmethod
    def connect(self) -> None:
        """Open links and start feedback. Never enables the robot."""

    @abstractmethod
    def disconnect(self) -> None:
        """Stop motion first, then close every link."""

    @abstractmethod
    def read_state(self) -> RobotState:
        """Newest state. Must not block (anything over 1 ms is a bug)."""

    @abstractmethod
    def enable(self) -> None: ...

    @abstractmethod
    def disable(self) -> None: ...

    @abstractmethod
    def clear_error(self) -> None: ...

    @abstractmethod
    def stop(self) -> None:
        """Stop all motion. Works in any mode, never raises just because nothing was moving."""

    @abstractmethod
    def set_speed(self, pct: int) -> None: ...

    @abstractmethod
    def jog_step(self, axis: str, delta: float, frame: str) -> None:
        """Relative step: 'X'..'Rz' in mm / deg, or 'J1'.. in deg."""

    @abstractmethod
    def move_to(self, target: Pose | Joints, kind: str) -> None:
        """Returns when the robot accepted the move, not when it finished."""

    # Optional: only called when the capability card says so.

    def jog_hold(self, axis: str, direction: int, frame: str) -> None:
        raise DriverError(-1, "hold jog not supported by this driver")

    def jog_release(self) -> None:
        raise DriverError(-1, "hold jog not supported by this driver")

    def stream_target(self, pose: Pose) -> None:
        raise DriverError(-1, "streaming not supported by this driver")

    def gripper(self, open: bool) -> None:
        raise DriverError(-1, "no gripper on this driver")

    def diagnose(self) -> list[Check]:
        """Driver-specific Doctor checks (ports, packet rate...)."""
        return []
