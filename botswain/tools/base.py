"""End effectors (grippers and other tools), independent of the arm they sit on.

An arm driver offers a ToolPort (how bytes reach the tool: RS485 through the controller, a serial cable, ...).
A Tool driver speaks its device's protocol through any port of the kind it needs. So a DH gripper works on any
arm that offers an RS485 tool port, and the arm drivers know nothing about grippers.
The worker owns the tool: it connects it after the arm, sends it Gripper commands, and merges its status into
the robot state the GUI and the recorder see.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Protocol

from botswain.drivers.base import Check


class ToolPort(Protocol):
    kind: str  # "rs485"

    def exchange(self, frame: list[int]) -> list[int]:
        """Send one frame to the tool and return its reply bytes ([] if it did not answer)."""


@dataclass(frozen=True)
class ToolStatus:
    commanded_open: bool | None = None  # last open/close we sent, None before the first
    opening: float | None = None  # measured opening 0 (closed) .. 1 (open), None if the tool does not report
    text: str = ""  # the tool's own word ("object caught", "not initialised", ...)


@dataclass(frozen=True)
class ToolCaps:
    open_close: bool = True
    force: bool = False
    force_range_pct: tuple[int, int] = (0, 0)


class Tool(ABC):
    caps: ToolCaps
    port_kind: str  # the kind of ToolPort this tool needs

    @abstractmethod
    def connect(self, port: ToolPort) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def set_open(self, open: bool) -> None: ...

    def set_force(self, pct: int) -> None:
        from botswain.drivers.base import DriverError

        raise DriverError(-1, "this end effector has no force setting")

    @abstractmethod
    def status(self) -> ToolStatus: ...

    def diagnose(self) -> list[Check]:
        return []
