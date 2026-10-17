"""Core data types shared by every part of Botswain.

Units inside the app are mm and degrees (same as Dobot and the GUI). Everything is a frozen dataclass,
so states and commands can be handed between threads without copies or locks.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum

AXES_CART = ("X", "Y", "Z", "Rx", "Ry", "Rz")
AXES_JOINT = ("J1", "J2", "J3", "J4", "J5", "J6")
FRAMES = ("base", "tool")
MOVE_KINDS = ("joint", "linear")

_CART_FIELDS = ("x", "y", "z", "rx", "ry", "rz")


def is_joint_axis(name: str) -> bool:
    return name.startswith("J") and name[1:].isdigit()


def joint_index(name: str) -> int:
    """'J3' -> 2."""
    if not is_joint_axis(name):
        raise ValueError(f"not a joint axis: {name!r}")
    return int(name[1:]) - 1


def cart_index(name: str) -> int:
    """'Rx' -> 3."""
    if name not in AXES_CART:
        raise ValueError(f"not a Cartesian axis: {name!r}")
    return AXES_CART.index(name)


@dataclass(frozen=True)
class Pose:
    """TCP pose: x, y, z in mm, rx, ry, rz in degrees."""

    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float

    @property
    def xyz(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @property
    def rpy(self) -> tuple[float, float, float]:
        return (self.rx, self.ry, self.rz)

    def as_list(self) -> list[float]:
        return [self.x, self.y, self.z, self.rx, self.ry, self.rz]

    def axis(self, name: str) -> float:
        """Value of one axis by its GUI name ('X'..'Rz')."""
        return getattr(self, _CART_FIELDS[cart_index(name)])

    def with_axis(self, name: str, value: float) -> Pose:
        return replace(self, **{_CART_FIELDS[cart_index(name)]: float(value)})


@dataclass(frozen=True)
class Joints:
    """Joint angles in degrees, J1 first."""

    values: tuple[float, ...]

    def __post_init__(self) -> None:
        # Accept any sequence (list, numpy array) but always store an immutable tuple of floats.
        object.__setattr__(self, "values", tuple(float(v) for v in self.values))

    def __len__(self) -> int:
        return len(self.values)

    def as_list(self) -> list[float]:
        return list(self.values)

    def axis(self, name: str) -> float:
        return self.values[joint_index(name)]

    def with_axis(self, name: str, value: float) -> Joints:
        v = list(self.values)
        v[joint_index(name)] = float(value)
        return Joints(tuple(v))


class Mode(Enum):
    """Robot mode, the same for every brand. Each driver maps its own codes onto these."""

    DISCONNECTED = "disconnected"
    POWERED_OFF = "powered off"
    DISABLED = "disabled"
    IDLE = "idle"
    MOVING = "moving"
    JOGGING = "jogging"
    STREAMING = "streaming"
    DRAG = "drag"
    PAUSED = "paused"
    ERROR = "error"
    COLLISION = "collision"
    UNKNOWN = "unknown"


@dataclass(frozen=True)
class RobotState:
    """Newest known robot state. t_ns is when it arrived in our process (clock.now_ns)."""

    t_ns: int
    mode: Mode
    joints: Joints
    tcp: Pose
    joints_target: Joints
    tcp_target: Pose
    speed_pct: int
    errors: tuple[int, ...] = ()
    gripper: float | None = None  # 0 = closed, 1 = open, None = unknown / no gripper
    raw_mode: int = -1  # the brand's own mode code, shown in the GUI for debugging
    link_ok: bool = False
    feedback_hz: float = 0.0

    @staticmethod
    def disconnected(dof: int, t_ns: int = 0) -> RobotState:
        zero_j = Joints((0.0,) * dof)
        zero_p = Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return RobotState(t_ns, Mode.DISCONNECTED, zero_j, zero_p, zero_j, zero_p, 0)


@dataclass(frozen=True)
class Capabilities:
    """The "what I can do" card of a driver. The GUI hides what the robot can't do."""

    name: str
    dof: int
    step_jog: bool
    hold_jog: bool
    streaming: bool
    stream_rate_hz: float
    gripper: bool
    drag_mode: bool
    frames: tuple[str, ...]
    max_tcp_speed_mm_s: float


# ---- commands: the only things the GUI and inputs can put in the worker queue ----


@dataclass(frozen=True)
class Connect:
    pass


@dataclass(frozen=True)
class Disconnect:
    pass


@dataclass(frozen=True)
class Enable:
    pass


@dataclass(frozen=True)
class Disable:
    pass


@dataclass(frozen=True)
class ClearError:
    pass


@dataclass(frozen=True)
class Stop:
    pass


@dataclass(frozen=True)
class SetSpeed:
    pct: int


@dataclass(frozen=True)
class JogStep:
    """Relative step: axis 'X'..'Rz' (mm / deg) or 'J1'..'J6' (deg)."""

    axis: str
    delta: float
    frame: str = "base"

    def __post_init__(self) -> None:
        if self.frame not in FRAMES:
            raise ValueError(f"unknown frame {self.frame!r}")


@dataclass(frozen=True)
class JogHold:
    """Continuous jog until JogRelease (or the watchdog) stops it."""

    axis: str
    direction: int
    frame: str = "base"

    def __post_init__(self) -> None:
        if self.direction not in (-1, 1):
            raise ValueError(f"direction must be -1 or +1, got {self.direction}")
        if self.frame not in FRAMES:
            raise ValueError(f"unknown frame {self.frame!r}")


@dataclass(frozen=True)
class JogTick:
    """'Still held' heartbeat from the GUI during a hold-jog."""


@dataclass(frozen=True)
class JogRelease:
    pass


@dataclass(frozen=True)
class MoveTo:
    target: Pose | Joints
    kind: str  # "joint" or "linear"

    def __post_init__(self) -> None:
        if self.kind not in MOVE_KINDS:
            raise ValueError(f"kind must be one of {MOVE_KINDS}, got {self.kind!r}")
        if self.kind == "linear" and not isinstance(self.target, Pose):
            raise ValueError("a linear move needs a Pose target")


@dataclass(frozen=True)
class StreamTarget:
    """One teleop point. t_ns = when the input sample arrived; hold = the operator's hold-to-move button."""

    pose: Pose
    t_ns: int
    source: str
    hold: bool


@dataclass(frozen=True)
class Gripper:
    open: bool


@dataclass(frozen=True)
class Diagnose:
    """Run the driver's Doctor checks on the worker thread (it owns the sockets); results go to reply()."""

    reply: Callable[[list], None]


Command = (
    Connect | Disconnect | Enable | Disable | ClearError | Stop | SetSpeed
    | JogStep | JogHold | JogTick | JogRelease | MoveTo | StreamTarget | Gripper | Diagnose
)
