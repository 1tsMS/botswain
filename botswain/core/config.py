"""Load and validate a robot YAML into dataclasses.

Every limit must be present and sane. A missing or misspelt limit raises ConfigError naming the field.
There are no default limits on purpose: a typo must never silently widen the workspace.
Relative paths in the YAML (the URDF) are relative to the YAML file's folder.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


class ConfigError(Exception):
    def __init__(self, field: str, message: str) -> None:
        super().__init__(f"{field}: {message}")
        self.field = field
        self.message = message


@dataclass(frozen=True)
class ConnectionCfg:
    ip: str
    dashboard_port: int = 29999
    motion_port: int = 29999
    feedback_port: int = 30004
    http_port: int = 22000


@dataclass(frozen=True)
class LimitsCfg:
    speed_cap_pct: int
    box_mm: tuple[tuple[float, float], tuple[float, float], tuple[float, float]]  # (min, max) for x, y, z
    joints_deg: tuple[tuple[float, float], ...]  # (min, max) per joint
    max_tcp_speed_mm_s: float
    max_step_deg: float
    teleport_mm: float
    stale_ms: float
    box_enabled: bool = False  # the GUI can switch the box on and off at runtime


@dataclass(frozen=True)
class StreamCfg:
    period_s: float = 0.03
    aheadtime: float = 50.0
    gain: float = 500.0


@dataclass(frozen=True)
class GripperCfg:
    type: str
    slave: int = 1
    baud: int = 115200


@dataclass(frozen=True)
class RobotConfig:
    name: str
    driver: str
    path: Path
    limits: LimitsCfg
    stream: StreamCfg
    connection: ConnectionCfg | None = None
    gripper: GripperCfg | None = None
    model_urdf: Path | None = None
    home_joints_deg: tuple[float, ...] | None = None
    home_confirmed: bool = False  # Home stays disabled until someone checked the pose is safe in the cell

    @property
    def dof(self) -> int:
        return len(self.limits.joints_deg)


TOP_KEYS = {"name", "driver", "connection", "model", "limits", "stream", "gripper", "home_joints_deg",
            "home_confirmed"}
LIMIT_KEYS = ("speed_cap_pct", "box_mm", "joints_deg", "max_tcp_speed_mm_s", "max_step_deg", "teleport_mm",
              "stale_ms")
DRIVERS_WITHOUT_CONNECTION = {"fake"}


def load_robot_config(path: Path | str) -> RobotConfig:
    path = Path(path).resolve()
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as e:
        raise ConfigError("file", f"cannot read {path}: {e}") from e
    except yaml.YAMLError as e:
        raise ConfigError("file", f"bad YAML in {path}: {e}") from e
    if not isinstance(raw, dict):
        raise ConfigError("file", f"{path} must be a YAML mapping")

    _no_unknown_keys(raw, TOP_KEYS, "")
    driver = _text(raw, "driver", "")
    limits = _limits(_section(raw, "limits", ""))

    connection = _connection(_section(raw, "connection", "")) if "connection" in raw else None
    if connection is None and driver not in DRIVERS_WITHOUT_CONNECTION:
        raise ConfigError("connection", f"driver {driver!r} needs a connection section (ip, ports)")

    urdf = None
    if "model" in raw:
        model = _section(raw, "model", "")
        _no_unknown_keys(model, {"urdf"}, "model.")
        urdf = (path.parent / _text(model, "urdf", "model.")).resolve()

    home = None
    if "home_joints_deg" in raw:
        home = _home(raw["home_joints_deg"], limits.joints_deg)
    home_confirmed = raw.get("home_confirmed", False)
    if not isinstance(home_confirmed, bool):
        raise ConfigError("home_confirmed", "must be true or false")

    return RobotConfig(
        name=_text(raw, "name", ""),
        driver=driver,
        path=path,
        limits=limits,
        stream=_stream(_section(raw, "stream", "")) if "stream" in raw else StreamCfg(),
        connection=connection,
        gripper=_gripper(_section(raw, "gripper", "")) if "gripper" in raw else None,
        model_urdf=urdf,
        home_joints_deg=home,
        home_confirmed=home_confirmed,
    )


# ---- sections ----


def _limits(d: dict) -> LimitsCfg:
    w = "limits."
    for key in LIMIT_KEYS:
        if key not in d:
            unknown = sorted(set(d) - set(LIMIT_KEYS))
            extra = f" (unknown keys here: {', '.join(unknown)}; typo?)" if unknown else ""
            raise ConfigError(w + key, "missing, every limit must be set" + extra)
    _no_unknown_keys(d, {*LIMIT_KEYS, "box_enabled"}, w)
    box_enabled = d.get("box_enabled", False)
    if not isinstance(box_enabled, bool):
        raise ConfigError(w + "box_enabled", "must be true or false")

    cap = _number(d, "speed_cap_pct", w, integer=True)
    if not 1 <= cap <= 100:
        raise ConfigError(w + "speed_cap_pct", f"must be 1..100, got {cap}")

    box = _section(d, "box_mm", w)
    _no_unknown_keys(box, {"x", "y", "z"}, w + "box_mm.")
    for axis in ("x", "y", "z"):
        if axis not in box:
            raise ConfigError(f"{w}box_mm.{axis}", "missing")
    box_t = tuple(_range(box[a], f"{w}box_mm.{a}") for a in ("x", "y", "z"))

    joints = d["joints_deg"]
    if not isinstance(joints, list) or not joints:
        raise ConfigError(w + "joints_deg", "must be a list of [min, max], one per joint")
    joints_t = tuple(_range(j, f"{w}joints_deg[{i}]") for i, j in enumerate(joints))

    return LimitsCfg(
        speed_cap_pct=int(cap),
        box_mm=box_t,  # type: ignore[arg-type]
        joints_deg=joints_t,
        max_tcp_speed_mm_s=_number(d, "max_tcp_speed_mm_s", w),
        max_step_deg=_number(d, "max_step_deg", w),
        teleport_mm=_number(d, "teleport_mm", w),
        stale_ms=_number(d, "stale_ms", w),
        box_enabled=box_enabled,
    )


def _connection(d: dict) -> ConnectionCfg:
    w = "connection."
    keys = {"ip", "dashboard_port", "motion_port", "feedback_port", "http_port"}
    _no_unknown_keys(d, keys, w)
    ports = {k: int(_number(d, k, w, integer=True)) for k in keys - {"ip"} if k in d}
    for k, v in ports.items():
        if not 1 <= v <= 65535:
            raise ConfigError(w + k, f"not a valid port: {v}")
    ip = _text(d, "ip", w)
    if not _valid_host(ip):
        raise ConfigError(w + "ip", f"not an IP address or host name: {ip!r}")
    return ConnectionCfg(ip=ip, **ports)


def _stream(d: dict) -> StreamCfg:
    w = "stream."
    _no_unknown_keys(d, {"period_s", "aheadtime", "gain"}, w)
    return StreamCfg(**{k: _number(d, k, w) for k in ("period_s", "aheadtime", "gain") if k in d})


def _gripper(d: dict) -> GripperCfg:
    w = "gripper."
    _no_unknown_keys(d, {"type", "slave", "baud"}, w)
    ints = {k: int(_number(d, k, w, integer=True)) for k in ("slave", "baud") if k in d}
    return GripperCfg(type=_text(d, "type", w), **ints)


def _home(value: Any, joint_limits: tuple[tuple[float, float], ...]) -> tuple[float, ...]:
    f = "home_joints_deg"
    if not isinstance(value, list) or len(value) != len(joint_limits):
        raise ConfigError(f, f"must be a list of {len(joint_limits)} joint angles")
    home = []
    for i, (v, (lo, hi)) in enumerate(zip(value, joint_limits, strict=True)):
        if not _is_number(v):
            raise ConfigError(f"{f}[{i}]", f"not a number: {v!r}")
        if not lo <= v <= hi:
            raise ConfigError(f"{f}[{i}]", f"{v} is outside the joint limit [{lo}, {hi}]")
        home.append(float(v))
    return tuple(home)


# ---- small validators ----


def _is_number(v: Any) -> bool:
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _section(d: dict, key: str, where: str) -> dict:
    if key not in d:
        raise ConfigError(where + key, "missing")
    if not isinstance(d[key], dict):
        raise ConfigError(where + key, "must be a mapping")
    return d[key]


def _text(d: dict, key: str, where: str) -> str:
    v = d.get(key)
    if not isinstance(v, str) or not v.strip():
        raise ConfigError(where + key, "missing or empty")
    return v


def _number(d: dict, key: str, where: str, integer: bool = False) -> float:
    if key not in d:
        raise ConfigError(where + key, "missing")
    v = d[key]
    if not _is_number(v) or (integer and not isinstance(v, int)):
        raise ConfigError(where + key, f"must be {'an integer' if integer else 'a number'}, got {v!r}")
    if v <= 0:
        raise ConfigError(where + key, f"must be > 0, got {v}")
    return float(v)


def _range(v: Any, field: str) -> tuple[float, float]:
    if not (isinstance(v, list) and len(v) == 2 and all(_is_number(x) for x in v)):
        raise ConfigError(field, f"must be [min, max], got {v!r}")
    lo, hi = float(v[0]), float(v[1])
    if lo >= hi:
        raise ConfigError(field, f"min {lo} must be below max {hi}")
    return (lo, hi)


def _no_unknown_keys(d: dict, known: set[str], where: str) -> None:
    unknown = sorted(set(d) - known)
    if unknown:
        raise ConfigError(where + unknown[0], f"unknown key (known: {', '.join(sorted(known))})")


def _valid_host(text: str) -> bool:
    import ipaddress
    import re

    try:
        ipaddress.ip_address(text)
        return True
    except ValueError:
        return re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9.-]{0,252}[A-Za-z0-9])?", text) is not None
