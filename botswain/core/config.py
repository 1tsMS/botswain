"""Robot and end-effector configs: load, validate, overlay, discover.

A config is a YAML "ID card": configs/robots/<name>.yaml says which protocol talks to the arm, where it is on the
network, its model file and its limits; configs/tools/<name>.yaml says which driver talks to the end effector
and how it is reached. Adding a robot of a known protocol, or a gripper of a known driver, is a new YAML only.

Every limit must be present and sane. A missing or misspelt key raises ConfigError naming the field: a typo must
never silently widen the workspace. Relative paths (the URDF) are relative to the YAML file's folder.
Lab-specific edits made in the app (IP, safety box, speeds) go to <name>.local.yaml next to the config and are
merged over it on load, so the shared, commented config file is never rewritten by the app.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "configs"
ROBOT_DIR = CONFIGS / "robots"
TOOL_DIR = CONFIGS / "tools"
SELECTION = CONFIGS / "selection.yaml"  # the robot and tool picked last (Settings page)


class ConfigError(Exception):
    def __init__(self, field: str, message: str) -> None:
        super().__init__(f"{field}: {message}")
        self.field = field
        self.message = message


@dataclass(frozen=True)
class ConnectionCfg:
    """Where the controller is: an address and its named ports. Which names exist is the protocol's business
    (a Dobot has dashboard / motion / feedback / http); the driver reads them with its own defaults."""

    ip: str
    ports: tuple[tuple[str, int], ...] = ()

    def port(self, name: str, default: int | None = None) -> int | None:
        return dict(self.ports).get(name, default)

    def with_ports(self, **ports: int) -> ConnectionCfg:
        return replace(self, ports=tuple({**dict(self.ports), **ports}.items()))


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
    # The speed control's 100 % sends this controller speed factor; lower % scale down linearly. Dobot's own
    # SpeedFactor is a share of the arm's full speed, so its 10 % jogs much faster than the gamepad's 10 %.
    controller_speed_max_pct: int = 100


@dataclass(frozen=True)
class StreamCfg:
    period_s: float = 0.03
    aheadtime: float = 50.0
    gain: float = 500.0


@dataclass(frozen=True)
class RobotConfig:
    name: str
    protocol: str  # which driver talks to this arm (drivers/registry.py), e.g. dobot_v3
    path: Path
    limits: LimitsCfg
    stream: StreamCfg
    vendor: str = ""
    model: str = ""
    connection: ConnectionCfg | None = None
    model_urdf: Path | None = None
    tool_port: str | None = None  # the kind of end-effector port this arm offers (rs485), None if none
    home_joints_deg: tuple[float, ...] | None = None
    home_confirmed: bool = False  # Home stays disabled until someone checked the pose is safe in the cell

    @property
    def dof(self) -> int:
        return len(self.limits.joints_deg)


@dataclass(frozen=True)
class ToolConfig:
    name: str
    driver: str  # which end-effector driver (botswain/tools/__init__.py), e.g. dh_modbus
    port: str  # the kind of port it needs on the arm, e.g. rs485
    path: Path
    vendor: str = ""
    model: str = ""
    slave: int = 1
    baud: int = 115200
    force_range_pct: tuple[int, int] | None = None
    mass_kg: float | None = None  # None = not filled in yet (the controller's payload must match it)
    tcp_offset_mm: tuple[float, float, float] | None = None  # tool tip from the flange, None = not measured


ROBOT_KEYS = {"name", "vendor", "model", "protocol", "connection", "urdf", "tool_port", "limits", "stream", "home"}
LIMIT_KEYS = ("speed_cap_pct", "box_mm", "joints_deg", "max_tcp_speed_mm_s", "max_step_deg", "teleport_mm",
              "stale_ms")
TOOL_KEYS = {"name", "vendor", "model", "driver", "port", "params", "force_range_pct", "mass_kg", "tcp_offset_mm"}
PROTOCOLS_WITHOUT_CONNECTION = {"fake"}
PORT_KINDS = ("rs485",)


# ---- loading ----


def overlay_path(path: Path) -> Path:
    return path.with_name(path.stem + ".local.yaml")


def _read(path: Path) -> dict:
    """The YAML file merged with its .local.yaml overlay (if any)."""
    raw = _read_yaml(path)
    local = overlay_path(path)
    if local.exists():
        raw = _merge(raw, _read_yaml(local))
    return raw


def _read_yaml(path: Path) -> dict:
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as e:
        raise ConfigError("file", f"cannot read {path}: {e}") from e
    except yaml.YAMLError as e:
        raise ConfigError("file", f"bad YAML in {path}: {e}") from e
    if not isinstance(raw, dict):
        raise ConfigError("file", f"{path} must be a YAML mapping")
    return raw


def _merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for key, value in over.items():
        out[key] = _merge(out[key], value) if isinstance(value, dict) and isinstance(out.get(key), dict) else value
    return out


def load_robot_config(path: Path | str) -> RobotConfig:
    path = Path(path).resolve()
    raw = _read(path)
    _no_unknown_keys(raw, ROBOT_KEYS, "")
    protocol = _text(raw, "protocol", "")
    limits = _limits(_section(raw, "limits", ""))

    connection = _connection(_section(raw, "connection", "")) if "connection" in raw else None
    if connection is None and protocol not in PROTOCOLS_WITHOUT_CONNECTION:
        raise ConfigError("connection", f"protocol {protocol!r} needs a connection section (ip, ports)")

    urdf = (path.parent / _text(raw, "urdf", "")).resolve() if "urdf" in raw else None
    if urdf is not None and not urdf.is_file():
        raise ConfigError("urdf", f"file not found: {urdf}")
    tool_port = raw.get("tool_port")
    if tool_port is not None and tool_port not in PORT_KINDS:
        raise ConfigError("tool_port", f"unknown port kind {tool_port!r} (known: {', '.join(PORT_KINDS)})")

    home, confirmed = None, False
    if "home" in raw:
        h = _section(raw, "home", "")
        _no_unknown_keys(h, {"joints_deg", "confirmed"}, "home.")
        home = _home(h.get("joints_deg"), limits.joints_deg)
        confirmed = h.get("confirmed", False)
        if not isinstance(confirmed, bool):
            raise ConfigError("home.confirmed", "must be true or false")

    return RobotConfig(
        name=_text(raw, "name", ""), protocol=protocol, path=path, limits=limits,
        stream=_stream(_section(raw, "stream", "")) if "stream" in raw else StreamCfg(),
        vendor=str(raw.get("vendor", "")), model=str(raw.get("model", "")),
        connection=connection, model_urdf=urdf, tool_port=tool_port,
        home_joints_deg=home, home_confirmed=confirmed,
    )


def load_tool_config(path: Path | str) -> ToolConfig:
    path = Path(path).resolve()
    raw = _read(path)
    _no_unknown_keys(raw, TOOL_KEYS, "")
    params = raw.get("params") or {}
    if not isinstance(params, dict):
        raise ConfigError("params", "must be a mapping")
    _no_unknown_keys(params, {"slave", "baud"}, "params.")
    ints = {k: int(_number(params, k, "params.", integer=True)) for k in ("slave", "baud") if k in params}
    force = None
    if raw.get("force_range_pct") is not None:
        lo, hi = _range(raw["force_range_pct"], "force_range_pct")
        force = (int(lo), int(hi))
    mass = None if raw.get("mass_kg") is None else _number(raw, "mass_kg", "")
    offset = raw.get("tcp_offset_mm")
    if offset is not None and not (isinstance(offset, list) and len(offset) == 3 and all(map(_is_number, offset))):
        raise ConfigError("tcp_offset_mm", f"must be [x, y, z] in mm, got {offset!r}")
    port = _text(raw, "port", "")
    if port not in PORT_KINDS:
        raise ConfigError("port", f"unknown port kind {port!r} (known: {', '.join(PORT_KINDS)})")
    return ToolConfig(
        name=_text(raw, "name", ""), driver=_text(raw, "driver", ""), port=port, path=path,
        vendor=str(raw.get("vendor", "")), model=str(raw.get("model", "")), force_range_pct=force, mass_kg=mass,
        tcp_offset_mm=tuple(float(v) for v in offset) if offset is not None else None, **ints,
    )


# ---- the app's own edits and choices ----


def save_overlay(path: Path, updates: dict) -> Path:
    """Merge updates into <name>.local.yaml (the shared config stays untouched). Returns the overlay path."""
    local = overlay_path(path)
    current = _read_yaml(local) if local.exists() else {}
    merged = _merge(current, updates)
    header = (f"# Edits made in Botswain (Settings) for this lab. Merged over {path.name} on load;\n"
              "# delete this file to go back to the shared config.\n")
    local.write_text(header + yaml.safe_dump(merged, sort_keys=False), encoding="utf-8")
    return local


def discover(folder: Path, loader) -> list[tuple[Path, Any | None, str]]:
    """Every config in a folder: (path, config or None, error text). .local.yaml overlays are not configs."""
    found = []
    for path in sorted(folder.glob("*.yaml")):
        if path.name.endswith(".local.yaml"):
            continue
        try:
            found.append((path, loader(path), ""))
        except ConfigError as e:
            found.append((path, None, str(e)))
    return found


def read_selection() -> tuple[Path | None, Path | None]:
    """(robot config, tool config) picked last; missing files count as not picked."""
    if not SELECTION.exists():
        return None, None
    try:
        raw = yaml.safe_load(SELECTION.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return None, None
    robot = ROBOT_DIR / raw["robot"] if raw.get("robot") else None
    tool = TOOL_DIR / raw["tool"] if raw.get("tool") else None
    return (robot if robot and robot.exists() else None), (tool if tool and tool.exists() else None)


def write_selection(robot: Path, tool: Path | None) -> None:
    SELECTION.write_text(yaml.safe_dump({"robot": robot.name, "tool": tool.name if tool else None}),
                         encoding="utf-8")


# ---- sections ----


def _limits(d: dict) -> LimitsCfg:
    w = "limits."
    for key in LIMIT_KEYS:
        if key not in d:
            unknown = sorted(set(d) - set(LIMIT_KEYS))
            extra = f" (unknown keys here: {', '.join(unknown)}; typo?)" if unknown else ""
            raise ConfigError(w + key, "missing, every limit must be set" + extra)
    _no_unknown_keys(d, {*LIMIT_KEYS, "box_enabled", "controller_speed_max_pct"}, w)
    box_enabled = d.get("box_enabled", False)
    if not isinstance(box_enabled, bool):
        raise ConfigError(w + "box_enabled", "must be true or false")

    cap = _number(d, "speed_cap_pct", w, integer=True)
    if not 1 <= cap <= 100:
        raise ConfigError(w + "speed_cap_pct", f"must be 1..100, got {cap}")
    ctrl_max = _number(d, "controller_speed_max_pct", w, integer=True) if "controller_speed_max_pct" in d else 100
    if not 1 <= ctrl_max <= 100:
        raise ConfigError(w + "controller_speed_max_pct", f"must be 1..100, got {ctrl_max}")

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
        controller_speed_max_pct=int(ctrl_max),
    )


def _connection(d: dict) -> ConnectionCfg:
    w = "connection."
    _no_unknown_keys(d, {"ip", "ports"}, w)
    ip = _text(d, "ip", w)
    if not _valid_host(ip):
        raise ConfigError(w + "ip", f"not an IP address or host name: {ip!r}")
    raw = d.get("ports") or {}
    if not isinstance(raw, dict):
        raise ConfigError(w + "ports", "must be a mapping like {dashboard: 29999}")
    ports = []
    for name in raw:
        v = _number(raw, name, w + "ports.", integer=True)
        if not 1 <= v <= 65535:
            raise ConfigError(f"{w}ports.{name}", f"not a valid port: {int(v)}")
        ports.append((str(name), int(v)))
    return ConnectionCfg(ip=ip, ports=tuple(ports))


def _stream(d: dict) -> StreamCfg:
    w = "stream."
    _no_unknown_keys(d, {"period_s", "aheadtime", "gain"}, w)
    return StreamCfg(**{k: _number(d, k, w) for k in ("period_s", "aheadtime", "gain") if k in d})


def _home(value: Any, joint_limits: tuple[tuple[float, float], ...]) -> tuple[float, ...]:
    f = "home.joints_deg"
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
