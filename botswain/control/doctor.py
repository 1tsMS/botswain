"""Doctor: health checks that never move the robot (SPEC 5.9).

Network checks (ping, ports) and offline probes open their own short connections and run in any thread.
Checks that need the live connection come from driver.diagnose(), which the worker runs (it owns the sockets).
"""

from __future__ import annotations

import re
import socket
import subprocess

import numpy as np

from botswain.core.config import RobotConfig
from botswain.core.types import RobotState
from botswain.drivers.base import Check

HINT_NET = "check the cable and controller power; the laptop needs a static IP on the robot's subnet (192.168.5.x)"


def ping(ip: str) -> Check:
    name = f"Ping {ip}"
    try:
        out = subprocess.run(["ping", "-n", "1", "-w", "500", ip], capture_output=True, text=True, timeout=3, check=False,
                             creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
    except (OSError, subprocess.TimeoutExpired) as e:
        return Check(name, "fail", str(e), HINT_NET)
    # Windows ping can exit 0 on "Destination host unreachable"; only a TTL line is a real answer.
    if out.returncode == 0 and "TTL=" in out.stdout.upper():
        ms = re.search(r"[=<](\d+)\s*ms", out.stdout)
        return Check(name, "ok", f"{ms.group(1)} ms" if ms else "reply")
    return Check(name, "fail", "no reply", HINT_NET)


def port_check(name: str, ip: str, port: int, required: bool = True, timeout_s: float = 0.5) -> Check:
    label = f"{name} port {port}"
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return Check(label, "ok", "open")
    except OSError as e:
        hint = HINT_NET if required else "optional port"
        return Check(label, "fail" if required else "warn", f"closed ({e.__class__.__name__})", hint)


def network_checks(cfg: RobotConfig, connected: bool) -> list[Check]:
    """Ping the controller and try each port the robot config names. While connected, the ports are in use by
    the driver (some controllers allow one client), so they are not opened a second time."""
    if cfg.connection is None:
        return [Check("Network", "ok", "no network for this robot")]
    c = cfg.connection
    checks = [ping(c.ip)]
    seen: set[int] = set()
    for name, port in c.ports:
        if port in seen:
            continue
        seen.add(port)
        if connected:
            checks.append(Check(f"{name.capitalize()} port {port}", "ok", "in use by Botswain"))
        else:
            checks.append(port_check(name.capitalize(), c.ip, port))
    return checks


def offline_checks(cfg: RobotConfig, sample_s: float = 1.0) -> list[Check]:
    """The robot's own checks that need no connection (each driver defines them)."""
    from botswain.drivers.registry import driver_class

    if cfg.connection is None:
        return []
    return driver_class(cfg).offline_checks(cfg, sample_s)



def model_check(model, state: RobotState, tol_mm: float = 10.0) -> Check:
    """URDF flange position for the measured joints vs the controller's TCP. Warn only: a tool offset is normal."""
    t = model.link_transform("Link6", state.joints.values)[:3, 3] * 1000.0
    d = float(np.linalg.norm(t - np.array(state.tcp.xyz)))
    ok = d <= tol_mm
    return Check("3D model vs robot", "ok" if ok else "warn", f"flange {d:.1f} mm from the reported TCP",
                 "" if ok else "a tool offset or a different joint zero; compare the 3D view with the real arm")
