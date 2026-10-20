"""Doctor: health checks that never move the robot (SPEC 5.9).

Network checks (ping, ports) and offline probes open their own short connections and run in any thread.
Checks that need the live connection come from driver.diagnose(), which the worker runs (it owns the sockets).
"""

from __future__ import annotations

import re
import socket
import subprocess
import time

import numpy as np

from botswain.core.config import RobotConfig
from botswain.core.types import RobotState
from botswain.drivers.base import Check, DriverError

HINT_NET = "check the cable and controller power; the laptop needs a static IP on the robot's subnet (192.168.5.x)"
FALLBACK_MOTION_PORT = 30003


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
        hint = HINT_NET if required else "only needed if motion_port is set to it"
        return Check(label, "fail" if required else "warn", f"closed ({e.__class__.__name__})", hint)


def network_checks(cfg: RobotConfig, connected: bool) -> list[Check]:
    if cfg.connection is None:
        return [Check("Network", "ok", "fake robot, nothing to check")]
    c = cfg.connection
    checks = [ping(c.ip)]
    seen: set[int] = set()
    for name, port in (("Dashboard", c.dashboard_port), ("Feedback", c.feedback_port), ("Motion", c.motion_port),
                       ("HTTP", c.http_port)):
        if port in seen:
            continue
        seen.add(port)
        if connected and name != "HTTP":
            checks.append(Check(f"{name} port {port}", "ok", "in use by Botswain"))  # don't open a 2nd client
        else:
            checks.append(port_check(name, c.ip, port))
    if FALLBACK_MOTION_PORT not in seen:
        checks.append(port_check("Motion fallback", c.ip, FALLBACK_MOTION_PORT, required=False))
    return checks


def offline_checks(cfg: RobotConfig, sample_s: float = 1.0) -> list[Check]:
    """Feedback rate and TCP/IP mode without connecting the driver (Doctor before Connect)."""
    from botswain.drivers.dobot.dashboard import HINT_TCP, NOT_TCP, DashboardClient
    from botswain.drivers.dobot.feedback import FrameReader

    if cfg.connection is None or not cfg.driver.startswith("dobot_"):
        return []
    c = cfg.connection
    checks = []
    try:
        with socket.create_connection((c.ip, c.feedback_port), timeout=0.5) as sock:
            sock.settimeout(0.2)
            reader, count, t_end = FrameReader(), 0, time.perf_counter() + sample_s
            while time.perf_counter() < t_end:
                try:
                    data = sock.recv(65536)
                except TimeoutError:
                    continue
                if not data:
                    break
                count += len(reader.feed(data))
        hz = count / sample_s
        status = "ok" if 100 <= hz <= 150 else "warn" if hz > 0 else "fail"
        checks.append(Check("Feedback rate", status, f"{hz:.0f} Hz, {reader.bad_frames} bad frames",
                            "" if status == "ok" else "expected ~125 Hz"))
    except OSError as e:
        checks.append(Check("Feedback rate", "fail", f"cannot read ({e.__class__.__name__})", HINT_NET))

    dash = DashboardClient(c.ip, c.dashboard_port, timeout_s=1.0)
    try:
        dash.connect()
        reply = dash.call("RobotMode()")
        if reply.error_id == NOT_TCP:
            checks.append(Check("TCP/IP mode", "fail", "controller answered 'Not Tcp'", HINT_TCP))
        else:
            checks.append(Check("TCP/IP mode", "ok", f"RobotMode = {reply.values}"))
    except DriverError as e:
        checks.append(Check("TCP/IP mode", "fail", e.message, e.hint))
    finally:
        dash.close()
    return checks


def model_check(model, state: RobotState, tol_mm: float = 10.0) -> Check:
    """URDF flange position for the measured joints vs the controller's TCP. Warn only: a tool offset is normal."""
    t = model.link_transform("Link6", state.joints.values)[:3, 3] * 1000.0
    d = float(np.linalg.norm(t - np.array(state.tcp.xyz)))
    ok = d <= tol_mm
    return Check("3D model vs robot", "ok" if ok else "warn", f"flange {d:.1f} mm from the reported TCP",
                 "" if ok else "a tool offset or a different joint zero; compare the 3D view with the real arm")
