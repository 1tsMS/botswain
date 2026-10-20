"""A pretend Dobot V4 controller on localhost, so the real DobotDriver can be tested end to end at home.

Speaks the real protocol: dashboard replies `ErrorID,{values},Cmd(...);` with real error codes, 1440-byte
feedback packets every 8 ms, and the HTTP gripper endpoint. Joints and pose are not linked (no IK).
Test switches: split_packets, garbage_every, tcp_mode, feedback_paused, inject_alarm(code).

    python -m botswain.drivers.dobot.fake_controller      # 29999, 30003, 30004, 22000 on 127.0.0.1
"""

from __future__ import annotations

import json
import logging
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from botswain.drivers.dobot.feedback import build_packet
from botswain.drivers.dobot.gripper import crc16_modbus
from botswain.drivers.fake import step_toward

log = logging.getLogger(__name__)

TICK_S = 0.008
LIN_SPEED = 250.0  # mm/s at 100 % speed factor
JOINT_SPEED = 60.0  # deg/s at 100 %
CART = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
MOTION = {"movj", "movl", "jointmovj", "relmovluser", "relmovltool", "reljointmovj", "servop", "movejog"}


def split_args(inner: str) -> list[str]:
    """Split on commas that are not inside {} [] ()."""
    out, depth, cur = [], 0, ""
    for ch in inner:
        if ch in "{[(":
            depth += 1
        elif ch in "}])":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def parse_command(text: str) -> tuple[str, list[str], dict[str, str]]:
    name, _, rest = text.partition("(")
    inner = rest.rsplit(")", 1)[0]
    pos, kw = [], {}
    for a in split_args(inner):
        key, eq, value = a.partition("=")
        if eq and key.strip().isidentifier():
            kw[key.strip().lower()] = value.strip()
        else:
            pos.append(a)
    return name.strip().lower(), pos, kw


def vec(text: str) -> np.ndarray:
    return np.array([float(x) for x in text.strip("{} ").split(",")])


class FakeDobotController:
    def __init__(self, host: str = "127.0.0.1", dashboard_port: int = 0, feedback_port: int = 0, http_port: int = 0,
                 motion_port: int | None = None) -> None:
        self.host = host
        self.split_packets = False
        self.garbage_every = 0
        self.tcp_mode = True
        self.feedback_paused = False
        self.pose_offset_mm = 0.0  # GetPose() differs from feedback by this much (Doctor test)
        self.legacy = False  # behave like the lab CR5's older firmware (positional syntax only)
        self.commands: list[str] = []
        self.gripper_frames: list[list[int]] = []
        self.gripper_regs: dict[int, int] = {0x0200: 0, 0x0201: 1, 0x0202: 1000}  # DH registers, not initialised
        self.gripper_object_permille: int | None = None  # set: a close stops here with "object caught"

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self.mode = 4
        self.alarms: list[int] = []
        self._enabled_before_alarm = False
        self.joints = np.array([0.0, 0.0, 90.0, 0.0, -90.0, 0.0])
        self.tcp = np.array([-482.0, -136.0, 484.0, 180.0, 0.0, 0.0])
        self.joints_target = self.joints.copy()
        self.tcp_target = self.tcp.copy()
        self.speed = 10
        self.jog: tuple[bool, int, int] | None = None  # (is_joint, index, sign)
        self.command_id = 0

        self._listeners: dict[str, socket.socket] = {}
        wanted = {"dashboard": dashboard_port, "feedback": feedback_port}
        if motion_port is not None:
            wanted["motion"] = motion_port
        for name, port in wanted.items():
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((host, port))
            s.listen(4)
            s.settimeout(0.2)
            self._listeners[name] = s
        self._http = ThreadingHTTPServer((host, http_port), self._http_handler())
        self._feedback_clients: list[socket.socket] = []
        self._clients: list[socket.socket] = []
        self._threads: list[threading.Thread] = []

    @property
    def ports(self) -> dict[str, int]:
        out = {name: s.getsockname()[1] for name, s in self._listeners.items()}
        out["http"] = self._http.server_address[1]
        return out

    def start(self) -> None:
        for name, s in self._listeners.items():
            target = self._accept_feedback if name == "feedback" else self._accept_dashboard
            self._spawn(target, s)
        self._spawn(self._feedback_loop)
        self._spawn(self._http.serve_forever, 0.2)

    def stop(self) -> None:
        self._stop.set()
        self._http.shutdown()
        self._http.server_close()
        for s in [*self._listeners.values(), *self._clients, *self._feedback_clients]:
            try:
                s.close()
            except OSError:
                pass
        for t in self._threads:
            t.join(1.0)

    def inject_alarm(self, code: int) -> None:
        with self._lock:
            self._enabled_before_alarm = self.mode in (5, 7, 8)
            self.alarms.append(code)
            self.mode = 9
            self._halt()

    # ---- threads ----

    def _spawn(self, target, *args) -> None:
        t = threading.Thread(target=target, args=args, daemon=True)
        t.start()
        self._threads.append(t)

    def _accept(self, listener: socket.socket) -> socket.socket | None:
        try:
            conn, _ = listener.accept()
        except (TimeoutError, OSError):
            return None
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return conn

    def _accept_dashboard(self, listener: socket.socket) -> None:
        while not self._stop.is_set():
            conn = self._accept(listener)
            if conn is not None:
                self._clients.append(conn)
                self._spawn(self._serve_dashboard, conn)

    def _accept_feedback(self, listener: socket.socket) -> None:
        while not self._stop.is_set():
            conn = self._accept(listener)
            if conn is not None:
                with self._lock:
                    self._feedback_clients.append(conn)

    def _serve_dashboard(self, conn: socket.socket) -> None:
        buf = b""
        conn.settimeout(0.2)
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except TimeoutError:
                continue
            except OSError:
                return
            if not data:
                return
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("ascii", "replace").strip()
                if text:
                    try:
                        conn.sendall(self.handle(text).encode("ascii"))
                    except OSError:
                        return

    def _feedback_loop(self) -> None:
        next_t = time.perf_counter()
        last = next_t
        sent = 0
        while not self._stop.is_set():
            now = time.perf_counter()
            with self._lock:
                self._integrate(now - last)
                packet = self._packet()
                clients = list(self._feedback_clients)
            last = now
            if not self.feedback_paused:
                sent += 1
                garbage = self.garbage_every and sent % self.garbage_every == 0
                for c in clients:
                    self._send_packet(c, packet, garbage)
            next_t += TICK_S
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()

    def _send_packet(self, conn: socket.socket, packet: bytes, garbage: bool) -> None:
        try:
            if garbage:
                conn.sendall(os.urandom(17))
            if self.split_packets:
                for a, b in ((0, 500), (500, 1000), (1000, len(packet))):
                    conn.sendall(packet[a:b])
                    time.sleep(0.0005)
            else:
                conn.sendall(packet)
        except OSError:
            with self._lock:
                if conn in self._feedback_clients:
                    self._feedback_clients.remove(conn)

    # ---- the robot ----

    def gripper_modbus(self, frame: list[int]) -> list[int]:
        """A DH gripper on the tool port: write (06) echoes and sets the register, read (03) answers it.
        A close stops at a 'caught object' (gripper_object_permille) and reports grip state 2."""
        reg, value = (frame[2] << 8) | frame[3], (frame[4] << 8) | frame[5]
        if frame[1] == 0x06:
            self.gripper_regs[reg] = value
            if reg == 0x0100:
                self.gripper_regs[0x0200] = 1
            if reg == 0x0103:
                obj = self.gripper_object_permille
                caught = obj is not None and value < obj
                self.gripper_regs[0x0202] = obj if caught else value
                self.gripper_regs[0x0201] = 2 if caught else 1
            return frame
        if frame[1] == 0x03:
            v = self.gripper_regs.get(reg, 0)
            body = [frame[0], 0x03, 2, v >> 8, v & 0xFF]
            crc = crc16_modbus(bytes(body))
            return [*body, crc & 0xFF, crc >> 8]
        return []

    def _packet(self) -> bytes:
        tcp_target = self.tcp if self.jog else self.tcp_target
        return build_packet(
            mode=self.mode, timestamp_ms=int(time.time() * 1000), q_actual=tuple(self.joints),
            q_target=tuple(self.joints_target), tcp_actual=tuple(self.tcp), tcp_target=tuple(tcp_target),
            speed_scaling=float(self.speed), enable=int(self.mode in (5, 6, 7, 8, 10)), error=int(self.mode == 9),
            collision=int(self.mode == 11), command_id=self.command_id)

    def _integrate(self, dt: float) -> None:
        scale = self.speed / 100.0
        if self.jog is not None:
            is_joint, i, sign = self.jog
            if is_joint:
                self.joints[i] += sign * JOINT_SPEED * scale * dt
            else:
                self.tcp[i] += sign * LIN_SPEED * scale * dt * (1.0 if i < 3 else 0.5)
            self.joints_target, self.tcp_target = self.joints.copy(), self.tcp.copy()
            return
        if self.mode != 7:
            return
        self.tcp[:3] = step_toward(self.tcp[:3], self.tcp_target[:3], LIN_SPEED * scale * dt)
        self.tcp[3:] = step_toward(self.tcp[3:], self.tcp_target[3:], LIN_SPEED * 0.5 * scale * dt)
        self.joints = step_toward(self.joints, self.joints_target, JOINT_SPEED * scale * dt)
        if np.array_equal(self.tcp, self.tcp_target) and np.array_equal(self.joints, self.joints_target):
            self.mode = 5

    def _halt(self) -> None:
        self.jog = None
        self.tcp_target, self.joints_target = self.tcp.copy(), self.joints.copy()

    def handle(self, text: str) -> str:
        """One dashboard command -> one reply string."""
        with self._lock:
            self.commands.append(text)
            if not self.tcp_mode:
                if text.lower().startswith("requestcontrol"):
                    self.tcp_mode = True  # like firmware >= 4.6: taking control switches to TCP mode
                    return f"0,{{}},{text};"
                return "Not Tcp"
            name, pos, kw = parse_command(text)
            if self.legacy:
                if pos and pos[0].strip() == "-0.000000":  # the lab CR5 reads "-0" as a bad first parameter
                    return f"-30001,{{}},{text};"
                if name in ("movj", "movl") and ("joint" in kw or "pose" in kw):
                    return f"-30001,{{}},{text};"
                if name == "movejog" and "coordtype=" in text:
                    return f"-30002,{{}},{text};"
                if name == "movejog" and "CoordType=" in text:
                    return f"-40002,{{}},{text};"
                if name == "movejog" and len(pos) > 1:  # positional frame: the lab CR5's split refusal
                    return f"-40001,0,{{}},;{text};"
                if name == "stop":
                    return f"-10000,{{}},{text};"
                if name == "servop" and kw:
                    return f"-20000,{{}},{text};"
            elif name == "jointmovj":
                return f"-10000,{{}},{text};"
            try:
                err, values = self._do(name, pos, kw)
            except (ValueError, IndexError):
                err, values = -30001, ""
            return f"{err},{{{values}}},{text};"

    def _do(self, name: str, pos: list[str], kw: dict[str, str]) -> tuple[int, str]:
        if name in MOTION:
            if name == "movejog" and not pos:
                if self.jog is not None:
                    self._halt()
                    if self.mode == 8:
                        self.mode = 5
                return 0, ""
            if self.mode == 9:
                return -2, ""
            if self.mode not in (5, 7, 8):
                return -1, ""
            return self._motion(name, pos, kw), ""
        match name:
            case "enablerobot":
                if self.mode == 9:
                    return -2, ""
                if self.mode in (3, 4):
                    self.mode = 5
                return 0, ""
            case "disablerobot":
                self._halt()
                if self.mode != 9:
                    self.mode = 4
                return 0, ""
            case "clearerror":
                self.alarms.clear()
                if self.mode == 9:
                    self.mode = 5 if self._enabled_before_alarm else 4
                return 0, ""
            case "startdrag":
                if self.mode != 5:
                    return -1, ""
                self._halt()
                self.mode = 6
                return 0, ""
            case "stopdrag":
                if self.mode == 6:
                    self.mode = 5
                return 0, ""
            case "stop" | "resetrobot":
                self._halt()
                if self.mode in (7, 8):
                    self.mode = 5
                return 0, ""
            case "robotmode":
                return 0, str(self.mode)
            case "geterrorid":
                return 0, "[[" + ",".join(str(a) for a in self.alarms) + "]]"
            case "getpose":
                p = self.tcp.copy()
                p[0] += self.pose_offset_mm
                return 0, ",".join(f"{v:.4f}" for v in p)
            case "getangle":
                return 0, ",".join(f"{v:.4f}" for v in self.joints)
            case "speedfactor":
                pct = int(pos[0])
                if not 1 <= pct <= 100:
                    return -40001, ""
                self.speed = pct
                return 0, ""
            case "requestcontrol" | "settoolpower" | "settool485" | "user" | "tool":
                return 0, ""
        return -10000, ""

    def _motion(self, name: str, pos: list[str], kw: dict[str, str]) -> int:
        self.command_id += 1
        if name == "movejog":
            axis = pos[0]
            coordtype = int(kw.get("coordtype", pos[1] if len(pos) > 1 else ("0" if axis.startswith("J") else "1")))
            is_joint = axis.startswith("J")
            if is_joint != (coordtype == 0):
                return -6
            index = int(axis[1]) - 1 if is_joint else CART[axis[:-1].lower()]
            self.jog = (is_joint, index, 1 if axis.endswith("+") else -1)
            self.mode = 8
            return 0
        if name == "jointmovj":
            self.joints_target = np.array([float(v) for v in pos[:6]])
        elif name in ("movj", "movl"):
            if "joint" in kw:
                self.joints_target = vec(kw["joint"])
            elif "pose" in kw:
                self.tcp_target = vec(kw["pose"])
            else:
                self.tcp_target = np.array([float(v) for v in pos[:6]])
        elif name in ("relmovluser", "relmovltool"):
            self.tcp_target = self.tcp_target + np.array([float(v) for v in pos[:6]])
        elif name == "reljointmovj":
            self.joints_target = self.joints_target + np.array([float(v) for v in pos[:6]])
        elif name == "servop":
            self.tcp_target = np.array([float(v) for v in pos[:6]])
        self.mode = 7
        return 0

    # ---- HTTP 22000 ----

    def _http_handler(self):
        ctrl = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:  # keep test output clean
                pass

            def _reply(self, code: int, body: dict) -> None:
                data = json.dumps(body).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self) -> None:
                if self.path == "/protocol/getAlarm":
                    self._reply(200, {"alarms": [{"id": a, "text": f"fake alarm {a}"} for a in ctrl.alarms]})
                else:
                    self._reply(404, {"error": "not found"})

            def do_POST(self) -> None:
                if self.path != "/interface/toolDataExchange":
                    self._reply(404, {"error": "not found"})
                    return
                body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
                frame = list(body.get("value", []))
                crc_ok = len(frame) >= 3 and crc16_modbus(bytes(frame[:-2])) == frame[-2] | (frame[-1] << 8)
                with ctrl._lock:
                    ctrl.gripper_frames.append(frame)
                    reply = ctrl.gripper_modbus(frame) if crc_ok else []
                self._reply(200, {"status": crc_ok, "data": {"value": reply, "index": len(ctrl.gripper_frames)}})

        return Handler


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    ctrl = FakeDobotController(dashboard_port=29999, feedback_port=30004, http_port=22000, motion_port=30003)
    ctrl.start()
    print(f"Fake Dobot controller on 127.0.0.1 {ctrl.ports}")
    print("keys + Enter: p pause feedback, a alarm 22, s split packets, g garbage, t TCP mode, q quit")
    try:
        while True:
            key = input().strip().lower()
            if key == "p":
                ctrl.feedback_paused = not ctrl.feedback_paused
                print("feedback paused" if ctrl.feedback_paused else "feedback running")
            elif key == "a":
                ctrl.inject_alarm(22)
                print("alarm 22")
            elif key == "s":
                ctrl.split_packets = not ctrl.split_packets
                print(f"split packets {ctrl.split_packets}")
            elif key == "g":
                ctrl.garbage_every = 0 if ctrl.garbage_every else 10
                print(f"garbage every {ctrl.garbage_every}")
            elif key == "t":
                ctrl.tcp_mode = not ctrl.tcp_mode
                print(f"TCP mode {ctrl.tcp_mode}")
            elif key == "q":
                break
    except (KeyboardInterrupt, EOFError):
        pass
    ctrl.stop()


if __name__ == "__main__":
    main()
