"""DobotDriver: the driver contract on a Dobot controller, V3 or V4 command set (dialects.py).

Commands go to the dashboard port (29999); moves can go to a separate motion port (30003) if the robot config
says so. State comes only from the 30004 feedback thread, so read_state() never touches a socket.
Connect never enables. stop() ends drag, ends a hold-jog and clears the queue; disconnect() stops first.
recover() is the way out of "it hit the table and now nothing answers".
"""

from __future__ import annotations

import json
import logging
import re
import socket
import time
import urllib.request
from dataclasses import replace

from botswain.core.clock import now_ns
from botswain.core.config import RobotConfig
from botswain.core.types import (
    Capabilities,
    Joints,
    Mode,
    Pose,
    RobotState,
    cart_index,
    is_joint_axis,
    joint_index,
)
from botswain.drivers.base import Check, Driver, DriverError
from botswain.drivers.dobot import commands as cmd
from botswain.drivers.dobot.dashboard import HINT_TCP, NO_LINK, NOT_TCP, TIMEOUT, DashboardClient
from botswain.drivers.dobot.dialects import V3, V4, Dialect
from botswain.drivers.dobot.feedback import STALE_NS, FeedbackPacket, FeedbackThread
from botswain.drivers.dobot.tool_port import DobotRs485Port

log = logging.getLogger(__name__)

START_SPEED_PCT = 10  # set on connect, so the first move after Enable is slow whatever the pendant had
ENABLE_TIMEOUT_S = 20.0  # EnableRobot answers only when enabling is done: over 5 s on the lab CR5
ALARM_WORDS = {-2: "collision", -3: "safe-skin collision"}  # GetErrorID codes with a name (V3 doc)
PORTS = {"dashboard": 29999, "feedback": 30004, "http": 22000}  # defaults; "motion" defaults to the dashboard
HINT_NET = "check the cable and controller power; the laptop needs a static IP on the robot's subnet (192.168.5.x)"


def dobot_ports(cfg: RobotConfig) -> dict[str, int]:
    """The controller's ports by name, from the robot config with Dobot's defaults filled in."""
    c = cfg.connection
    ports = {name: c.port(name, default) for name, default in PORTS.items()}
    ports["motion"] = c.port("motion", ports["dashboard"])
    return ports


class DobotDriver(Driver):
    dialect: Dialect = V4

    def __init__(self, cfg: RobotConfig) -> None:
        if cfg.connection is None:
            raise DriverError(-1, f"{self.dialect.name} needs a connection section in the robot config")
        self.cfg = cfg
        c = cfg.connection
        self.ports = dobot_ports(cfg)
        self.capabilities = Capabilities(
            name=cfg.name, dof=6, step_jog=True, hold_jog=True, streaming=True,
            stream_rate_hz=1.0 / cfg.stream.period_s,
            gripper=False,  # end effectors are separate drivers (botswain/tools), reached through tool_port()
            frames=("base", "tool"), max_tcp_speed_mm_s=cfg.limits.max_tcp_speed_mm_s,
            drag_mode=True, request_control=self.dialect.request_control,
        )
        p = self.ports
        self._dash = DashboardClient(c.ip, p["dashboard"])
        self._motion = self._dash if p["motion"] == p["dashboard"] else DashboardClient(c.ip, p["motion"])
        self._feedback: FeedbackThread | None = None
        self._latest: tuple[FeedbackPacket, int] | None = None  # (packet, arrival ns), replaced atomically
        self._connected = False
        self._speed_pct = 0
        self._streaming = False
        self._alarm_ids: tuple[int, ...] = ()

    @property
    def latency_ms(self) -> float | None:
        return self._dash.last_rtt_ms

    # ---- links ----

    def connect(self) -> None:
        if self._connected:
            return
        c = self.cfg.connection
        try:
            self._dash.connect()
            if self._motion is not self._dash:
                self._motion.connect()
            self._feedback = FeedbackThread(c.ip, self.ports["feedback"], self._on_packet)
            self._feedback.start()
            if self._dash.call(cmd.robot_mode()).error_id == NOT_TCP:
                raise DriverError(NOT_TCP, "controller is not in TCP/IP mode", HINT_TCP)
            speed = min(START_SPEED_PCT, self.cfg.limits.speed_cap_pct)
            self._dash.call_ok(cmd.speed_factor(self._controller_pct(speed)))
            self._speed_pct = speed
        except Exception:
            self._close_links()
            raise
        self._connected = True
        self._alarm_ids = ()

    def disconnect(self) -> None:
        # The worker calls stop() itself right before this and reports a failure there; here a dead link
        # must not stop us from closing the sockets.
        try:
            self.stop()
        except DriverError as e:
            log.warning("stop before disconnect failed: %s", e)
        finally:
            self._close_links()

    def _close_links(self) -> None:
        self._connected = False
        self._streaming = False
        if self._feedback is not None:
            self._feedback.stop()
            self._feedback = None
        self._latest = None
        self._dash.close()
        if self._motion is not self._dash:
            self._motion.close()

    def _on_packet(self, packet: FeedbackPacket, t_ns: int) -> None:
        self._latest = (packet, t_ns)

    # ---- state ----

    def read_state(self) -> RobotState:
        latest = self._latest
        now = now_ns()
        if not self._connected or latest is None:
            mode = Mode.UNKNOWN if self._connected else Mode.DISCONNECTED
            fault = "no feedback from port 30004 yet" if self._connected else ""
            return replace(RobotState.disconnected(6, now), mode=mode, speed_pct=self._speed_pct, fault=fault)
        p, t = latest
        mode = self.dialect.modes.get(p.mode, Mode.UNKNOWN)
        if mode is Mode.MOVING and self._streaming:
            mode = Mode.STREAMING
        link_ok = now - t < STALE_NS and self._dash.connected
        return RobotState(
            t_ns=t, mode=mode, joints=Joints(p.q_actual), tcp=Pose(*p.tcp_actual), joints_target=Joints(p.q_target),
            tcp_target=Pose(*p.tcp_target), speed_pct=self._speed_pct,
            errors=self._alarm_ids if mode in (Mode.ERROR, Mode.COLLISION) else (),
            raw_mode=p.mode, link_ok=link_ok,
            feedback_hz=self._feedback.hz if self._feedback else 0.0,
            fault=self._fault(mode, now - t),
        )

    def _fault(self, mode: Mode, age_ns: int) -> str:
        if self._dash.silent or not self._dash.connected:
            return "controller not answering on port 29999: press Recover"
        if age_ns >= STALE_NS:
            return f"feedback lost (last packet {age_ns / 1e9:.1f} s ago)"
        if mode in (Mode.ERROR, Mode.COLLISION):
            return "alarm: " + _alarm_words(self._alarm_ids)
        return ""

    # ---- commands ----

    def enable(self) -> None:
        self._ok(self._dash, cmd.enable_robot(), timeout_s=ENABLE_TIMEOUT_S)

    def disable(self) -> None:
        self._streaming = False
        self._ok(self._dash, cmd.disable_robot())

    def clear_error(self) -> None:
        self._ok(self._dash, cmd.clear_error())
        self._refresh_alarms()

    def stop(self) -> None:
        """End drag, end a hold-jog (MoveJog()), clear the queue and any stream (Stop() / ResetRobot()).

        A refusal (e.g. ErrorID -2 in alarm) is fine: nothing is moving. Only a dead link raises, because
        then the stop did NOT reach the robot and the operator must use the physical e-stop.
        """
        self._streaming = False
        if not self._connected:
            return
        failures = []
        steps = [cmd.move_jog(None), self.dialect.stop]
        latest = self._latest
        if latest is not None and self.dialect.modes.get(latest[0].mode) is Mode.DRAG:
            steps.insert(0, cmd.stop_drag())  # drag first: the arm holds where it is
        for text in steps:
            try:
                reply = self._dash.call(text)
            except DriverError as e:
                failures.append(f"{text}: {e.message}")
                continue
            if reply.error_id != 0:
                log.info("%s answered %d (nothing to stop)", text, reply.error_id)
        if failures:
            raise DriverError(NO_LINK, "STOP could not be sent: " + "; ".join(failures),
                              "use the physical e-stop, then press Recover")

    def recover(self) -> list[str]:
        """The way out of a stuck controller (e.g. it pressed into the table and stopped answering).

        1. reconnect the dashboard if it stopped answering, 2. STOP (drag, jog, queue), 3. ClearError,
        4. read the alarms back. It never enables: after a fault the operator decides that (Enable button).
        Returns what happened, one line per step, for the log.
        """
        done: list[str] = []
        try:
            self._dash.call(cmd.robot_mode(), timeout_s=1.0)
        except DriverError:
            pass
        if self._dash.silent or not self._dash.connected:
            self._dash.reconnect()  # raises if the controller is gone: then only a power cycle helps
            done.append("dashboard 29999 reconnected")
            if self._motion is not self._dash:
                self._motion.reconnect()
        try:
            self.stop()
            done.append("stopped (drag, jog, motion queue)")
        except DriverError as e:
            done.append(f"stop: {e.message}")
        reply = self._dash.call(cmd.clear_error())
        done.append("ClearError() accepted" if reply.error_id == 0 else f"ClearError() answered {reply.error_id}")
        self._refresh_alarms()
        done.append("alarms now: " + (_alarm_words(self._alarm_ids) if self._alarm_ids else "none"))
        return done

    def set_speed(self, pct: int) -> None:
        self._ok(self._dash, cmd.speed_factor(self._controller_pct(pct)))
        self._speed_pct = int(pct)

    def _controller_pct(self, pct: int) -> int:
        """Speed control % -> Dobot SpeedFactor: 100 % is limits.controller_speed_max_pct, never below 1."""
        return max(1, round(pct * self.cfg.limits.controller_speed_max_pct / 100))

    def jog_step(self, axis: str, delta: float, frame: str) -> None:
        d = [0.0] * 6
        if is_joint_axis(axis):
            d[joint_index(axis)] = delta
            self._ok(self._motion, cmd.rel_joint_movj(d))
        else:
            d[cart_index(axis)] = delta
            text = cmd.rel_movl_tool(d) if frame == "tool" else cmd.rel_movl_user(d, user=0)
            self._ok(self._motion, text)

    def move_to(self, target: Pose | Joints, kind: str) -> None:
        if isinstance(target, Joints):
            self._ok(self._motion, self.dialect.joint_move(target))
        else:
            self._ok(self._motion, self.dialect.pose_move(target, kind == "linear"))

    def jog_hold(self, axis: str, direction: int, frame: str) -> None:
        text = self.dialect.jog(f"{axis}{'+' if direction > 0 else '-'}", frame)
        if text is None:
            raise DriverError(-1, f"hold-jog in the {frame} frame is not verified on {self.dialect.label} yet",
                              "use step jog, or the base frame")
        self._ok(self._dash, text)

    def jog_release(self) -> None:
        self._ok(self._dash, cmd.move_jog(None))

    def stream_target(self, pose: Pose) -> None:
        s = self.cfg.stream
        self._streaming = True
        self._ok(self._motion, self.dialect.servo(pose, s.period_s, s.aheadtime, s.gain))

    def drag(self, on: bool) -> None:
        """StartDrag()/StopDrag(). The payload on the controller must match the real tool, or the arm drifts
        or sags in drag: check it in DobotStudio before the first drag."""
        self._ok(self._dash, cmd.start_drag() if on else cmd.stop_drag())

    def tool_port(self) -> DobotRs485Port | None:
        """RS485 on the tool flange, through the controller's HTTP server (if the robot config offers it)."""
        c = self.cfg.connection
        return DobotRs485Port(f"http://{c.ip}:{self.ports['http']}") if self.cfg.tool_port == "rs485" else None

    def request_control(self) -> None:
        """RequestControl() (V4 >= 4.6 only). Works before Connect, which is the point: Connect fails with
        'Not Tcp' until the controller is in TCP mode. The controller allows it only while disabled."""
        if not self.dialect.request_control:
            raise DriverError(-1, f"{self.dialect.label} has no RequestControl()",
                              "switch to TCP/IP mode in DobotStudio, then close DobotStudio")
        if self._connected:
            self._ok(self._dash, cmd.request_control())
            return
        c = self.cfg.connection
        dash = DashboardClient(c.ip, self.ports["dashboard"])
        dash.connect()
        try:
            dash.call_ok(cmd.request_control())
        finally:
            dash.close()

    # ---- Doctor ----

    @classmethod
    def offline_checks(cls, cfg: RobotConfig, sample_s: float = 1.0) -> list[Check]:
        """Feedback rate and TCP/IP mode without connecting (Doctor before Connect). Short own connections."""
        from botswain.drivers.dobot.feedback import FrameReader

        c, p = cfg.connection, dobot_ports(cfg)
        checks = [Check("Protocol", "ok", cls.dialect.label)]
        try:
            with socket.create_connection((c.ip, p["feedback"]), timeout=0.5) as sock:
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
        dash = DashboardClient(c.ip, p["dashboard"], timeout_s=1.0)
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

    def diagnose(self) -> list[Check]:
        checks: list[Check] = [Check("Protocol", "ok", self.dialect.label)]
        fb = self._feedback
        if fb is None or self._latest is None:
            checks.append(Check("Feedback 30004", "fail", "no packets", "is port 30004 reachable?"))
        else:
            hz = fb.hz
            status = "ok" if 100 <= hz <= 150 else "warn" if hz > 0 else "fail"
            checks.append(Check("Feedback 30004", status, f"{hz:.0f} Hz, {fb.bad_frames} bad frames",
                                "" if status == "ok" else "expected ~125 Hz; check the cable and the laptop load"))
        try:
            mode_reply = self._dash.call(cmd.robot_mode())
        except DriverError as e:
            return [*checks, Check("Dashboard 29999", "fail", e.message, "press Recover; " + e.hint)]
        if mode_reply.error_id == NOT_TCP:
            return [*checks, Check("TCP/IP mode", "fail", "controller answered 'Not Tcp'", HINT_TCP)]
        checks.append(Check("TCP/IP mode", "ok", f"dashboard answers ({self._dash.last_rtt_ms:.1f} ms)"))
        code = int(mode_reply.numbers()[0]) if mode_reply.numbers() else -1
        mode = self.dialect.modes.get(code, Mode.UNKNOWN)
        mode_status = "ok" if mode in (Mode.DISABLED, Mode.IDLE) else "fail" if mode in (
            Mode.ERROR, Mode.COLLISION) else "warn"
        checks.append(Check("Robot mode", mode_status, f"{code} ({self.dialect.mode_text.get(code, 'unknown')})",
                            "" if mode_status == "ok" else "expected disabled or enabled-idle before moving"))
        self._refresh_alarms()
        alarm_text = self._alarm_text() if self._alarm_ids else ""
        checks.append(Check("Alarms", "ok" if not self._alarm_ids else "fail",
                            "none" if not self._alarm_ids else f"{_alarm_words(self._alarm_ids)} {alarm_text}".strip(),
                            "" if not self._alarm_ids else "fix the cause, then Recover"))
        checks.append(self._motion_port_check())
        ctrl_max = self.cfg.limits.controller_speed_max_pct
        checks.append(Check("Speed scaling", "ok", f"speed control 100 % = SpeedFactor {ctrl_max} "
                            f"(now {self._speed_pct} % = {self._controller_pct(self._speed_pct)})"))
        checks.append(self._pose_check())
        return checks

    def _motion_port_check(self) -> Check:
        """Moves go to 29999 or, per the V3 doc, 30003. Reports whether 30003 accepts a connection and answers;
        sends only RobotMode() (never a move), and only when 30003 is not in use already."""
        c, p = self.cfg.connection, self.ports
        if p["motion"] != p["dashboard"]:
            return Check("Motion port", "ok", f"moves go to {p['motion']}")
        # Information, not a fault: either answer is a working setup today.
        try:
            with socket.create_connection((c.ip, 30003), timeout=1.0) as s:
                s.settimeout(1.0)
                s.sendall(b"RobotMode()\n")
                try:
                    answer = s.recv(256).decode("ascii", "replace").strip()
                except TimeoutError:
                    answer = ""
        except OSError as e:
            return Check("Motion port", "ok", f"moves go to 29999; 30003 not reachable ({type(e).__name__})")
        heard = f"answers ({answer[:40]!r})" if answer else "accepts connections but did not answer RobotMode()"
        return Check("Motion port", "ok", f"moves go to 29999; 30003 {heard}",
                     "the V3 doc puts moves on 30003: try connection.ports.motion: 30003 in the lab")

    def _pose_check(self) -> Check:
        latest = self._latest
        if latest is None:
            return Check("Pose agreement", "warn", "no feedback to compare")
        try:
            pose = self._dash.call_ok(cmd.get_pose()).numbers()
        except DriverError as e:
            return Check("Pose agreement", "warn", e.message, e.hint)
        fb = latest[0].tcp_actual
        d_mm = max(abs(a - b) for a, b in zip(pose[:3], fb[:3], strict=False))
        d_deg = max(abs((a - b + 180) % 360 - 180) for a, b in zip(pose[3:6], fb[3:6], strict=False))
        ok = d_mm <= 0.5 and d_deg <= 0.5
        return Check("Pose agreement", "ok" if ok else "warn", f"GetPose vs feedback: {d_mm:.2f} mm, {d_deg:.2f} deg",
                     "" if ok else "a user/tool frame may be active on the controller; check DobotStudio")

    def _alarm_text(self) -> str:
        url = f"http://{self.cfg.connection.ip}:{self.ports['http']}/protocol/getAlarm"
        try:
            with urllib.request.urlopen(url, timeout=1.0) as resp:
                body = resp.read().decode("utf-8", "replace")
            return json.dumps(json.loads(body))[:200] if body.strip().startswith(("{", "[")) else body[:200]
        except (OSError, ValueError) as e:
            return f"(alarm text unavailable: {e})"

    # ---- helpers ----

    def _ok(self, client: DashboardClient, text: str, timeout_s: float | None = None) -> None:
        try:
            client.call_ok(text, timeout_s)
        except DriverError as e:
            if e.code == -2:  # in alarm: fetch the alarm ids so the GUI can show them
                self._refresh_alarms()
            elif e.code == TIMEOUT:
                raise DriverError(e.code, e.message, "the controller stopped answering: press Recover") from e
            raise

    def _refresh_alarms(self) -> None:
        try:
            reply = self._dash.call(cmd.get_error_id())
        except DriverError as e:
            log.warning("GetErrorID failed: %s", e)
            return
        if reply.error_id == 0:
            self._alarm_ids = tuple(int(x) for x in re.findall(r"-?\d+", reply.values))


def _alarm_words(ids: tuple[int, ...]) -> str:
    if not ids:
        return "unknown (see the Doctor)"
    return ", ".join(ALARM_WORDS.get(i, str(i)) for i in ids)


class DobotV3Driver(DobotDriver):
    dialect = V3


class DobotV4Driver(DobotDriver):
    dialect = V4
