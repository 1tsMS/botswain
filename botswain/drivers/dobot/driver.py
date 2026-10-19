"""DobotDriver: the driver contract on a Dobot V4 controller.

Commands go to the dashboard port (29999); ServoP can go to a separate motion port (30003) if the YAML says
so. State comes only from the 30004 feedback thread, so read_state() never touches a socket.
Connect never enables. stop() sends MoveJog() then Stop(), and disconnect() stops first.
"""

from __future__ import annotations

import json
import logging
import re
import threading
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
from botswain.drivers.dobot.dashboard import HINT_TCP, NO_LINK, NOT_TCP, DashboardClient
from botswain.drivers.dobot.feedback import STALE_NS, FeedbackPacket, FeedbackThread
from botswain.drivers.dobot.gripper import (
    FORCE_RANGE,
    GRIP_STATES,
    REG_CURRENT_POS,
    REG_GRIP_STATE,
    REG_INIT_STATE,
    DhGripper,
)

log = logging.getLogger(__name__)

MODE_MAP = {3: Mode.POWERED_OFF, 4: Mode.DISABLED, 5: Mode.IDLE, 6: Mode.DRAG, 7: Mode.MOVING, 8: Mode.JOGGING,
            9: Mode.ERROR, 10: Mode.PAUSED, 11: Mode.COLLISION}  # 1 init, 2 brake open -> UNKNOWN
MODE_TEXT = {1: "init", 2: "brake open", 3: "powered off", 4: "disabled", 5: "enabled, idle", 6: "drag mode",
             7: "running (queue)", 8: "single move (jog)", 9: "error (alarm)", 10: "paused", 11: "collision"}
START_SPEED_PCT = 10  # set on connect, so the first move after Enable is slow whatever the pendant had
ENABLE_TIMEOUT_S = 20.0  # EnableRobot answers only when enabling is done: over 5 s on the lab CR5
COORD_FRAME = {"base": 1, "tool": 2}  # MoveJog coordtype; joints use 0
GRIPPER_POLL_S = 0.25  # gripper status readout period
GRIPPER_RETRY_S = 2.0  # after a failed read, try again this much later


class DobotDriver(Driver):
    def __init__(self, cfg: RobotConfig) -> None:
        if cfg.connection is None:
            raise DriverError(-1, "dobot_v4 needs a connection section in the robot YAML")
        self.cfg = cfg
        c = cfg.connection
        self.capabilities = Capabilities(
            name=cfg.name, dof=6, step_jog=True, hold_jog=True, streaming=True,
            stream_rate_hz=1.0 / cfg.stream.period_s,
            gripper=cfg.gripper is not None and cfg.gripper.type == "dh_modbus", drag_mode=False,
            frames=("base", "tool"), max_tcp_speed_mm_s=cfg.limits.max_tcp_speed_mm_s,
            gripper_force=cfg.gripper is not None and cfg.gripper.type == "dh_modbus",
        )
        self._dash = DashboardClient(c.ip, c.dashboard_port)
        self._motion = self._dash if c.motion_port == c.dashboard_port else DashboardClient(c.ip, c.motion_port)
        self._feedback: FeedbackThread | None = None
        self._latest: tuple[FeedbackPacket, int] | None = None  # (packet, arrival ns), replaced atomically
        self._connected = False
        self._speed_pct = 0
        self._streaming = False
        self._alarm_ids: tuple[int, ...] = ()
        self._gripper_dev = (DhGripper(f"http://{c.ip}:{c.http_port}", cfg.gripper.slave)
                             if self.capabilities.gripper and cfg.gripper else None)
        self._gripper_ready = False
        self._gripper_pos: float | None = None
        self._gripper_force: int | None = None  # what the gripper's force register holds, as far as we know
        self._gripper_force_wanted: int | None = None  # set before init: written right after it
        self._gripper_meas: tuple[float | None, str] = (None, "")  # (opening 0..1, status), from the poll thread
        self._gripper_poll: threading.Thread | None = None
        self._gripper_poll_stop = threading.Event()
        self.legacy_syntax: bool | None = None  # None = not known yet, learned from the first refusal
        self._forms: dict[str, int] = {}  # command kind -> index of the form this controller accepts

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
            self._feedback = FeedbackThread(c.ip, c.feedback_port, self._on_packet)
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
        self._gripper_ready = False
        self._gripper_force = None
        self._alarm_ids = ()
        if self._gripper_dev is not None:
            self._gripper_poll_stop.clear()
            self._gripper_poll = threading.Thread(target=self._poll_gripper, name="gripper-poll", daemon=True)
            self._gripper_poll.start()

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
        self._gripper_poll_stop.set()
        if self._gripper_poll is not None:
            self._gripper_poll.join(2.0)
            self._gripper_poll = None
        self._gripper_meas = (None, "")
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
            return replace(RobotState.disconnected(6, now), mode=mode, speed_pct=self._speed_pct)
        p, t = latest
        mode = MODE_MAP.get(p.mode, Mode.UNKNOWN)
        if mode is Mode.MOVING and self._streaming:
            mode = Mode.STREAMING
        link_ok = now - t < STALE_NS and self._dash.connected
        return RobotState(
            t_ns=t, mode=mode, joints=Joints(p.q_actual), tcp=Pose(*p.tcp_actual), joints_target=Joints(p.q_target),
            tcp_target=Pose(*p.tcp_target), speed_pct=self._speed_pct,
            errors=self._alarm_ids if mode in (Mode.ERROR, Mode.COLLISION) else (),
            gripper=self._gripper_pos, gripper_pos=self._gripper_meas[0], gripper_status=self._gripper_meas[1],
            raw_mode=p.mode, link_ok=link_ok,
            feedback_hz=self._feedback.hz if self._feedback else 0.0,
        )

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
        """MoveJog() ends a hold-jog, Stop() clears the motion queue (and ServoP). Both are always tried.

        A refusal (e.g. ErrorID -2 in alarm) is fine: nothing is moving. Only a dead link raises, because
        then the stop did NOT reach the robot and the operator must use the physical e-stop.
        """
        self._streaming = False
        if not self._connected:
            return
        failures = []
        try:
            reply = self._dash.call(cmd.move_jog(None))  # ends a hold-jog in every firmware
            if reply.error_id != 0:
                log.info("MoveJog() answered %d (no jog running)", reply.error_id)
        except DriverError as e:
            failures.append(f"MoveJog(): {e.message}")
        # Stop() on V4, ResetRobot() on V3 firmware: the command that clears the motion queue and streams.
        stop_forms = [cmd.stop(), cmd.reset_robot()]
        if self._forms.get("stop") is not None:
            stop_forms = [stop_forms[self._forms["stop"]]]
        for i, text in enumerate(stop_forms):
            try:
                reply = self._dash.call(text)
            except DriverError as e:
                failures.append(f"{text}: {e.message}")
                break
            if reply.error_id == -10000 and i < len(stop_forms) - 1:
                log.warning("%s unknown on this controller, sending %s", text, stop_forms[i + 1])
                continue
            if reply.error_id == -10000:
                failures.append(f"{text}: unknown command on this controller")
            else:
                self._forms.setdefault("stop", [cmd.stop(), cmd.reset_robot()].index(text))
                if reply.error_id != 0:
                    log.info("%s answered %d (nothing to stop)", text, reply.error_id)
            break
        if failures:
            raise DriverError(NO_LINK, "STOP could not be sent: " + "; ".join(failures),
                              "use the physical e-stop, then check the cable and the controller")

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
        # MovJ(joint=) and MovJ(pose=) share a name but not a fallback, so each learns its own form.
        if isinstance(target, Joints):
            self._dialect(self._motion, cmd.mov_j_joints(target), cmd.joint_mov_j_legacy(target), kind="movj_joint")
        elif kind == "linear":
            self._dialect(self._motion, cmd.mov_l_pose(target), cmd.mov_l_pose_legacy(target))
        else:
            self._dialect(self._motion, cmd.mov_j_pose(target), cmd.mov_j_pose_legacy(target), kind="movj_pose")

    def jog_hold(self, axis: str, direction: int, frame: str) -> None:
        sign = "+" if direction > 0 else "-"
        coordtype = 0 if is_joint_axis(axis) else COORD_FRAME[frame]
        # Joint, base and tool jogs each learn their own form: the lab CR5 takes MoveJog(J1+) for joints but
        # refuses the matching positional form for Cartesian axes.
        kind = "jog_joint" if coordtype == 0 else f"jog_{frame}"
        self._dialect(self._dash, cmd.move_jog(f"{axis}{sign}", coordtype),
                      cmd.move_jog_legacy(f"{axis}{sign}", coordtype), kind=kind, jog=True)

    def _dialect(self, client: DashboardClient, modern: str, legacy: str | list[str], kind: str = "",
                 jog: bool = False) -> None:
        """Send the V4.6 keyword form; if the controller refuses it as not understood (nothing moved), try the
        older forms in order and remember what worked. jog: a frame mismatch (-6) or a parameter-range refusal
        also means "not this form" (old MoveJog reads an unknown frame as an out-of-range value), and a
        refused jog never moves."""
        forms = [modern, *([legacy] if isinstance(legacy, str) else legacy)]
        known = self._forms.get(kind or modern.split("(")[0])
        if known is not None:
            self._ok(client, forms[known])
            return
        for i, text in enumerate(forms):
            try:
                self._ok(client, text)
            except DriverError as e:
                last = i == len(forms) - 1
                if last or not (_syntax_refusal(e.code) or (jog and (e.code == -6 or -49999 <= e.code <= -40001))):
                    raise
                log.warning("%s not understood (%d), trying the next form", text, e.code)
                continue
            self._forms[kind or modern.split("(")[0]] = i
            if i > 0:
                self.legacy_syntax = True
                log.warning("this controller uses the older Dobot command form: %s", text)
            elif self.legacy_syntax is None:
                self.legacy_syntax = False
            return

    def jog_release(self) -> None:
        self._ok(self._dash, cmd.move_jog(None))

    def stream_target(self, pose: Pose) -> None:
        s = self.cfg.stream
        self._streaming = True
        self._dialect(self._motion, cmd.servo_p(pose, s.period_s, s.aheadtime, s.gain), cmd.servo_p_legacy(pose),
                      kind="servop")

    def gripper(self, open: bool) -> None:
        if self._gripper_dev is None:
            raise DriverError(-1, "no DH gripper configured in the robot YAML")
        if not self._gripper_ready:
            log.info("gripper init reply: %s", self._gripper_dev.init())
            self._gripper_ready = True
        wanted = self._gripper_force_wanted
        if wanted is not None and wanted != self._gripper_force:
            self._write_force(wanted)
        reply = self._gripper_dev.open() if open else self._gripper_dev.close()
        log.info("gripper %s reply: %s", "open" if open else "close", reply)
        self._gripper_pos = 1.0 if open else 0.0  # commanded, not measured: DH position read-back comes later

    def set_gripper_force(self, pct: int) -> None:
        """Closing force for every later close (the gripper keeps it). Before init it waits for the first close."""
        if self._gripper_dev is None:
            raise DriverError(-1, "no DH gripper configured in the robot YAML")
        lo, hi = FORCE_RANGE
        if not lo <= pct <= hi:
            raise DriverError(-1, f"gripper force must be {lo}..{hi} %, got {pct}")
        self._gripper_force_wanted = int(pct)
        if self._gripper_ready:
            self._write_force(int(pct))

    def _write_force(self, pct: int) -> None:
        log.info("gripper force %d %% reply: %s", pct, self._gripper_dev.set_force(pct))
        self._gripper_force = pct

    def _poll_gripper(self) -> None:
        """Gripper status for the GUI, read-only. Never initialises: DH init moves the jaws."""
        dev, failed, period = self._gripper_dev, False, 0.0
        while not self._gripper_poll_stop.wait(period):
            try:
                if not self._gripper_ready:
                    if dev.read(REG_INIT_STATE) != 1:
                        self._gripper_meas = (None, "not initialised: press Open or Close once")
                        period = 1.0
                        continue
                    self._gripper_ready = True  # already initialised (e.g. by DobotStudio): no init motion needed
                state, pos = dev.read(REG_GRIP_STATE), dev.read(REG_CURRENT_POS)
                self._gripper_meas = (min(max(pos / 1000.0, 0.0), 1.0), GRIP_STATES.get(state, f"state {state}"))
                if failed:
                    log.info("gripper readout working again")
                failed, period = False, GRIPPER_POLL_S
            except DriverError as e:
                if not failed:
                    log.warning("gripper readout failed (retrying every %.0f s): %s", GRIPPER_RETRY_S, e.message)
                self._gripper_meas = (None, "no reading")
                failed, period = True, GRIPPER_RETRY_S

    def request_control(self) -> None:
        """RequestControl() (firmware >= 4.6). Works before Connect, which is the point: Connect fails with
        'Not Tcp' until the controller is in TCP mode. The controller allows it only while disabled."""
        if self._connected:
            self._ok(self._dash, cmd.request_control())
            return
        c = self.cfg.connection
        dash = DashboardClient(c.ip, c.dashboard_port)
        dash.connect()
        try:
            dash.call_ok(cmd.request_control())
        finally:
            dash.close()

    # ---- Doctor ----

    def diagnose(self) -> list[Check]:
        checks: list[Check] = []
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
            return [*checks, Check("Dashboard 29999", "fail", e.message, e.hint)]
        if mode_reply.error_id == NOT_TCP:
            return [*checks, Check("TCP/IP mode", "fail", "controller answered 'Not Tcp'", HINT_TCP)]
        checks.append(Check("TCP/IP mode", "ok", f"dashboard answers ({self._dash.last_rtt_ms:.1f} ms)"))
        code = int(mode_reply.numbers()[0]) if mode_reply.numbers() else -1
        mode_status = "ok" if code in (4, 5) else "fail" if code in (9, 11) else "warn"
        checks.append(Check("Robot mode", mode_status, f"{code} ({MODE_TEXT.get(code, 'unknown')})",
                            "" if mode_status == "ok" else "expected 4 (disabled) or 5 (idle) before moving"))
        self._refresh_alarms()
        alarm_text = self._alarm_text() if self._alarm_ids else ""
        checks.append(Check("Alarms", "ok" if not self._alarm_ids else "fail",
                            "none" if not self._alarm_ids else f"{list(self._alarm_ids)} {alarm_text}".strip(),
                            "" if not self._alarm_ids else "fix the cause, then Clear error"))
        syntax = {None: "not known yet (no move sent)", False: "V4.6 keyword form",
                  True: "older positional form (JointMovJ, CoordType=)"}[self.legacy_syntax]
        checks.append(Check("Command syntax", "ok", syntax))
        ctrl_max = self.cfg.limits.controller_speed_max_pct
        checks.append(Check("Speed scaling", "ok", f"speed control 100 % = SpeedFactor {ctrl_max} "
                            f"(now {self._speed_pct} % = {self._controller_pct(self._speed_pct)})"))
        checks.append(self._pose_check())
        return checks

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
        url = f"http://{self.cfg.connection.ip}:{self.cfg.connection.http_port}/protocol/getAlarm"
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
            raise

    def _refresh_alarms(self) -> None:
        try:
            reply = self._dash.call(cmd.get_error_id())
        except DriverError as e:
            log.warning("GetErrorID failed: %s", e)
            return
        if reply.error_id == 0:
            self._alarm_ids = tuple(int(x) for x in re.findall(r"-?\d+", reply.values))


def _syntax_refusal(code: int) -> bool:
    """-10000 unknown command, -20000 wrong parameter count, -3000X / -5000X parameter type: the controller did
    not understand the command, so nothing moved and another form may be tried. Range errors (-4000X) are not."""
    return code in (-10000, -20000) or -39999 <= code <= -30000 or -59999 <= code <= -50000
