"""The robot worker: one background thread that owns the driver.

The GUI and every input only put commands in the queue. Each tick (default 8 ms) the worker reads the
newest state, handles STOP first, runs queued commands through the SafetyGate, calls the driver, runs the
hold-jog watchdog and publishes state at most `publish_hz`. Qt-free on purpose: the app wraps the callbacks
in Qt signals, which queue them onto the GUI thread.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, replace

from botswain.control.safety import Rejected, SafetyGate
from botswain.core.clock import now_ns
from botswain.core.log import LEVELS
from botswain.core.types import (
    ClearError,
    Command,
    Connect,
    Diagnose,
    Disable,
    Disconnect,
    Enable,
    Gripper,
    JogHold,
    JogRelease,
    JogStep,
    JogTick,
    Mode,
    MoveTo,
    Pose,
    RequestControl,
    RobotState,
    SetGripperForce,
    SetSpeed,
    Stop,
    StreamTarget,
)
from botswain.drivers.base import Check, Driver, DriverError

log = logging.getLogger(__name__)

HOLD_TIMEOUT_NS = 150_000_000  # no "still held" tick for this long -> release the hold-jog
RULE_LOG_PERIOD_NS = 1_000_000_000  # at most one log line per second per gate rule
STOP_LOOKAHEAD_S = 0.06  # release a hold-jog this long before it would reach a limit (stop latency)
HOLD_IDLE_NS = 300_000_000  # robot idle this long into a hold-jog: the controller ended the jog by itself


@dataclass(frozen=True)
class CommandRecord:
    """What came in (raw), what went to the driver after the gate (sent, None if rejected) and which rule acted."""

    t_ns: int
    raw: Command
    sent: Command | None
    rule: str | None


@dataclass(frozen=True)
class Commanded:
    """The last pose target we sent (after the gate), what came in before the gate, and the gripper command.

    The recorder logs this next to the measured state, so a dataset holds commanded AND measured actions.
    """

    t_ns: int = 0
    raw: Pose | None = None
    sent: Pose | None = None
    gripper_open: bool | None = None


class RobotWorker:
    def __init__(self, driver: Driver, gate: SafetyGate, tick_s: float = 0.008, publish_hz: float = 30.0) -> None:
        self.driver = driver
        self.gate = gate
        self.tick_s = tick_s
        self.publish_period_ns = int(1e9 / publish_hz)
        self.on_state: Callable[[RobotState], None] | None = None
        self.on_log: Callable[[str, str], None] | None = None
        self.on_teleport: Callable[[], None] | None = None  # the input should re-anchor (teleop mapping)
        self.last_command: CommandRecord | None = None
        self.commanded = Commanded()  # replaced as a whole, so readers on other threads see a consistent one

        self._queue: queue.SimpleQueue[Command] = queue.SimpleQueue()
        self._stop_requested = threading.Event()
        self._quit = threading.Event()
        self._thread = threading.Thread(target=self._run, name="botswain-worker", daemon=True)
        self._state = RobotState.disconnected(driver.capabilities.dof)
        self._state_lock = threading.Lock()
        self._connected = False
        self._hold_tick_ns: int | None = None  # last "still held" time while a hold-jog runs, else None
        self._hold_cmd: JogHold | None = None
        self._hold_start_ns = 0
        self._hold_idle_logged = False
        self._prev_tcp: tuple[float, float, float, int] | None = None  # for the TCP speed estimate
        self.stream_period_ns = int(gate.period_s * 1e9)
        self._stream_latest: StreamTarget | None = None  # newest teleop target, replaced by every new sample
        self._stream_input_ns = 0  # arrival time of the newest sample
        self._streaming = False
        self._last_stream_ns = 0
        self._stream_latched = False  # after STOP: no streaming until the operator releases hold-to-move
        self._next_publish_ns = 0
        self._last_rule_log_ns: dict[str, int] = {}

    # ---- thread-safe API (any thread) ----

    def start(self) -> None:
        self._thread.start()

    def submit(self, cmd: Command) -> None:
        if isinstance(cmd, Stop):
            self.request_stop()
        else:
            self._queue.put(cmd)

    def stream(self, cmd: StreamTarget) -> None:
        """Teleop input (any thread). Not queued: only the newest target matters, one goes out per period."""
        self._stream_input_ns = cmd.t_ns
        self._stream_latest = cmd

    def request_stop(self) -> None:
        """STOP lane: jumps the queue, clears it, ends any jog or stream, calls driver.stop()."""
        self._stop_requested.set()

    def shutdown(self, timeout_s: float = 2.0) -> None:
        """Stop, disconnect, join. Safe to call twice or before start()."""
        if not self._thread.is_alive():
            return
        self._quit.set()
        self._thread.join(timeout_s)
        if self._thread.is_alive():
            log.error("worker did not stop within %.1f s", timeout_s)

    def latest_state(self) -> RobotState:
        with self._state_lock:
            return self._state

    @property
    def pending(self) -> int:
        return self._queue.qsize()

    @property
    def running(self) -> bool:
        return self._thread.is_alive()

    # ---- the loop (worker thread only) ----

    def _run(self) -> None:
        next_tick = time.perf_counter()
        try:
            while not self._quit.is_set():
                try:
                    self._tick()
                except Exception:  # a bug in a tick must not kill the robot's only owner
                    log.exception("worker tick crashed")
                    self._log_rule("tick", "error", "worker tick crashed, see log file")
                next_tick += self.tick_s
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    # time.sleep is high resolution on Windows since Python 3.11; Event.wait is not.
                    time.sleep(delay)
                else:
                    next_tick = time.perf_counter()  # fell behind (slow driver call): don't try to catch up
        finally:
            self._close_driver()

    def _tick(self) -> None:
        state = self._read_state()
        while True:
            if self._stop_requested.is_set():
                self._handle_stop()
                state = self._read_state()
                break
            try:
                cmd = self._queue.get_nowait()
            except queue.Empty:
                break
            self._handle(cmd, state)
            state = self._read_state()  # the next command must see what this one changed
        self._watchdog(state)
        self._stream_tick(state)
        self._publish(state)

    def _handle_stop(self) -> None:
        self._stop_requested.clear()
        dropped = 0
        while True:
            try:
                self._queue.get_nowait()
                dropped += 1
            except queue.Empty:
                break
        self._hold_tick_ns = None
        if self._streaming or self._stream_latest is not None:
            self._stream_latched = True  # the phone keeps sending hold=true; don't restart on the next sample
        self._end_stream()
        self._call("stop", self.driver.stop)
        self._log("warn", f"STOP ({dropped} queued command(s) dropped)")

    def _handle(self, cmd: Command, state: RobotState) -> None:
        now = now_ns()
        if isinstance(cmd, JogTick):
            if self._hold_tick_ns is not None:
                self._hold_tick_ns = now
            return
        verdict = self.gate.check(cmd, state)
        name = type(cmd).__name__
        if isinstance(verdict, Rejected):
            self.last_command = CommandRecord(now, cmd, None, verdict.rule)
            self._log_rule(verdict.rule, "warn", f"{name} rejected ({verdict.rule}): {verdict.reason}")
            return
        rules = ",".join(verdict.clamped) or None
        self.last_command = CommandRecord(now, cmd, verdict.cmd, rules)
        if rules:
            self._log_rule(rules, "info", f"{name} clamped by {rules}")
        if isinstance(cmd, MoveTo) and isinstance(cmd.target, Pose):
            self.commanded = replace(self.commanded, t_ns=now, raw=cmd.target, sent=verdict.cmd.target)
        elif isinstance(cmd, Gripper):
            self.commanded = replace(self.commanded, gripper_open=cmd.open)
        self._execute(verdict.cmd)

    def _execute(self, cmd: Command) -> None:
        d = self.driver
        match cmd:
            case Connect():
                if self._call("connect", d.connect):
                    self._connected = True
                    self._log("info", f"connected: {d.capabilities.name}")
            case Disconnect():
                self._hold_tick_ns = None
                self._call("stop", d.stop)
                self._call("disconnect", d.disconnect)
                self._connected = False
                self._log("info", "disconnected")
            case Enable():
                self._call("enable", d.enable)
            case Disable():
                self._call("disable", d.disable)
            case ClearError():
                self._call("clear_error", d.clear_error)
            case SetSpeed(pct=pct):
                self._call("set_speed", d.set_speed, pct)
            case JogStep(axis=axis, delta=delta, frame=frame):
                self._call("jog_step", d.jog_step, axis, delta, frame)
            case JogHold(axis=axis, direction=direction, frame=frame):
                if self._call("jog_hold", d.jog_hold, axis, direction, frame):
                    self._hold_tick_ns = self._hold_start_ns = now_ns()
                    self._hold_cmd = cmd
                    self._hold_idle_logged = False
            case JogRelease():
                # Always sent, even if we think no hold is active: a release that does nothing is harmless.
                self._hold_tick_ns = None
                if d.capabilities.hold_jog:
                    self._call("jog_release", d.jog_release)
            case MoveTo(target=target, kind=kind):
                self._call("move_to", d.move_to, target, kind)
            case StreamTarget():
                self._log("error", "StreamTarget must go through worker.stream(), not the queue")
            case Gripper(open=is_open):
                self._call("gripper", d.gripper, is_open)
            case SetGripperForce(pct=pct):
                if self._call("gripper_force", d.set_gripper_force, pct):
                    self._log("info", f"gripper closing force {pct} %")
            case RequestControl():
                if self._call("request_control", d.request_control):
                    self._log("info", "controller control taken (RequestControl)")
            case Diagnose(reply=reply):
                try:
                    checks = d.diagnose()
                except Exception as e:
                    log.exception("diagnose crashed")
                    checks = [Check("Driver checks", "fail", f"{type(e).__name__}: {e}")]
                reply(checks)
            case _:
                self._log("error", f"worker has no handler for {cmd!r}")

    def _watchdog(self, state: RobotState) -> None:
        """Release a hold-jog when the GUI stops ticking (frozen, crashed) or the arm reaches a limit."""
        if self._hold_tick_ns is None:
            self._prev_tcp = None
            return
        if now_ns() - self._hold_tick_ns > HOLD_TIMEOUT_NS:
            self._hold_tick_ns = None
            self._log("warn", "hold-jog released by watchdog: no 'still held' tick for 150 ms")
            self._call("jog_release", self.driver.jog_release)
            return
        if (state.mode is Mode.IDLE and not self._hold_idle_logged
                and now_ns() - self._hold_start_ns > HOLD_IDLE_NS):
            # Diagnosis only: nothing is re-sent. Tells "the controller stopped the jog" apart from a release.
            self._hold_idle_logged = True
            self._log("warn", "the robot stopped the hold-jog by itself (idle while the key is still held)")
        # Predictive: margin = 1 mm + distance covered during the stop latency, from the measured TCP speed.
        x, y, z = state.tcp.xyz
        speed = 0.0
        if self._prev_tcp is not None and state.t_ns > self._prev_tcp[3]:
            px, py, pz, pt = self._prev_tcp
            speed = ((x - px) ** 2 + (y - py) ** 2 + (z - pz) ** 2) ** 0.5 / ((state.t_ns - pt) / 1e9)
        self._prev_tcp = (x, y, z, state.t_ns)
        margin = 1.0 + speed * STOP_LOOKAHEAD_S
        why = (self.gate.hold_violation(self._hold_cmd, state, margin_mm=margin)
               if self._hold_cmd is not None else None)
        if why is not None:
            self._hold_tick_ns = None
            self.gate.counts[why[0]] = self.gate.counts.get(why[0], 0) + 1
            self._log("warn", f"hold-jog released ({why[0]}): {why[1]}")
            self._call("jog_release", self.driver.jog_release)

    def _stream_tick(self, state: RobotState) -> None:
        now = now_ns()
        cmd = self._stream_latest
        if cmd is None:
            if self._streaming and now - self._stream_input_ns > self.gate.limits.stale_ms * 1e6:
                self._log("warn", "teleop input went silent, stream stopped")
                self._stop_stream()
            return
        if now - self._last_stream_ns < self.stream_period_ns:
            return
        self._stream_latest = None
        if self._stream_latched:
            if not cmd.hold:
                self._stream_latched = False
                self._log("info", "hold-to-move released after STOP, teleop can start again")
            return
        if not self._streaming:
            if not cmd.hold:
                return  # idle phone, nothing to do
            self.gate.reset_stream(state.tcp)  # seeded from the measured pose: the first target can't jump
        verdict = self.gate.check(cmd, state)
        if isinstance(verdict, Rejected):
            self.last_command = CommandRecord(now, cmd, None, verdict.rule)
            if verdict.rule in ("hold", "stale", "ready"):
                if self._streaming:
                    self._stop_stream()
            elif verdict.rule == "teleport" and self.on_teleport is not None:
                self.on_teleport()
            if verdict.rule != "hold":
                self._log_rule(verdict.rule, "warn", f"stream rejected ({verdict.rule}): {verdict.reason}")
            return
        rules = ",".join(verdict.clamped) or None
        self.last_command = CommandRecord(now, cmd, verdict.cmd, rules)
        if rules:
            self._log_rule(f"stream:{rules}", "info", f"stream clamped by {rules}")
        self.commanded = replace(self.commanded, t_ns=now, raw=cmd.pose, sent=verdict.cmd.pose)
        if self._call("stream_target", self.driver.stream_target, verdict.cmd.pose):
            self._streaming = True
            self._last_stream_ns = now
        else:
            # Do not hammer the controller 33 times a second with a refused command: stop the stream and
            # wait for the operator to release hold-to-move before trying again.
            self._stream_latched = True
            self._stop_stream()
            self._log("error", "teleop stream stopped after a refused command; release hold to try again")

    def _stop_stream(self) -> None:
        self._end_stream()
        self._call("stop", self.driver.stop)

    def _end_stream(self) -> None:
        self._streaming = False
        self._stream_latest = None
        self.gate.reset_stream(None)

    def _read_state(self) -> RobotState:
        try:
            state = self.driver.read_state()
        except Exception as e:  # noqa: BLE001 -- keep the last state, mark the link dead so motion is refused
            self._log_rule("read_state", "error", f"read_state failed: {e}")
            state = replace(self.latest_state(), link_ok=False, feedback_hz=0.0)
        with self._state_lock:
            self._state = state
        return state

    def _publish(self, state: RobotState) -> None:
        now = now_ns()
        if now < self._next_publish_ns:
            return
        # Fixed grid, so 8 ms ticks still average the full publish rate instead of rounding down.
        self._next_publish_ns += self.publish_period_ns
        if self._next_publish_ns <= now:
            self._next_publish_ns = now + self.publish_period_ns
        if self.on_state is not None:
            try:
                self.on_state(state)
            except Exception:
                log.exception("on_state callback crashed")

    def _close_driver(self) -> None:
        if not self._connected:
            return
        self._call("stop", self.driver.stop)
        self._call("disconnect", self.driver.disconnect)
        self._connected = False

    # ---- helpers ----

    def _call(self, name: str, fn: Callable[..., None], *args: object) -> bool:
        """Run one driver call. Errors are logged with their hint and the loop goes on."""
        try:
            fn(*args)
            return True
        except DriverError as e:
            hint = f". {e.hint}" if e.hint else ""
            self._log("error", f"{name} failed: {e.message} (code {e.code}){hint}")
        except Exception as e:  # a driver bug: full traceback to the file, short line to the GUI
            log.exception("driver.%s crashed", name)
            self._emit("error", f"{name} crashed: {type(e).__name__}: {e}")
        return False

    def _log(self, level: str, text: str) -> None:
        log.log(LEVELS[level], text)
        self._emit(level, text)

    def _emit(self, level: str, text: str) -> None:
        if self.on_log is not None:
            try:
                self.on_log(level, text)
            except Exception:
                log.exception("on_log callback crashed")

    def _log_rule(self, key: str, level: str, text: str) -> None:
        now = now_ns()
        if now - self._last_rule_log_ns.get(key, -RULE_LOG_PERIOD_NS) >= RULE_LOG_PERIOD_NS:
            self._last_rule_log_ns[key] = now
            self._log(level, text)
