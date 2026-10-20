"""Runs a Blockly program. The program is Blockly's JSON save format; this walks it block by block.

No code generation and no eval: every block type is handled here, unknown blocks are refused. Every motion
goes through worker.submit(), so the safety gate checks it like a button press. The runner waits for each
move to finish, and stops the whole program when STOP is pressed, the gate refuses a command, or the robot
reports an error.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections.abc import Callable, Iterator
from contextlib import contextmanager

from botswain.control.points import Point, PointStore
from botswain.control.worker import RobotWorker
from botswain.core.types import (
    Command,
    Gripper,
    JogStep,
    Joints,
    Mode,
    MoveTo,
    Pose,
    SetGripperForce,
    SetSpeed,
)

log = logging.getLogger(__name__)

ACCEPT_TIMEOUT_S = 2.0  # the worker must take the command within this
START_TIMEOUT_S = 0.5  # a move that shows no motion within this is treated as already done (tiny move)
MOVE_TIMEOUT_S = 120.0
GRIPPER_WAIT_S = 0.6
MAX_REPEAT = 10000
MAX_LOOP = 100000  # while / count loops stop after this many rounds
LOOP_PAUSE_S = 0.001


class ProgramError(Exception):
    pass


class ProgramRunner:
    def __init__(self, worker: RobotWorker, points: PointStore, home: tuple[float, ...] | None,
                 on_block: Callable[[str | None], None] | None = None,
                 log_cb: Callable[[str, str], None] | None = None) -> None:
        self.worker = worker
        self.points = points
        self.home = home
        self.on_block = on_block
        self.log = log_cb or (lambda level, text: None)
        self._abort = threading.Event()
        self._busy = threading.Lock()  # one program at a time: Blocks and the Points table share this runner

    @property
    def busy(self) -> bool:
        return self._busy.locked()

    def abort(self) -> None:
        self._abort.set()

    def run(self, program: dict) -> None:
        """Blocking: call from a thread. Raises ProgramError with the reason when it stops early."""
        starts = [b for b in program.get("blocks", {}).get("blocks", []) if b.get("type") == "botswain_start"]
        if len(starts) != 1:
            raise ProgramError("the program needs exactly one 'when Run is pressed' block")
        with self._claim():
            self._vars: dict[str, object] = {}
            try:
                self._stack(starts[0].get("inputs", {}).get("DO", {}).get("block"))
            finally:
                self._highlight(None)

    def run_points(self, points: list[Point], on_point: Callable[[str | None], None] | None = None) -> None:
        """Visit saved points in order: each by its own move (joint to its joints, or linear to its pose), then its
        wait. Blocking: call from a thread. Same rules as a program: gate refusal, STOP or an error ends it."""
        report = on_point or (lambda name: None)
        with self._claim():
            try:
                for p in points:
                    self._check_abort()
                    report(p.name)
                    self._move(MoveTo(p.pose, "linear") if p.move == "linear" else MoveTo(p.joints, "joint"))
                    if p.wait_s > 0:
                        self._sleep(p.wait_s)
            finally:
                report(None)

    @contextmanager
    def _claim(self) -> Iterator[None]:
        if not self._busy.acquire(blocking=False):
            raise ProgramError("a program is already running")
        try:
            self._abort.clear()
            yield
        finally:
            self._busy.release()

    # ---- walking (statements) ----

    def _stack(self, block: dict | None) -> None:
        while block is not None:
            self._check_abort()
            self._highlight(block.get("id"))
            self._exec(block)
            block = block.get("next", {}).get("block")

    def _exec(self, b: dict) -> None:
        f = b.get("fields", {})
        match b.get("type"):
            case "botswain_move_point":
                point = self._point(f.get("POINT", ""))
                linear = f.get("KIND") == "linear"
                self._move(MoveTo(point.pose, "linear") if linear else MoveTo(point.joints, "joint"))
            case "botswain_move_relative":
                self._move(JogStep(f["AXIS"], self._arg(b, "DIST"), "base"))
            case "botswain_move_joints":
                self._move(MoveTo(Joints(tuple(_num(f, f"J{i}") for i in range(1, 7))), "joint"))
            case "botswain_move_pose":
                pose = Pose(*(_num(f, k) for k in ("X", "Y", "Z", "RX", "RY", "RZ")))
                self._move(MoveTo(pose, "linear" if f.get("KIND") == "linear" else "joint"))
            case "botswain_home":
                if self.home is None:
                    raise ProgramError("Home is not confirmed in the robot config (home.confirmed)")
                self._move(MoveTo(Joints(self.home), "joint"))
            case "botswain_gripper":
                self._submit(Gripper(f.get("STATE") == "open"))
                self._sleep(GRIPPER_WAIT_S)
            case "botswain_gripper_force":
                pct = int(self._arg(b, "PCT"))
                if not 1 <= pct <= 100:
                    raise ProgramError(f"gripper force must be 1..100 %, got {pct}")
                self._submit(SetGripperForce(pct))
            case "botswain_speed":
                pct = int(self._arg(b, "PCT"))
                if not 1 <= pct <= 100:
                    raise ProgramError(f"speed must be 1..100 %, got {pct}")
                self._submit(SetSpeed(pct))
            case "botswain_wait":
                self._sleep(max(0.0, self._arg(b, "SECONDS")))
            case "botswain_log":
                self.log("info", f"program: {self._text(b, 'TEXT')}")
            case "controls_repeat_ext" | "controls_repeat":
                times = int(_num(f, "TIMES")) if "TIMES" in f else int(self._arg(b, "TIMES"))
                if not 0 <= times <= MAX_REPEAT:
                    raise ProgramError(f"repeat count must be 0..{MAX_REPEAT}, got {times}")
                for _ in range(times):
                    self._stack(_input(b, "DO"))
            case "controls_whileUntil":
                until = f.get("MODE") == "UNTIL"
                for n in range(MAX_LOOP + 1):
                    if n == MAX_LOOP:
                        raise ProgramError(f"loop ran {MAX_LOOP} times: stopped (a loop that never ends?)")
                    self._check_abort()
                    if bool(self._value(_input(b, "BOOL"))) == until:
                        break
                    self._stack(_input(b, "DO"))
                    time.sleep(LOOP_PAUSE_S)  # a loop with no motion in it must not spin a CPU core
            case "controls_for":
                var = _var_id(f)
                start, end, step = self._arg(b, "FROM"), self._arg(b, "TO"), abs(self._arg(b, "BY")) or 1.0
                step = step if end >= start else -step
                value, n = start, 0
                while (value <= end) if step > 0 else (value >= end):
                    n += 1
                    if n > MAX_LOOP:
                        raise ProgramError(f"count loop ran {MAX_LOOP} times: stopped")
                    self._vars[var] = value
                    self._stack(_input(b, "DO"))
                    value += step
            case "controls_if":
                extra = b.get("extraState", {})
                for k in range(int(extra.get("elseIfCount", 0)) + 1):
                    if self._value(_input(b, f"IF{k}")):
                        self._stack(_input(b, f"DO{k}"))
                        return
                if extra.get("hasElse") or "ELSE" in b.get("inputs", {}):
                    self._stack(_input(b, "ELSE"))
            case "variables_set":
                self._vars[_var_id(f)] = self._value(_input(b, "VALUE"))
            case "math_change":
                var = _var_id(f)
                self._vars[var] = _finite(self._vars.get(var, 0.0)) + self._arg(b, "DELTA")
            case other:
                raise ProgramError(f"block {other!r} is not supported")

    # ---- values (expressions) ----

    def _value(self, b: dict | None):
        """A value block -> number or bool. An empty slot is an error, never a silent 0."""
        if b is None:
            raise ProgramError("a value slot is empty")
        f = b.get("fields", {})
        match b.get("type"):
            case "math_number":
                return _num(f, "NUM")
            case "logic_boolean":
                return f.get("BOOL") == "TRUE"
            case "text":
                return str(f.get("TEXT", ""))
            case "math_arithmetic":
                a, c = self._arg(b, "A"), self._arg(b, "B")
                op = f.get("OP")
                if op == "DIVIDE" and c == 0:
                    raise ProgramError("division by zero")
                ops = {"ADD": lambda: a + c, "MINUS": lambda: a - c, "MULTIPLY": lambda: a * c,
                       "DIVIDE": lambda: a / c, "POWER": lambda: a ** c}
                if op not in ops:
                    raise ProgramError(f"arithmetic {op!r} is not supported")
                try:
                    return _finite(ops[op]())
                except (OverflowError, ZeroDivisionError, ValueError):
                    raise ProgramError(f"{a:g} {op.lower()} {c:g} has no finite result") from None
            case "logic_compare":
                a, c = self._value(_input(b, "A")), self._value(_input(b, "B"))
                ops = {"EQ": a == c, "NEQ": a != c}
                if f.get("OP") in ops:
                    return ops[f["OP"]]
                a, c = _finite(a), _finite(c)
                return {"LT": a < c, "LTE": a <= c, "GT": a > c, "GTE": a >= c}[f.get("OP", "LT")]
            case "logic_operation":
                a = bool(self._value(_input(b, "A")))
                if f.get("OP") == "AND":
                    return a and bool(self._value(_input(b, "B")))
                return a or bool(self._value(_input(b, "B")))
            case "logic_negate":
                return not self._value(_input(b, "BOOL"))
            case "variables_get":
                var = _var_id(f)
                if var not in self._vars:
                    raise ProgramError("a variable is read before it was set")
                return self._vars[var]
            case "botswain_tcp":
                return float(self.worker.latest_state().tcp.axis(f.get("AXIS", "X")))
            case "botswain_joint":
                return float(self.worker.latest_state().joints.axis(f.get("JOINT", "J1")))
            case "botswain_gripper_caught":
                return self.worker.latest_state().gripper_status == "object caught"
            case "botswain_gripper_opening":
                opening = self.worker.latest_state().gripper_pos
                if opening is None:
                    raise ProgramError("the gripper reports no opening")
                return opening * 100.0
            case other:
                raise ProgramError(f"value block {other!r} is not supported")

    def _arg(self, b: dict, name: str) -> float:
        """A number from a field (older programs) or from a value input (blocks with a slot)."""
        if name in b.get("fields", {}):
            return _num(b["fields"], name)
        return _finite(self._value(_input(b, name)))

    def _text(self, b: dict, name: str) -> str:
        if name in b.get("fields", {}):
            return str(b["fields"][name])
        value = self._value(_input(b, name))
        return f"{value:g}" if isinstance(value, float) else str(value)

    def _point(self, name: str):
        try:
            return self.points.get(name)
        except KeyError:
            raise ProgramError(f"saved point {name!r} does not exist") from None

    # ---- talking to the worker ----

    def _submit(self, cmd: Command) -> None:
        """Submit and wait until the worker took it; a gate refusal stops the program."""
        self.worker.submit(cmd)
        end = time.perf_counter() + ACCEPT_TIMEOUT_S
        while True:
            self._check_abort()
            rec = self.worker.last_command
            if rec is not None and rec.raw is cmd:
                if rec.sent is None:
                    raise ProgramError(f"{type(cmd).__name__} refused by the safety gate ({rec.rule})")
                return
            if time.perf_counter() > end:
                raise ProgramError(f"the robot worker did not take {type(cmd).__name__}")
            time.sleep(0.005)

    def _move(self, cmd: Command) -> None:
        self._submit(cmd)
        start = time.perf_counter()
        moving_seen = False
        while True:
            self._check_abort()
            mode = self.worker.latest_state().mode
            if mode in (Mode.ERROR, Mode.COLLISION, Mode.DISCONNECTED):
                raise ProgramError(f"robot is in {mode.value}")
            if mode in (Mode.MOVING, Mode.JOGGING):
                moving_seen = True
            elif mode is Mode.IDLE and (moving_seen or time.perf_counter() - start > START_TIMEOUT_S):
                return
            if time.perf_counter() - start > MOVE_TIMEOUT_S:
                raise ProgramError("move did not finish within 2 minutes")
            time.sleep(0.01)

    def _sleep(self, seconds: float) -> None:
        end = time.perf_counter() + seconds
        while time.perf_counter() < end:
            self._check_abort()
            time.sleep(min(0.02, max(0.0, end - time.perf_counter())))

    def _check_abort(self) -> None:
        if self._abort.is_set():
            raise ProgramError("stopped")

    def _highlight(self, block_id: str | None) -> None:
        if self.on_block is not None:
            self.on_block(block_id)


def _num(fields: dict, key: str) -> float:
    try:
        v = float(fields[key])
    except (KeyError, TypeError, ValueError):
        raise ProgramError(f"field {key} is missing or not a number") from None
    if not math.isfinite(v):
        raise ProgramError(f"field {key} is not a finite number")
    return v


def _finite(v) -> float:
    try:
        v = float(v)
    except (TypeError, ValueError):
        raise ProgramError(f"not a number: {v!r}") from None
    if not math.isfinite(v):
        raise ProgramError("not a finite number")
    return v


def _input(block: dict, name: str) -> dict | None:
    """The block plugged into a slot (a real block wins over the slot's default shadow)."""
    slot = block.get("inputs", {}).get(name, {})
    return slot.get("block") or slot.get("shadow")


def _var_id(fields: dict) -> str:
    var = fields.get("VAR")
    key = var.get("id") if isinstance(var, dict) else var
    if not key:
        raise ProgramError("a variable block has no variable")
    return str(key)
