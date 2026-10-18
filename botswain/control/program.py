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
from collections.abc import Callable

from botswain.control.points import PointStore
from botswain.control.worker import RobotWorker
from botswain.core.types import Command, Gripper, JogStep, Joints, Mode, MoveTo, Pose, SetSpeed

log = logging.getLogger(__name__)

ACCEPT_TIMEOUT_S = 2.0  # the worker must take the command within this
START_TIMEOUT_S = 0.5  # a move that shows no motion within this is treated as already done (tiny move)
MOVE_TIMEOUT_S = 120.0
GRIPPER_WAIT_S = 0.6
MAX_REPEAT = 10000


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

    def abort(self) -> None:
        self._abort.set()

    def run(self, program: dict) -> None:
        """Blocking: call from a thread. Raises ProgramError with the reason when it stops early."""
        self._abort.clear()
        starts = [b for b in program.get("blocks", {}).get("blocks", []) if b.get("type") == "botswain_start"]
        if len(starts) != 1:
            raise ProgramError("the program needs exactly one 'when Run is pressed' block")
        try:
            self._stack(starts[0].get("inputs", {}).get("DO", {}).get("block"))
        finally:
            self._highlight(None)

    # ---- walking ----

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
                name = f.get("POINT", "")
                try:
                    point = self.points.get(name)
                except KeyError:
                    raise ProgramError(f"saved point {name!r} does not exist") from None
                self._move(MoveTo(point.joints, "joint"))
            case "botswain_move_relative":
                self._move(JogStep(f["AXIS"], _num(f, "DIST"), "base"))
            case "botswain_move_joints":
                self._move(MoveTo(Joints(tuple(_num(f, f"J{i}") for i in range(1, 7))), "joint"))
            case "botswain_move_pose":
                pose = Pose(*(_num(f, k) for k in ("X", "Y", "Z", "RX", "RY", "RZ")))
                self._move(MoveTo(pose, "linear" if f.get("KIND") == "linear" else "joint"))
            case "botswain_home":
                if self.home is None:
                    raise ProgramError("Home is not confirmed in the robot YAML (home_confirmed)")
                self._move(MoveTo(Joints(self.home), "joint"))
            case "botswain_gripper":
                self._submit(Gripper(f.get("STATE") == "open"))
                self._sleep(GRIPPER_WAIT_S)
            case "botswain_speed":
                self._submit(SetSpeed(int(_num(f, "PCT"))))
            case "botswain_wait":
                self._sleep(max(0.0, _num(f, "SECONDS")))
            case "controls_repeat_ext" | "controls_repeat":
                times = int(_num(f, "TIMES")) if "TIMES" in f else _times(b)
                if not 0 <= times <= MAX_REPEAT:
                    raise ProgramError(f"repeat count must be 0..{MAX_REPEAT}, got {times}")
                body = b.get("inputs", {}).get("DO", {}).get("block")
                for _ in range(times):
                    self._stack(body)
            case other:
                raise ProgramError(f"block {other!r} is not supported")

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


def _times(block: dict) -> int:
    """controls_repeat_ext keeps its count in a math_number shadow/block on the TIMES input."""
    inp = block.get("inputs", {}).get("TIMES", {})
    inner = inp.get("block") or inp.get("shadow") or {}
    if inner.get("type") != "math_number":
        raise ProgramError("repeat count must be a plain number")
    return int(_num(inner.get("fields", {}), "NUM"))
