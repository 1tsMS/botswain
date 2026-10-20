"""Blockly programs run by ProgramRunner on the fake robot (no browser involved)."""

import itertools
import threading
import time

import pytest
from helpers import ROBOTS, dh_tool, wait_for

from botswain.control.points import PointStore
from botswain.control.program import ProgramError, ProgramRunner
from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Mode
from botswain.drivers.fake import FakeDriver


@pytest.fixture
def rig(tmp_path):
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s), tool=dh_tool())
    w.start()
    w.submit(Connect())
    w.submit(Enable())
    assert wait_for(lambda: w.latest_state().mode is Mode.IDLE)
    points = PointStore(tmp_path / "p.json")
    seen = []
    runner = ProgramRunner(w, points, cfg.home_joints_deg, on_block=seen.append)
    yield w, points, runner, seen
    w.shutdown()


def chain(*blocks):
    """Link blocks with 'next' like Blockly's JSON does."""
    for a, b in itertools.pairwise(blocks):
        a["next"] = {"block": b}
    return blocks[0]


def program(*blocks):
    return {"blocks": {"languageVersion": 0, "blocks": [
        {"type": "botswain_start", "id": "start", "inputs": {"DO": {"block": chain(*blocks)}}}]}}


def b(kind, block_id="", **fields):
    return {"type": kind, "id": block_id or kind, "fields": fields}


def test_relative_moves_speed_and_wait(rig):
    w, _, runner, seen = rig
    z0 = w.latest_state().tcp.z
    runner.run(program(b("botswain_speed", PCT=50), b("botswain_move_relative", "m1", AXIS="Z", DIST=-30),
                       b("botswain_wait", SECONDS=0.1), b("botswain_move_relative", "m2", AXIS="Z", DIST=10)))
    assert w.latest_state().tcp.z == pytest.approx(z0 - 20, abs=1e-3)
    assert w.latest_state().speed_pct == 50
    assert seen[:2] == ["botswain_speed", "m1"] and seen[-1] is None  # highlight follows, then clears


def test_repeat_and_gripper(rig):
    w, _, runner, _ = rig
    z0 = w.latest_state().tcp.z
    body = chain(b("botswain_gripper", STATE="close"), b("botswain_move_relative", AXIS="Z", DIST=-5))
    loop = {"type": "controls_repeat_ext", "id": "loop", "inputs": {
        "TIMES": {"shadow": {"type": "math_number", "fields": {"NUM": 3}}}, "DO": {"block": body}}}
    runner.run(program(b("botswain_speed", PCT=100), loop))
    assert w.latest_state().tcp.z == pytest.approx(z0 - 15, abs=1e-3)
    assert w.latest_state().gripper == 0.0


def test_move_to_saved_point(rig):
    w, points, runner, _ = rig
    start = w.latest_state()
    points.add("pick", start.tcp, start.joints)
    runner.run(program(b("botswain_speed", PCT=100), b("botswain_move_relative", AXIS="X", DIST=40),
                       b("botswain_move_point", POINT="pick")))
    assert w.latest_state().joints.values == pytest.approx(start.joints.values, abs=1e-6)
    with pytest.raises(ProgramError, match="does not exist"):
        runner.run(program(b("botswain_move_point", POINT="gone")))


def test_gate_refusal_stops_the_program(rig):
    _w, _, runner, _ = rig
    with pytest.raises(ProgramError, match="refused by the safety gate"):
        runner.run(program(b("botswain_move_joints", J1=0, J2=0, J3=170, J4=0, J5=-90, J6=0),  # J3 limit 160
                           b("botswain_move_relative", AXIS="Z", DIST=10)))


def test_stop_aborts_mid_move(rig):
    w, _, runner, _ = rig
    errors = []

    def go():
        try:
            runner.run(program(b("botswain_speed", PCT=5), b("botswain_move_relative", AXIS="Z", DIST=-200),
                               b("botswain_move_relative", AXIS="Z", DIST=200)))
        except ProgramError as e:
            errors.append(str(e))

    t = threading.Thread(target=go)
    t.start()
    assert wait_for(lambda: w.latest_state().mode is Mode.MOVING)
    runner.abort()
    w.request_stop()
    t.join(2.0)
    assert errors == ["stopped"]
    assert wait_for(lambda: w.latest_state().mode is Mode.IDLE)
    z = w.latest_state().tcp.z
    time.sleep(0.2)
    assert w.latest_state().tcp.z == pytest.approx(z, abs=1e-6)  # nothing more was sent


def test_program_shape_is_checked(rig):
    _, _, runner, _ = rig
    with pytest.raises(ProgramError, match="exactly one"):
        runner.run({"blocks": {"blocks": []}})
    with pytest.raises(ProgramError, match="not supported"):
        runner.run(program(b("text_print")))
    with pytest.raises(ProgramError, match="finite"):
        runner.run(program(b("botswain_move_relative", AXIS="X", DIST="nan")))


def num(n):
    return {"shadow": {"type": "math_number", "fields": {"NUM": n}}}


def value(kind, **fields):
    return {"block": {"type": kind, "fields": fields}}


def test_if_else_on_robot_state_and_variables(rig):
    w, _, runner, _ = rig
    z0 = w.latest_state().tcp.z
    var = {"VAR": {"id": "v1"}}
    set_d = {"type": "variables_set", "id": "set", "fields": var, "inputs": {"VALUE": num(-12)}}
    move = {"type": "botswain_move_relative", "id": "mv", "fields": {"AXIS": "Z"},
            "inputs": {"DIST": value("variables_get", **var)}}
    up = {"type": "botswain_move_relative", "id": "up", "fields": {"AXIS": "Z"}, "inputs": {"DIST": num(99)}}
    cond = {"block": {"type": "logic_compare", "fields": {"OP": "GT"},
                      "inputs": {"A": value("botswain_tcp", AXIS="Z"), "B": num(z0 + 500)}}}
    branch = {"type": "controls_if", "id": "if", "extraState": {"hasElse": True},
              "inputs": {"IF0": cond, "DO0": {"block": up}, "ELSE": {"block": move}}}
    runner.run(program(b("botswain_speed", PCT=100), set_d, branch))
    assert w.latest_state().tcp.z == pytest.approx(z0 - 12, abs=1e-3)  # Z is not > z0+500: the else branch ran


def test_count_loop_and_arithmetic(rig):
    w, _, runner, _ = rig
    x0 = w.latest_state().tcp.x
    var = {"VAR": {"id": "i"}}
    step = {"type": "botswain_move_relative", "id": "mv", "fields": {"AXIS": "X"}, "inputs": {"DIST": {"block": {
        "type": "math_arithmetic", "fields": {"OP": "MULTIPLY"},
        "inputs": {"A": value("variables_get", **var), "B": num(2)}}}}}
    loop = {"type": "controls_for", "id": "for", "fields": var,
            "inputs": {"FROM": num(1), "TO": num(3), "BY": num(1), "DO": {"block": step}}}
    runner.run(program(b("botswain_speed", PCT=100), loop))
    assert w.latest_state().tcp.x == pytest.approx(x0 + 12, abs=1e-3)  # 2 + 4 + 6


def test_endless_while_is_stopped(rig, monkeypatch):
    from botswain.control import program as prog

    _, _, runner, _ = rig
    monkeypatch.setattr(prog, "MAX_LOOP", 50)
    monkeypatch.setattr(prog, "LOOP_PAUSE_S", 0.0)
    loop = {"type": "controls_whileUntil", "id": "w", "fields": {"MODE": "WHILE"},
            "inputs": {"BOOL": value("logic_boolean", BOOL="TRUE"),
                       "DO": {"block": {"type": "botswain_log", "id": "l", "fields": {"TEXT": "x"}}}}}
    with pytest.raises(ProgramError, match="never ends"):
        runner.run(program(loop))


def test_gripper_values_and_force(rig):
    w, _, runner, _ = rig
    w.driver.tool_bus.object_permille = 400
    caught = {"type": "variables_set", "id": "c", "fields": {"VAR": {"id": "got"}},
              "inputs": {"VALUE": value("botswain_gripper_caught")}}
    force = {"type": "botswain_gripper_force", "id": "f", "inputs": {"PCT": num(35)}}
    runner.run(program(force, b("botswain_gripper", STATE="close"), b("botswain_wait", SECONDS=0.6), caught))
    assert runner._vars["got"] is True
    assert w.latest_state().gripper_pos == pytest.approx(0.4)


def test_empty_slot_and_unknown_blocks_are_refused(rig):
    _, _, runner, _ = rig
    with pytest.raises(ProgramError, match="empty"):
        runner.run(program({"type": "botswain_wait", "id": "w", "inputs": {}}))
    with pytest.raises(ProgramError, match="not supported"):
        runner.run(program({"type": "procedures_callnoreturn", "id": "p"}))
