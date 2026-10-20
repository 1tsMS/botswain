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
