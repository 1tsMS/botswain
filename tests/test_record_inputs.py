"""End to end: a real LeRobot dataset, no camera, one episode per teleop input, through the real input classes.
Then read back with the dataset checker. Skipped when LeRobot is not installed (pip install -e .[record])."""

import time
from importlib.util import find_spec

import pytest
from helpers import ROBOTS, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Mode
from botswain.drivers.fake import FakeDriver
from botswain.record.session import RecordingSession, lerobot_writer_factory
from botswain.teleop.gamepad import GamepadTeleop, PadState
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.phone import PhoneSample
from botswain.teleop.session import TeleopSession
from botswain.teleop.vr import VrSession, parse_vr

pytestmark = pytest.mark.skipif(find_spec("lerobot") is None, reason="LeRobot not installed")


def drive(seconds: float, step) -> None:
    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        step(time.perf_counter() - (end - seconds))
        time.sleep(0.02)


def record_three_inputs(root, seconds: float = 1.5) -> None:
    """One episode each: gamepad (stick forward), phone (moved 4 cm), VR (hand 4 cm forward)."""
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    worker = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s))
    worker.start()
    try:
        worker.submit(Connect())
        worker.submit(Enable())
        assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
        session = RecordingSession(worker, lerobot_writer_factory("fake"))

        def episode(task, step, release):
            session.start(root, "botswain/selftest", task, 30, {})
            drive(seconds, step)
            release()
            assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=2)
            assert session.save() > 0

        pad = GamepadTeleop(worker, lambda: PadState(False))
        episode("gamepad: push forward",
                lambda t: worker.stream(pad.step(PadState(True, ly=1.0, buttons=frozenset({"lb"})), 0.02)),
                lambda: worker.stream(pad.step(PadState(True), 0.02)))

        phone = TeleopSession(worker, TeleopMapper(scale_pos=1.0))
        phone_sample = lambda t, hold=True: PhoneSample(now_ns(), (0.0, 0.0, -0.04 * min(t, 1.0)),
                                                         (1.0, 0.0, 0.0, 0.0), hold, False, 1.0)
        episode("phone: move 4 cm", lambda t: phone.on_sample(phone_sample(t)),
                lambda: phone.on_sample(phone_sample(1.0, hold=False)))

        vr = VrSession(worker, TeleopMapper(scale_pos=1.0))
        hand = lambda t, grip=0.9: {"tracked": True, "pos": [0.0, 1.0, -0.04 * min(t, 1.0)],
                                    "quat": [0, 0, 0, 1], "grip": grip, "trigger": 0.0}
        episode("vr: hand 4 cm forward", lambda t: vr.on_sample(parse_vr({"right": hand(t)}, now_ns())),
                lambda: vr.on_sample(parse_vr({"right": hand(1.0, grip=0.0)}, now_ns())))
        session.close()
    finally:
        worker.shutdown()


def test_every_input_is_recorded_and_tagged(tmp_path):
    from botswain.record.check import check_dataset

    root = tmp_path / "selftest"
    record_three_inputs(root)
    report = check_dataset(root)
    assert [e.task for e in report.episodes] == ["gamepad: push forward", "phone: move 4 cm",
                                                  "vr: hand 4 cm forward"]
    assert [e.main_source for e in report.episodes] == ["gamepad", "phone", "vr"]
    # No episode inherits the previous episode's input: idle frames are "none".
    assert all(set(e.sources) <= {e.main_source, "none"} for e in report.episodes), report.episodes
    for e in report.episodes:
        assert e.frames >= 30 and e.travel_mm > 10, e  # really moved (gamepad at 10 % = 10 mm/s), really sampled
        assert e.state_repeats == 0 and e.bad_values == 0, e
        assert 25 <= e.real_fps <= 35, e
    assert not report.problems
