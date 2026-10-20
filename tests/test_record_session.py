"""Recording session rules, plus a real LeRobot round trip from a running fake robot."""

import time

import numpy as np
import pytest
from helpers import ROBOTS, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, JogStep, Mode
from botswain.drivers.fake import FakeDriver
from botswain.record.session import RecordingSession, lerobot_writer_factory


class FakeWriter:
    def __init__(self, *args):
        self.args, self.frames, self.saved, self.final = args, [], 0, False

    def add(self, frame):
        self.frames.append(frame)

    def save(self):
        self.saved += 1

    def discard(self):
        self.frames = []

    def finalize(self):
        self.final = True


class Cam:
    def __init__(self, h=48, w=64):
        self.shape = (h, w, 3)

    def latest(self):
        return np.zeros(self.shape, dtype=np.uint8), now_ns()


@pytest.fixture
def worker():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s))
    w.start()
    w.submit(Connect())
    w.submit(Enable())
    assert wait_for(lambda: w.latest_state().mode is Mode.IDLE)
    yield w
    w.shutdown()


def test_session_rules(worker, tmp_path):
    writers = []
    session = RecordingSession(worker, lambda *a: writers.append(FakeWriter(*a)) or writers[-1])
    with pytest.raises(ValueError, match="task"):
        session.start(tmp_path / "ds", "x/ds", "  ", 30, {"front": Cam()})
    session.start(tmp_path / "ds", "x/ds", "pick", 30, {"front": Cam()})
    time.sleep(0.2)
    assert session.save() > 0 and session.episodes_saved == 1
    session.start(tmp_path / "ds", "x/ds", "pick", 30, {"front": Cam()})  # same dataset, second episode
    session.discard()
    with pytest.raises(ValueError, match="changed"):
        session.start(tmp_path / "ds", "x/ds", "pick", 30, {"front": Cam(96, 128)})
    session.close()
    assert len(writers) == 1 and writers[0].final and writers[0].saved == 1


def test_existing_folder_is_never_overwritten(worker, tmp_path):
    (tmp_path / "ds").mkdir()
    (tmp_path / "ds" / "keep.txt").write_text("old data")
    session = RecordingSession(worker, FakeWriter)
    with pytest.raises(ValueError, match="already holds data"):
        session.start(tmp_path / "ds", "x/ds", "pick", 30, {})


def test_camera_without_picture_is_refused(worker, tmp_path):
    class Dark:
        def latest(self):
            return None

    with pytest.raises(ValueError, match="no picture"):
        RecordingSession(worker, FakeWriter).start(tmp_path / "ds", "x/ds", "pick", 30, {"front": Dark()})


def test_lerobot_round_trip_from_fake_robot(worker, tmp_path):
    pytest.importorskip("lerobot")
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = tmp_path / "pick"
    session = RecordingSession(worker, lerobot_writer_factory("fake_cr5"))
    session.start(root, "botswain/pick", "pick the cube", 30, {"front": Cam()})
    worker.submit(JogStep("Z", -20.0))
    time.sleep(1.0)
    n = session.save()
    session.close()
    assert 28 <= n <= 32
    ds = LeRobotDataset("botswain/pick", root=root, video_backend="pyav")
    assert len(ds) == n and ds.num_episodes == 1
    first, last = ds[0], ds[n - 1]
    assert first["observation.images.front"].shape == (3, 48, 64)
    z = 6 + 2  # observation.state = 6 joints, then x, y, z ...
    assert float(last["observation.state"][z]) < float(first["observation.state"][z])  # the arm moved down
    assert first["task"] == "pick the cube"


def test_refuses_to_record_without_a_robot(tmp_path):
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg), SafetyGate(cfg.limits, cfg.stream.period_s))
    w.start()
    try:
        with pytest.raises(ValueError, match="not connected"):
            RecordingSession(w, FakeWriter).start(tmp_path / "ds", "x/ds", "pick", 30, {})
    finally:
        w.shutdown()


def _drive(worker, source, seconds=0.4):
    from botswain.core.types import StreamTarget

    end = time.perf_counter() + seconds
    while time.perf_counter() < end:
        p = worker.latest_state().tcp
        worker.stream(StreamTarget(p.with_axis("X", p.x + 1.0), now_ns(), source, True))
        time.sleep(0.02)
    p = worker.latest_state().tcp
    worker.stream(StreamTarget(p, now_ns(), source, False))
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)


def test_dataset_guard_flags_a_different_input(worker, tmp_path):
    # One dataset per input (training normalises with the dataset's own stats): a gamepad episode in a phone
    # dataset must be caught before it is saved.
    session = RecordingSession(worker, lambda *a: FakeWriter(*a))
    session.start(tmp_path / "d", "botswain/d", "pick", 30, {})
    _drive(worker, "phone")
    assert session.mismatch() is None  # first episode sets the dataset's input
    session.save()
    assert session.dataset_source == "phone"
    session.start(tmp_path / "d", "botswain/d", "pick", 30, {})
    _drive(worker, "gamepad")
    assert session.mismatch() == "this episode was driven by gamepad, but this dataset holds phone episodes"
    session.discard()
    session.start(tmp_path / "d", "botswain/d", "pick", 30, {})
    time.sleep(0.2)  # idle episode: no input, no complaint
    assert session.mismatch() is None
    session.save()
