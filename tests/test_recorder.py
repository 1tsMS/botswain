"""Recorder timing and commanded-vs-measured labels, with a fake writer (no LeRobot, no torch)."""

import itertools
import time

import numpy as np
import pytest
from helpers import ROBOTS, wait_for

from botswain.control.safety import SafetyGate
from botswain.control.worker import RobotWorker
from botswain.core.clock import now_ns
from botswain.core.config import load_robot_config
from botswain.core.types import Connect, Enable, Gripper, Mode, StreamTarget
from botswain.drivers.fake import FakeDriver
from botswain.record.recorder import Recorder, build_features


class FakeWriter:
    def __init__(self):
        self.frames, self.episodes, self.discarded, self.finalized = [], [], 0, False

    def add(self, frame):
        self.frames.append(frame)

    def save(self):
        self.episodes.append(self.frames)
        self.frames = []

    def discard(self):
        self.frames = []
        self.discarded += 1

    def finalize(self):
        self.finalized = True


class FakeCam:
    def latest(self):
        return np.zeros((48, 64, 3), dtype=np.uint8), now_ns()


@pytest.fixture
def worker():
    cfg = load_robot_config(ROBOTS / "fake.yaml")
    w = RobotWorker(FakeDriver(cfg, lin_speed_mm_s=1000.0), SafetyGate(cfg.limits, cfg.stream.period_s))
    w.start()
    w.submit(Connect())
    w.submit(Enable())
    assert wait_for(lambda: w.latest_state().mode is Mode.IDLE)
    yield w
    w.shutdown()


def test_features_match_frame_keys(worker):
    feats = build_features(6, {"front": (48, 64)})
    frame = Recorder(worker, {"front": FakeCam()}, 30, FakeWriter()).sample()
    assert set(feats) == set(frame) - {"task"}
    for key, spec in feats.items():
        assert frame[key].shape == spec["shape"], key


def test_fixed_rate_frame_count(worker):
    writer = FakeWriter()
    rec = Recorder(worker, {}, 30, writer)
    rec.start_episode("test")
    time.sleep(2.0)
    n = rec.save_episode()
    assert 59 <= n <= 62 and len(writer.episodes[0]) == n


def test_action_is_commanded_state_is_measured(worker):
    writer = FakeWriter()
    rec = Recorder(worker, {}, 30, writer)
    p = worker.latest_state().tcp
    far = p.with_axis("X", p.x + 40)  # the step clamp lets 4.5 mm through per stream period
    worker.stream(StreamTarget(far, now_ns(), "test", True))
    assert wait_for(lambda: worker.commanded.sent is not None)
    worker.submit(Gripper(False))
    assert wait_for(lambda: worker.commanded.gripper_open is False)
    frame = rec.sample()
    assert frame["observation.command_raw"][0] == pytest.approx(far.x)  # input before the gate
    assert frame["action"][0] == pytest.approx(p.x + 4.5)  # what we sent after the gate
    assert frame["action"][0] != frame["observation.command_raw"][0]
    assert frame["observation.state"][6] == pytest.approx(worker.latest_state().tcp.x, abs=5.0)  # measured
    assert frame["action"][6] == 0.0  # gripper commanded closed


def test_real_times_stored(worker):
    writer = FakeWriter()
    rec = Recorder(worker, {"front": FakeCam()}, 30, writer)
    rec.start_episode("t")
    time.sleep(0.3)
    rec.save_episode()
    t = [f["observation.t_state_ns"][0] for f in writer.episodes[0]]
    cams = [f["observation.t_cam_front_ns"][0] for f in writer.episodes[0]]
    assert all(b > a for a, b in itertools.pairwise(t)) and all(b > a for a, b in itertools.pairwise(cams))


def test_discard_leaves_no_episode(worker):
    writer = FakeWriter()
    rec = Recorder(worker, {}, 30, writer)
    rec.start_episode("bad take")
    time.sleep(0.2)
    rec.discard_episode()
    rec.close()
    assert writer.episodes == [] and writer.discarded == 1 and writer.finalized


def test_lerobot_writer_needs_lerobot(tmp_path):
    pytest.importorskip("lerobot")  # only runs once LeRobot is installed (pip install -e .[record])
    from botswain.record.lerobot_writer import LeRobotWriter

    writer = LeRobotWriter(tmp_path / "ds", "botswain/test", 30, build_features(6, {}), "fake")
    writer.finalize()
