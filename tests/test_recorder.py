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
    assert wait_for(lambda: worker.commanded.gripper == 0.0)
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


def test_input_source_is_recorded(worker):
    from dataclasses import replace

    from botswain.record.recorder import SOURCE_CODES, Recorder, build_features

    names = build_features(6, {})["observation.input_source"]["names"]
    assert names == ["source (0 none, 1 gui, 2 phone, 3 gamepad, 4 vr, 5 drag)"]
    worker.commanded = replace(worker.commanded, source="vr")  # the last target came from VR...
    frame = Recorder(worker, {}, 30, writer=None).sample()
    assert frame["observation.input_source"][0] == SOURCE_CODES[""] == 0  # ...but nothing drives the arm now


# ---- the three bugs found in MS's first lab datasets (2026-10-25) ----

class SlowWriter(FakeWriter):
    """Stalls once, like a PNG write or a GC pause: the recorder must not squeeze time."""

    def add(self, frame):
        super().add(frame)
        if len(self.frames) == 10:
            time.sleep(0.25)


def test_a_late_recorder_fills_the_slots_it_missed_instead_of_squeezing_time(worker):
    writer = SlowWriter()
    rec = Recorder(worker, {}, 30, writer)
    t0 = time.perf_counter()
    rec.start_episode("test")
    time.sleep(2.0)
    n = rec.save_episode()
    elapsed = time.perf_counter() - t0
    assert rec.late_frames >= 6  # the 0.25 s stall is about 7 slots
    assert abs(n - elapsed * 30) <= 3  # frame count still matches real time: frame k = start + k / 30
    t = np.array([f["observation.t_state_ns"][0] for f in writer.episodes[0]]) / 1e9
    assert (t[-1] - t[0]) == pytest.approx((n - 1) / 30, abs=0.2)  # real span vs what the dataset says


def test_frames_no_input_drives_never_carry_a_stale_teleop_target(worker):
    from botswain.core.types import Joints, MoveTo

    writer = FakeWriter()
    rec = Recorder(worker, {}, 30, writer)
    p = worker.latest_state().tcp
    for i in range(20):  # a phone streams a target, then lets go
        worker.stream(StreamTarget(p.with_axis("X", p.x + 3 * i), now_ns(), "phone", True))
        time.sleep(0.02)
    worker.stream(StreamTarget(worker.latest_state().tcp, now_ns(), "phone", False))
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE)
    rec.start_episode("test")
    time.sleep(0.3)  # idle
    worker.submit(MoveTo(Joints((20, 10, 80, 0, -60, 0)), "joint"))  # a joint move, like Home
    assert wait_for(lambda: worker.latest_state().mode is Mode.MOVING)
    assert wait_for(lambda: worker.latest_state().mode is Mode.IDLE, timeout=8)
    time.sleep(0.2)
    rec.save_episode()
    for f in writer.episodes[0]:
        state_xyz, act_xyz = f["observation.state"][6:9], f["action"][:3]
        source = int(f["observation.input_source"][0])
        if source == 0:  # idle: stay where you are
            assert np.allclose(act_xyz, state_xyz, atol=0.5)
        elif source == 1:  # the joint move: the controller's own target, never the phone's old one
            assert np.allclose(act_xyz, f["observation.controller_target"][:3], atol=0.5)
    assert {int(f["observation.input_source"][0]) for f in writer.episodes[0]} >= {0, 1}


class FlipWorker:
    """Rx flipping between +179.9 and -179.9 (tool pointing straight down), as the CR5 reports it."""

    def __init__(self, rx_values):
        from botswain.control.worker import Commanded

        self.rx = iter(rx_values)
        self.commanded = Commanded()

    def latest_state(self):
        from botswain.core.types import Joints, Pose, RobotState

        rx = next(self.rx, -179.9)
        pose = Pose(-400, 100, 300, rx, 0.0, 90.0)
        return RobotState(now_ns(), Mode.IDLE, Joints((0,) * 6), pose, Joints((0,) * 6), pose, 10, link_ok=True)

    def active_source(self, state):
        return ""


def test_angles_are_unwrapped_and_every_episode_starts_on_the_same_branch():
    writer = FakeWriter()
    rec = Recorder(FlipWorker(itertools.cycle([179.9, -179.9, 179.8, -179.7])), {}, 30, writer)
    rec.start_episode("one")
    time.sleep(0.4)
    rec.save_episode()
    rx = np.array([f["observation.state"][9] for f in writer.episodes[0]])
    assert np.abs(np.diff(rx)).max() < 1.0  # no 360 deg jumps
    act = np.array([f["action"][3] for f in writer.episodes[0]])
    assert np.abs(act - rx).max() < 1.0  # the action sits on the state's branch
    rec.worker = FlipWorker(itertools.cycle([-179.9, 179.9]))  # next episode starts on the other side
    rec.start_episode("two")
    time.sleep(0.2)
    rec.save_episode()
    assert abs(writer.episodes[1][0]["observation.state"][9] - rx[0]) < 1.0  # same branch as episode one
