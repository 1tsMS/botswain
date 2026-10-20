import pytest
from helpers import ROBOTS, wait_for

from botswain.core.config import load_robot_config
from botswain.core.types import Joints, Mode, Pose
from botswain.drivers.base import DriverError
from botswain.drivers.fake import FakeDriver
from botswain.drivers.registry import make_driver


@pytest.fixture
def cfg():
    return load_robot_config(ROBOTS / "fake.yaml")


@pytest.fixture
def fake(cfg):
    d = FakeDriver(cfg, lin_speed_mm_s=1000.0, joint_speed_deg_s=1000.0)
    d.connect()
    d.set_speed(100)
    return d


def settled(d):
    return wait_for(lambda: d.read_state().mode is not Mode.MOVING, timeout=2.0)


def test_registry_builds_fake(cfg):
    assert isinstance(make_driver(cfg), FakeDriver)


def test_fake_starts_disabled(cfg):
    d = FakeDriver(cfg)
    assert d.read_state().mode is Mode.DISCONNECTED
    d.connect()
    s = d.read_state()
    assert s.mode is Mode.DISABLED and s.link_ok


def test_enable_goes_idle(fake):
    fake.enable()
    assert fake.read_state().mode is Mode.IDLE


def test_jog_step_moves_x_by_delta(fake):
    fake.enable()
    x0 = fake.read_state().tcp.x
    fake.jog_step("X", 1.0, "base")
    assert settled(fake)
    assert abs(fake.read_state().tcp.x - (x0 + 1.0)) < 1e-4


def test_joint_jog_step(fake):
    fake.enable()
    j0 = fake.read_state().joints.axis("J6")
    fake.jog_step("J6", -2.0, "base")
    assert settled(fake)
    assert abs(fake.read_state().joints.axis("J6") - (j0 - 2.0)) < 1e-4


def test_move_to_reports_moving_then_idle(fake):
    fake.enable()
    start = fake.read_state().tcp
    fake.move_to(start.with_axis("Z", start.z + 50), "linear")
    assert fake.read_state().mode is Mode.MOVING
    assert settled(fake)
    s = fake.read_state()
    assert s.mode is Mode.IDLE and abs(s.tcp.z - (start.z + 50)) < 1e-4


def test_move_to_joints(fake):
    fake.enable()
    fake.move_to(Joints((10, 0, 90, 0, -90, 0)), "joint")
    assert settled(fake)
    assert fake.read_state().joints.axis("J1") == 10.0


def test_inject_error_gives_error_mode_and_clear_recovers(fake):
    fake.enable()
    fake.inject_error(22)
    s = fake.read_state()
    assert s.mode is Mode.ERROR and 22 in s.errors
    with pytest.raises(DriverError) as e:
        fake.enable()
    assert e.value.code == -2
    fake.clear_error()
    s = fake.read_state()
    assert s.mode is Mode.DISABLED and s.errors == ()


def test_motion_refused_when_disabled(fake):
    with pytest.raises(DriverError):
        fake.jog_step("X", 1.0, "base")


def test_stop_is_safe_in_any_mode(cfg):
    d = FakeDriver(cfg, lin_speed_mm_s=10.0)
    d.stop()  # disconnected
    d.connect()
    d.stop()  # disabled
    d.enable()
    p = d.read_state().tcp
    d.move_to(p.with_axis("X", p.x + 100), "linear")
    assert d.read_state().mode is Mode.MOVING
    d.stop()  # moving
    assert d.read_state().mode is Mode.IDLE
    d.inject_error(1)
    d.stop()  # error
    d.drop_link()
    d.stop()  # link lost
    assert d.read_state().mode is Mode.ERROR


def test_hold_jog_moves_until_release(fake):
    fake.enable()
    x0 = fake.read_state().tcp.x
    fake.jog_hold("X", -1, "base")
    assert wait_for(lambda: fake.read_state().tcp.x < x0 - 1.0)
    fake.jog_release()
    s = fake.read_state()
    assert s.mode is Mode.IDLE
    x1 = s.tcp.x
    assert wait_for(lambda: False, timeout=0.05) is False
    assert fake.read_state().tcp.x == x1


def test_drop_link_reports_link_lost_and_refuses_commands(fake):
    fake.enable()
    fake.drop_link()
    s = fake.read_state()
    assert not s.link_ok and s.feedback_hz == 0.0
    with pytest.raises(DriverError) as e:
        fake.jog_step("X", 1.0, "base")
    assert e.value.code == -101


def test_stream_target_follows(fake):
    fake.enable()
    p = fake.read_state().tcp
    fake.stream_target(p.with_axis("Y", p.y + 5))
    assert fake.read_state().mode is Mode.STREAMING
    assert wait_for(lambda: abs(fake.read_state().tcp.y - (p.y + 5)) < 1e-4)


def test_rejects_wrong_joint_count(fake):
    fake.enable()
    with pytest.raises(DriverError):
        fake.move_to(Joints((0, 0, 0)), "joint")
    assert isinstance(fake.read_state().tcp, Pose)
