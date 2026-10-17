from datetime import timedelta

import pytest

from botswain.core.clock import SessionClock, now_ns
from botswain.core.types import JogHold, JogStep, Joints, MoveTo, Pose, joint_index


def test_pose_roundtrip():
    values = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    p = Pose(*values)
    assert p.as_list() == values
    assert p.xyz == (1.0, 2.0, 3.0)
    assert p.rpy == (4.0, 5.0, 6.0)


def test_pose_axis_by_gui_name():
    p = Pose(1, 2, 3, 4, 5, 6)
    assert p.axis("Rz") == 6
    assert p.with_axis("Y", 9).y == 9
    with pytest.raises(ValueError):
        p.axis("J1")


def test_joints_store_float_tuple():
    j = Joints([0, 1, 2, 3, 4, 5])
    assert j.values == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
    assert j.with_axis("J3", 90).axis("J3") == 90.0
    assert joint_index("J6") == 5


def test_commands_validate_arguments():
    with pytest.raises(ValueError):
        MoveTo(Pose(0, 0, 0, 0, 0, 0), "curve")
    with pytest.raises(ValueError):
        MoveTo(Joints((0,) * 6), "linear")
    with pytest.raises(ValueError):
        JogHold("X", 2)
    with pytest.raises(ValueError):
        JogStep("X", 1.0, "world")


def test_clock_maps_to_wall_time():
    clock = SessionClock()
    t = now_ns()
    assert t >= clock.start_ns
    assert clock.to_wall(clock.start_ns + 1_500_000_000) - clock.start_wall == timedelta(seconds=1.5)
