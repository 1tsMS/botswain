import numpy as np
import pytest

from botswain.core.types import Pose
from botswain.teleop.mapping import TeleopMapper, remap_about_z

ROBOT = Pose(-482, -136, 484, 180, 0, 0)
Q0 = (1.0, 0.0, 0.0, 0.0)


def test_anchor_on_hold_down():
    m = TeleopMapper()
    assert m.update((0.3, 1.2, -0.5), Q0, hold=False, phone_scale=1.0, robot_pose=ROBOT) is None
    assert m.update((0.3, 1.2, -0.5), Q0, hold=True, phone_scale=1.0, robot_pose=ROBOT) == ROBOT


def test_scale_pos():
    m = TeleopMapper(scale_pos=0.3)
    m.update((0.0, 0.0, 0.0), Q0, True, 1.0, ROBOT)
    # phone moves 0.1 m away from the user (WebXR -z = forward) -> robot +X by 0.1 * 0.3 = 30 mm
    p = m.update((0.0, 0.0, -0.1), Q0, True, 1.0, ROBOT)
    assert p.x - ROBOT.x == pytest.approx(30.0) and p.y == pytest.approx(ROBOT.y) and p.z == pytest.approx(ROBOT.z)
    up = m.update((0.0, 0.1, 0.0), Q0, True, 0.5, ROBOT)  # phone up, phone-side scale 0.5
    assert up.z - ROBOT.z == pytest.approx(15.0)


def test_remap_turns_axes():
    m = TeleopMapper(scale_pos=1.0, remap=remap_about_z(90))
    m.update((0, 0, 0), Q0, True, 1.0, ROBOT)
    p = m.update((0.0, 0.0, -0.01), Q0, True, 1.0, ROBOT)  # forward 10 mm -> robot +Y
    assert p.y - ROBOT.y == pytest.approx(10.0) and p.x == pytest.approx(ROBOT.x)


def test_rotation_off_by_default():
    m = TeleopMapper()
    m.update((0, 0, 0), Q0, True, 1.0, ROBOT)
    turned = (np.cos(np.radians(20)), 0.0, np.sin(np.radians(20)), 0.0)  # 40 deg about the phone's y
    assert m.update((0, 0, 0), turned, True, 1.0, ROBOT).rpy == ROBOT.rpy


def test_rotation_on_changes_orientation():
    m = TeleopMapper(scale_rot=1.0)
    m.update((0, 0, 0), Q0, True, 1.0, ROBOT)
    turned = (np.cos(np.radians(10)), 0.0, np.sin(np.radians(10)), 0.0)
    assert m.update((0, 0, 0), turned, True, 1.0, ROBOT).rpy != pytest.approx(ROBOT.rpy)


def test_reanchor_resets_offsets():
    m = TeleopMapper(scale_pos=1.0)
    m.update((0, 0, 0), Q0, True, 1.0, ROBOT)
    m.update((0, 0, -0.05), Q0, True, 1.0, ROBOT)
    moved = ROBOT.with_axis("X", ROBOT.x + 50)
    m.request_reanchor()
    assert m.update((0.4, 0.0, 0.0), Q0, True, 1.0, moved) == moved  # new anchor, no jump
    assert m.update((0.4, 0.0, -0.01), Q0, True, 1.0, moved).x == pytest.approx(moved.x + 10)


def test_tilt_only_drops_the_twist_about_the_vertical():
    m = TeleopMapper(scale_rot=1.0, tilt_only=True)
    m.update((0, 0, 0), Q0, True, 1.0, ROBOT)
    twist = (np.cos(np.radians(20)), 0.0, np.sin(np.radians(20)), 0.0)  # 40 deg about WebXR y (up)
    assert m.update((0, 0, 0), twist, True, 1.0, ROBOT).rpy == pytest.approx(ROBOT.rpy, abs=1e-6)
    tilt = (np.cos(np.radians(10)), np.sin(np.radians(10)), 0.0, 0.0)  # 20 deg about WebXR x (right)
    assert m.update((0, 0, 0), tilt, True, 1.0, ROBOT).rpy != pytest.approx(ROBOT.rpy, abs=1.0)


def test_thumbstick_yaw_turns_about_base_z_and_resets_on_anchor():
    start = Pose(-482, -136, 484, 180, 0, 30)
    m = TeleopMapper(tilt_only=True)  # rotation following off: the stick still turns Rz
    m.nudge_yaw(10)  # not held yet: ignored
    m.update((0, 0, 0), Q0, True, 1.0, start)
    m.nudge_yaw(15)
    p = m.update((0, 0, 0), Q0, True, 1.0, start)
    # Rz(15) on top of rpy (180, 0, 30) in fixed axes: same tool tilt, rz 45
    assert p.rx == pytest.approx(180.0) or p.rx == pytest.approx(-180.0)
    assert p.ry == pytest.approx(0.0, abs=1e-6) and p.rz == pytest.approx(45.0)
    m.request_reanchor()
    assert m.update((0, 0, 0), Q0, True, 1.0, p) == p  # the new anchor already holds the yaw: no double turn
