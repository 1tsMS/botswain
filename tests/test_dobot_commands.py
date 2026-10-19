import pytest

from botswain.core.types import Joints, Pose
from botswain.drivers.dobot import commands as c

P = Pose(1, 2, 3, 4, 5, 6)


def test_servo_p_string():
    assert c.servo_p(P, 0.03, 50, 500) == (
        "ServoP(1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,t=0.030,aheadtime=50.0,gain=500.0)")


def test_move_jog_cartesian():
    assert c.move_jog("X+", 1) == "MoveJog(X+,coordtype=1)"
    assert c.move_jog("J3-", 0) == "MoveJog(J3-,coordtype=0)"


def test_move_jog_stop():
    assert c.move_jog(None) == "MoveJog()"


def test_rel_movl_user():
    assert c.rel_movl_user([1, 0, 0, 0, 0, 0], v=20) == (
        "RelMovLUser(1.000000,0.000000,0.000000,0.000000,0.000000,0.000000,user=0,v=20)")


def test_rel_moves_without_speed_use_global_factor():
    assert c.rel_movl_tool([0, 0, -1, 0, 0, 0]).endswith(",tool=0)")
    assert c.rel_joint_movj([0, 0, 0, 0, 0, 2]) == (
        "RelJointMovJ(0.000000,0.000000,0.000000,0.000000,0.000000,2.000000)")


def test_absolute_moves():
    assert c.mov_l_pose(P) == "MovL(pose={1.000000,2.000000,3.000000,4.000000,5.000000,6.000000})"
    assert c.mov_j_pose(P, v=10).startswith("MovJ(pose={") and c.mov_j_pose(P, v=10).endswith("},v=10)")
    assert c.mov_j_joints(Joints((0, 0, 90, 0, -90, 0))) == (
        "MovJ(joint={0.000000,0.000000,90.000000,0.000000,-90.000000,0.000000})")


def test_simple_commands():
    assert c.enable_robot() == "EnableRobot()"
    assert c.speed_factor(10) == "SpeedFactor(10)"
    assert c.set_tool_485(115200) == 'SetTool485(115200,"N",1)'
    assert c.set_tool_power(True) == "SetToolPower(1)"


@pytest.mark.parametrize("call", [
    lambda: c.speed_factor(0), lambda: c.speed_factor(101), lambda: c.servo_p(P, 0.001, 50, 500),
    lambda: c.servo_p(P, 0.03, 10, 500), lambda: c.servo_p(P, 0.03, 50, 5000), lambda: c.move_jog("X+", 3),
    lambda: c.rel_movl_user([1, 2, 3]), lambda: c.mov_l_pose(P, v=0),
])
def test_out_of_range_never_becomes_a_command(call):
    with pytest.raises(ValueError):
        call()


def test_negative_zero_is_never_sent():
    # Lab CR5: JointMovJ(-0.000000,...) -> -30001. Saved point P1 had J1 = -0.0.
    from botswain.drivers.dobot import commands as c

    text = c.joint_mov_j_legacy([-0.0, -1e-9, 91.69, -6.8e-06, -90.6, 0.82])
    assert text == "JointMovJ(0.000000,0.000000,91.690000,-0.000007,-90.600000,0.820000)"
    assert "-0.000000" not in c.servo_p_legacy(Pose(-0.0, -0.0000004, 0, 0, 0, -0.0))
