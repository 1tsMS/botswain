"""Exact Dobot V4 command strings (protocol doc V4.6.6). Pure functions, no sockets.

Numbers use 6 decimals like the official SDK. Ranges are checked here so a bad value never reaches the arm.
Moves leave out v= on purpose: they then run at the global SpeedFactor, which the speed control and the
safety gate's speed cap own.
"""

from __future__ import annotations

from collections.abc import Sequence

from botswain.core.types import Joints, Pose


def _f(v: float) -> str:
    return f"{float(v):.6f}"


def _six(values: Sequence[float], what: str) -> str:
    if len(values) != 6:
        raise ValueError(f"{what} needs 6 values, got {len(values)}")
    return ",".join(_f(v) for v in values)


def _opt_v(v: int | None) -> str:
    if v is None:
        return ""
    if not 1 <= v <= 100:
        raise ValueError(f"v must be 1..100, got {v}")
    return f",v={int(v)}"


def enable_robot(load: float | None = None) -> str:
    return "EnableRobot()" if load is None else f"EnableRobot({_f(load)})"


def disable_robot() -> str:
    return "DisableRobot()"


def clear_error() -> str:
    return "ClearError()"


def stop() -> str:
    return "Stop()"


def robot_mode() -> str:
    return "RobotMode()"


def get_error_id() -> str:
    return "GetErrorID()"


def get_pose() -> str:
    return "GetPose()"


def get_angle() -> str:
    return "GetAngle()"


def request_control() -> str:
    return "RequestControl()"


def speed_factor(pct: int) -> str:
    if not 1 <= pct <= 100:
        raise ValueError(f"speed factor must be 1..100, got {pct}")
    return f"SpeedFactor({int(pct)})"


def mov_j_joints(j: Joints | Sequence[float], v: int | None = None) -> str:
    values = j.values if isinstance(j, Joints) else j
    return f"MovJ(joint={{{_six(values, 'MovJ joint')}}}{_opt_v(v)})"


def mov_j_pose(p: Pose, v: int | None = None) -> str:
    return f"MovJ(pose={{{_six(p.as_list(), 'MovJ pose')}}}{_opt_v(v)})"


def mov_l_pose(p: Pose, v: int | None = None) -> str:
    return f"MovL(pose={{{_six(p.as_list(), 'MovL pose')}}}{_opt_v(v)})"


def rel_movl_user(d6: Sequence[float], v: int | None = None, user: int = 0) -> str:
    return f"RelMovLUser({_six(d6, 'RelMovLUser')},user={int(user)}{_opt_v(v)})"


def rel_movl_tool(d6: Sequence[float], v: int | None = None, tool: int = 0) -> str:
    return f"RelMovLTool({_six(d6, 'RelMovLTool')},tool={int(tool)}{_opt_v(v)})"


def rel_joint_movj(d6: Sequence[float], v: int | None = None) -> str:
    return f"RelJointMovJ({_six(d6, 'RelJointMovJ')}{_opt_v(v)})"


def move_jog(axis: str | None, coordtype: int | None = None) -> str:
    """MoveJog(X+,coordtype=1) starts a continuous jog; MoveJog() stops it.

    coordtype: 0 joint (J1+..), 1 user frame, 2 tool frame. A mismatch gives ErrorID -6 on the controller.
    """
    if axis is None:
        return "MoveJog()"
    if coordtype is None:
        return f"MoveJog({axis})"
    if coordtype not in (0, 1, 2):
        raise ValueError(f"coordtype must be 0, 1 or 2, got {coordtype}")
    return f"MoveJog({axis},coordtype={coordtype})"


def servo_p(p: Pose, t: float, aheadtime: float, gain: float) -> str:
    if not 0.004 <= t <= 3600.0:
        raise ValueError(f"ServoP t must be 0.004..3600 s, got {t}")
    if not 20.0 <= aheadtime <= 100.0:
        raise ValueError(f"ServoP aheadtime must be 20..100, got {aheadtime}")
    if not 200.0 <= gain <= 1000.0:
        raise ValueError(f"ServoP gain must be 200..1000, got {gain}")
    return f"ServoP({_six(p.as_list(), 'ServoP')},t={t:.3f},aheadtime={aheadtime:.1f},gain={gain:.1f})"


def set_tool_power(on: bool) -> str:
    return f"SetToolPower({1 if on else 0})"


def set_tool_485(baud: int, parity: str = "N", stop: int = 1) -> str:
    return f'SetTool485({int(baud)},"{parity}",{int(stop)})'


# ---- older controller firmware ----
# Some V4 controllers (seen on the lab CR5, 2026-10-19) refuse the keyword forms above with -30001/-30002
# (named parameter not understood) but accept the older positional forms below. The driver tries the
# keyword form first and falls back to these only on that syntax refusal, so nothing moves twice.


def joint_mov_j_legacy(j: Joints | Sequence[float]) -> str:
    values = j.values if isinstance(j, Joints) else j
    return f"JointMovJ({_six(values, 'JointMovJ')})"


def mov_j_pose_legacy(p: Pose) -> str:
    return f"MovJ({_six(p.as_list(), 'MovJ pose')})"


def mov_l_pose_legacy(p: Pose) -> str:
    return f"MovL({_six(p.as_list(), 'MovL pose')})"


def move_jog_legacy(axis: str, coordtype: int) -> str:
    if coordtype not in (0, 1, 2):
        raise ValueError(f"coordtype must be 0, 1 or 2, got {coordtype}")
    return f"MoveJog({axis},CoordType={coordtype},User=0,Tool=0)"
