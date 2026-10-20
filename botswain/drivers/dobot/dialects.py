"""The two Dobot TCP/IP command sets, side by side. Picked by the robot config (protocol: dobot_v3 / dobot_v4).

Dobot ships two firmware lines with different command syntax AND different robot-mode numbers:
- V3: controller 3.5.x (CR series, e.g. the lab CR5). Protocol doc: Dobot TCP/IP V3 (Trossen mirror).
- V4: controller 4.x (CRA, Nova, E6). Protocol doc: TCP/IP remote control interface guide V4.6.6.
Nothing here is guessed: each form is from that version's doc, and the V3 forms were accepted by the lab CR5
(2026-10-19). Reading V3 mode codes with the V4 table made Botswain take "jogging" (11) for "collision" and
cut every hold-jog after one frame, which is why the table travels with the command set.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from botswain.core.types import Joints, Mode, Pose
from botswain.drivers.dobot import commands as cmd


@dataclass(frozen=True)
class Dialect:
    name: str
    label: str  # for the Doctor and the GUI
    modes: dict[int, Mode]
    mode_text: dict[int, str]
    joint_move: Callable[[Joints], str]
    pose_move: Callable[[Pose, bool], str]  # (pose, linear)
    servo: Callable[[Pose, float, float, float], str]  # (pose, t, aheadtime, gain)
    jog: Callable[[str, str], str | None]  # ("X+", frame) -> command, or None if this frame is not supported
    stop: str  # clears the motion queue and any ServoP stream
    request_control: bool  # RequestControl() exists (take TCP control without DobotStudio)


def _jog_v3(axis: str, frame: str) -> str | None:
    """V3 MoveJog: joints bare, Cartesian with CoordType 0 = user frame (0 = base), 1 = joint, 2 = tool.
    The lab CR5 refused CoordType=1 for X+ (-40002, joint frame) and accepted CoordType=0. Tool frame is in
    the doc but not tested on the lab arm yet, so it stays off until it is."""
    if axis.startswith("J"):
        return cmd.move_jog(axis)
    if frame == "base":
        return f"MoveJog({axis},CoordType=0,User=0,Tool=0)"
    return None


def _jog_v4(axis: str, frame: str) -> str | None:
    if axis.startswith("J"):
        return cmd.move_jog(axis, 0)
    return cmd.move_jog(axis, {"base": 1, "tool": 2}[frame])


V3 = Dialect(
    name="dobot_v3", label="Dobot V3 (controller 3.5.x)",
    modes={4: Mode.DISABLED, 5: Mode.IDLE, 6: Mode.DRAG, 7: Mode.MOVING, 8: Mode.DRAG, 9: Mode.ERROR,
           10: Mode.PAUSED, 11: Mode.JOGGING},  # 1 init, 2 brake open, 3 reserved -> UNKNOWN
    mode_text={1: "init", 2: "brake open", 4: "disabled", 5: "enabled, idle", 6: "drag", 7: "running",
               8: "drag (recording)", 9: "alarm", 10: "paused", 11: "jogging"},
    joint_move=cmd.joint_mov_j_v3,
    pose_move=lambda p, linear: cmd.mov_l_pose_v3(p) if linear else cmd.mov_j_pose_v3(p),
    servo=lambda p, t, ahead, gain: cmd.servo_p_v3(p),  # V3 ServoP takes the pose only
    jog=_jog_v3,
    stop=cmd.reset_robot(),
    request_control=False,
)

V4 = Dialect(
    name="dobot_v4", label="Dobot V4 (controller 4.x)",
    modes={3: Mode.POWERED_OFF, 4: Mode.DISABLED, 5: Mode.IDLE, 6: Mode.DRAG, 7: Mode.MOVING, 8: Mode.JOGGING,
           9: Mode.ERROR, 10: Mode.PAUSED, 11: Mode.COLLISION},
    mode_text={1: "init", 2: "brake open", 3: "powered off", 4: "disabled", 5: "enabled, idle", 6: "drag",
               7: "running", 8: "jogging", 9: "alarm", 10: "paused", 11: "collision"},
    joint_move=cmd.mov_j_joints,
    pose_move=lambda p, linear: cmd.mov_l_pose(p) if linear else cmd.mov_j_pose(p),
    servo=cmd.servo_p,
    jog=_jog_v4,
    stop=cmd.stop(),
    request_control=True,
)

DIALECTS = {d.name: d for d in (V3, V4)}
