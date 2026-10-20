"""python -m botswain [--robot configs/robots/dobot_cr5.yaml] [--tool configs/tools/dh_pge_50_26.yaml]

Without arguments it opens the robot and end effector picked last on the Settings page, else the fake robot.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from botswain.core.config import ROBOT_DIR, ConfigError, load_robot_config, load_tool_config, read_selection


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="botswain", description=__doc__.splitlines()[0])
    parser.add_argument("--robot", type=Path, help="robot config (default: picked last in Settings, else fake)")
    parser.add_argument("--tool", type=Path, help="end-effector config (default: picked last in Settings)")
    args = parser.parse_args(argv)
    picked_robot, picked_tool = read_selection()
    robot_path = (args.robot or picked_robot or ROBOT_DIR / "fake.yaml").resolve()
    tool_path = args.tool if args.tool else (picked_tool if not args.robot else None)
    try:
        cfg = load_robot_config(robot_path)
        tool_cfg = load_tool_config(tool_path) if tool_path else None
    except ConfigError as e:
        _startup_error(f"Config not valid: {e}")
        return 2

    from botswain.app import run  # Qt loads only after the configs are known to be good

    return run(cfg, tool_cfg)


def _startup_error(text: str) -> None:
    """Started with pythonw (Botswain.bat) there is no console: also write it to the log file."""
    if sys.stderr is not None:
        print(text, file=sys.stderr)
    log = Path(__file__).resolve().parents[1] / "logs" / "botswain.log"
    log.parent.mkdir(exist_ok=True)
    with log.open("a", encoding="utf-8") as f:
        f.write(f"STARTUP ERROR {text}\n")


if __name__ == "__main__":
    sys.exit(main())
