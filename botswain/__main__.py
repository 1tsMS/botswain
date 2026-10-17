"""python -m botswain [--robot robots/dobot_cr5.yaml]

Without --robot it opens the robot used last time (robots/.last), else the fake robot.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from botswain.core.config import ConfigError, load_robot_config

ROBOTS = Path(__file__).resolve().parents[1] / "robots"
LAST = ROBOTS / ".last"


def _default_robot() -> Path:
    if LAST.exists():
        last = Path(LAST.read_text(encoding="utf-8").strip())
        if last.exists():
            return last
    return ROBOTS / "fake.yaml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="botswain", description=__doc__.splitlines()[0])
    parser.add_argument("--robot", type=Path, help="robot YAML (default: last used, else robots/fake.yaml)")
    args = parser.parse_args(argv)
    path = (args.robot or _default_robot()).resolve()
    try:
        cfg = load_robot_config(path)
    except ConfigError as e:
        print(f"Robot config {path} is not valid: {e}", file=sys.stderr)
        return 2
    LAST.write_text(str(path), encoding="utf-8")

    from botswain.app import run  # Qt loads only after the config is known to be good

    return run(cfg)


if __name__ == "__main__":
    sys.exit(main())
