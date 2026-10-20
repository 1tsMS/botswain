"""Driver name in the robot YAML -> driver class. Imports are lazy so one brand's code never loads for another."""

from __future__ import annotations

from importlib import import_module

from botswain.core.config import ConfigError, RobotConfig
from botswain.drivers.base import Driver

DRIVERS = {
    "fake": ("botswain.drivers.fake", "FakeDriver"),
    "dobot_v3": ("botswain.drivers.dobot.driver", "DobotV3Driver"),  # controller 3.5.x (CR series)
    "dobot_v4": ("botswain.drivers.dobot.driver", "DobotV4Driver"),  # controller 4.x (CRA, Nova, E6)
}


def make_driver(cfg: RobotConfig) -> Driver:
    if cfg.driver not in DRIVERS:
        raise ConfigError("driver", f"unknown driver {cfg.driver!r} (known: {', '.join(sorted(DRIVERS))})")
    module, cls = DRIVERS[cfg.driver]
    return getattr(import_module(module), cls)(cfg)
