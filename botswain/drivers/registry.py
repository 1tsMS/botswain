"""Driver name in the robot YAML -> driver class. Imports are lazy so one brand's code never loads for another."""

from __future__ import annotations

from importlib import import_module

from botswain.core.config import ConfigError, RobotConfig
from botswain.drivers.base import Driver

DRIVERS = {
    "fake": ("botswain.drivers.fake", "FakeDriver"),
}


def make_driver(cfg: RobotConfig) -> Driver:
    if cfg.driver not in DRIVERS:
        raise ConfigError("driver", f"unknown driver {cfg.driver!r} (known: {', '.join(sorted(DRIVERS))})")
    module, cls = DRIVERS[cfg.driver]
    return getattr(import_module(module), cls)(cfg)
