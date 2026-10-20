"""Config name -> driver class, for arms (protocol:) and end effectors (driver:).

Imports are lazy so one brand's code never loads for another. Adding a protocol or an end-effector driver is one
module plus one line here; every robot or tool that speaks it is then a config file only.
"""

from __future__ import annotations

from importlib import import_module

from botswain.core.config import ConfigError, RobotConfig, ToolConfig
from botswain.drivers.base import Driver
from botswain.tools import TOOLS
from botswain.tools.base import Tool

PROTOCOLS = {
    "fake": ("botswain.drivers.fake", "FakeDriver"),
    "dobot_v3": ("botswain.drivers.dobot.driver", "DobotV3Driver"),  # controller 3.5.x (CR series)
    "dobot_v4": ("botswain.drivers.dobot.driver", "DobotV4Driver"),  # controller 4.x (CRA, Nova, E6)
}


def driver_class(cfg: RobotConfig) -> type[Driver]:
    if cfg.protocol not in PROTOCOLS:
        raise ConfigError("protocol", f"unknown protocol {cfg.protocol!r} (known: {', '.join(sorted(PROTOCOLS))})")
    module, cls = PROTOCOLS[cfg.protocol]
    return getattr(import_module(module), cls)


def make_driver(cfg: RobotConfig) -> Driver:
    return driver_class(cfg)(cfg)


def make_tool(tool: ToolConfig, robot: RobotConfig) -> Tool:
    """The end effector's driver, after checking the arm offers the port it needs."""
    if tool.driver not in TOOLS:
        raise ConfigError("driver", f"unknown end-effector driver {tool.driver!r} (known: {', '.join(sorted(TOOLS))})")
    if robot.tool_port != tool.port:
        offered = robot.tool_port or "no tool port"
        raise ConfigError("port", f"{tool.name} needs an {tool.port} tool port; {robot.name} offers {offered}")
    module, cls = TOOLS[tool.driver]
    kwargs = {"slave": tool.slave}
    if tool.force_range_pct:
        kwargs["force_range_pct"] = tool.force_range_pct
    return getattr(import_module(module), cls)(**kwargs)
