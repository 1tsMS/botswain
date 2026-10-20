"""End-effector drivers. A tool config names one of these (driver: dh_modbus); see base.py for the contract."""

from __future__ import annotations

TOOLS = {
    "dh_modbus": ("botswain.tools.dh_modbus", "DhModbusTool"),  # DH-Robotics PGE / PGI / AG over RS485
}
