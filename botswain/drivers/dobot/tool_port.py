"""The Dobot's RS485 tool port: raw frames through the controller's HTTP server (port 22000).

POST /interface/toolDataExchange {"value": [bytes]} -> {"status": true, "data": {"value": [reply bytes]}}.
Seen on the lab CR5 for writes (echo) and reads (DH register replies, CRC valid).
"""

from __future__ import annotations

import json
import urllib.error
import urllib.request

from botswain.drivers.base import DriverError


class DobotRs485Port:
    kind = "rs485"

    def __init__(self, base_url: str, timeout_s: float = 1.0) -> None:
        self.url = base_url.rstrip("/") + "/interface/toolDataExchange"
        self.timeout_s = timeout_s

    def exchange(self, frame: list[int]) -> list[int]:
        req = urllib.request.Request(self.url, data=json.dumps({"value": frame}).encode(),
                                     headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                body = resp.read().decode("utf-8", "replace")
        except (urllib.error.URLError, TimeoutError, OSError) as e:
            raise DriverError(-200, f"tool port request failed ({e})",
                              "check tool power and the RS485 settings (e.g. 115200 8N1) in DobotStudio") from e
        try:
            return [int(b) for b in json.loads(body)["data"]["value"]]
        except (ValueError, KeyError, TypeError):
            return []  # the controller answered, the tool did not: the tool driver reports it
