"""Phone teleop over WebXR: an HTTPS + websocket server the phone's browser connects to.

We serve the phone page from the `teleop` package (SpesRobotics, Apache-2.0) but run our own uvicorn server:
the package's run() always uses its shared certificate and cannot be stopped. Ours uses
assets/certs/{cert,key}.pem when present (per-laptop certificate), else the package one, and stops cleanly.
Phone message (websocket /ws): {"type": "pose", "data": {position{x,y,z} m, orientation{x,y,z,w}, move,
gripper "open"/"close", scale}}.
"""

from __future__ import annotations

import json
import logging
import re
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import teleop
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from botswain.core.clock import now_ns

log = logging.getLogger(__name__)

# FastAPI reads the endpoint annotations at runtime, so these imports must be module-level.
CERT_DIR = Path(__file__).resolve().parents[2] / "assets" / "certs"


@dataclass(frozen=True)
class PhoneSample:
    t_ns: int
    position_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    hold: bool
    gripper_closed: bool
    scale: float


def parse_message(data: dict, t_ns: int) -> PhoneSample:
    """The 'data' part of a pose message. Raises KeyError/ValueError on a malformed one."""
    p, o = data["position"], data["orientation"]
    return PhoneSample(
        t_ns=t_ns,
        position_m=(float(p["x"]), float(p["y"]), float(p["z"])),
        quat_wxyz=(float(o["w"]), float(o["x"]), float(o["y"]), float(o["z"])),
        hold=bool(data.get("move", False)),
        gripper_closed=data.get("gripper") == "close",
        scale=float(data.get("scale", 1.0)),
    )


def local_ips() -> list[str]:
    """Private IPv4 addresses of this laptop, robot LAN (192.168.5.x) last. No internet needed
    (the teleop package asks 8.8.8.8, which fails on a hotspot without internet)."""
    try:
        ips = {info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}
    except OSError:
        ips = set()
    private = [ip for ip in ips if ip.startswith(("192.168.", "10.")) or re.match(r"172\.(1[6-9]|2\d|3[01])\.", ip)]
    return sorted(private, key=lambda ip: (ip.startswith("192.168.5."), ip))


class PhoneServer:
    def __init__(self, on_sample: Callable[[PhoneSample], None], port: int = 4443, host: str = "0.0.0.0") -> None:
        self.on_sample = on_sample
        self.port = port
        self.clients = 0
        self.samples = 0
        self.bad_messages = 0
        self._rate_window = (now_ns(), 0)
        self.hz = 0.0

        frontend = Path(teleop.__file__).parent
        app = FastAPI()
        app.mount("/assets", StaticFiles(directory=frontend / "assets"), name="assets")

        @app.get("/")
        async def index() -> FileResponse:
            return FileResponse(frontend / "index.html")

        @app.websocket("/ws")
        async def ws(websocket: WebSocket) -> None:
            await websocket.accept()
            self.clients += 1
            log.info("phone connected")
            try:
                while True:
                    self._handle(await websocket.receive_text())
            except WebSocketDisconnect:
                pass
            finally:
                self.clients -= 1
                log.info("phone disconnected")

        own = CERT_DIR / "cert.pem"
        cert_dir = CERT_DIR if own.exists() and (CERT_DIR / "key.pem").exists() else frontend
        self.cert_is_shared = cert_dir == frontend
        config = uvicorn.Config(app, host=host, port=port, ssl_certfile=str(cert_dir / "cert.pem"),
                                ssl_keyfile=str(cert_dir / "key.pem"), log_level="warning")
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, name="phone-server", daemon=True)

    @property
    def urls(self) -> list[str]:
        return [f"https://{ip}:{self.port}" for ip in local_ips()] or [f"https://<laptop-ip>:{self.port}"]

    def start(self, timeout_s: float = 5.0) -> None:
        self._thread.start()
        end = time.perf_counter() + timeout_s
        while not self._server.started:
            if not self._thread.is_alive() or time.perf_counter() > end:
                raise RuntimeError(f"phone server did not start on port {self.port} (port in use?)")
            time.sleep(0.02)

    def stop(self) -> None:
        self._server.should_exit = True
        if self._thread.is_alive():
            self._thread.join(3.0)

    def _handle(self, text: str) -> None:
        t = now_ns()
        try:
            msg = json.loads(text)
            if msg.get("type") != "pose":
                return
            sample = parse_message(msg["data"], t)
        except (ValueError, KeyError, TypeError):
            self.bad_messages += 1
            return
        self.samples += 1
        start, count = self._rate_window
        if t - start >= 1_000_000_000:
            self.hz = (count + 1) * 1e9 / (t - start)
            self._rate_window = (t, 0)
        else:
            self._rate_window = (start, count + 1)
        self.on_sample(sample)
