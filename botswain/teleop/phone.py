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
import math
import os
import re
import secrets
import shutil
import socket
import subprocess
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

import teleop
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from botswain.core.clock import now_ns

log = logging.getLogger(__name__)

# FastAPI reads the endpoint annotations at runtime, so these imports must be module-level.
CERT_DIR = Path(__file__).resolve().parents[2] / "assets" / "certs"


# The package page's Start button does nothing at all when WebXR is missing or AR fails to start.
# This makes every such failure visible on the phone.
DIAGNOSE_JS = """<script>
(function () {
  function say(msg) { alert("Botswain: " + msg); }
  window.addEventListener("unhandledrejection", function (e) {
    say("AR could not start: " + (e.reason && (e.reason.message || e.reason)) +
        ". Check Chrome is up to date and Google Play Services for AR is installed.");
  });
  document.addEventListener("click", function (e) {
    if (!e.target || e.target.id !== "start-button") return;
    if (!window.isSecureContext) { say("not a secure page: open the https:// address"); return; }
    if (!navigator.xr) { say("this browser has no WebXR. Open the address in Google Chrome (not the built-in or an in-app browser)."); return; }
    navigator.xr.isSessionSupported("immersive-ar").then(function (ok) {
      if (!ok) say("this phone/browser cannot run WebXR AR. Install or update 'Google Play Services for AR' from the Play Store; the phone must be on Google's ARCore device list.");
    });
  }, true);
})();
</script>"""

CHECK_PAGE = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<body style="font:16px sans-serif;padding:16px"><h3>Botswain phone check</h3><pre id=o>checking...</pre><script>
var o = document.getElementById("o"), lines = [];
lines.push("secure page (https): " + window.isSecureContext);
lines.push("WebXR present: " + !!navigator.xr);
lines.push("browser: " + navigator.userAgent);
function done() { o.textContent = lines.join("
"); }
if (navigator.xr) navigator.xr.isSessionSupported("immersive-ar").then(function (ok) {
  lines.push("immersive-ar supported: " + ok + (ok ? "  -> go back to / and press Start" :
    "  -> install/update Google Play Services for AR, use Chrome")); done(); }, function (e) {
  lines.push("immersive-ar check failed: " + e); done(); });
else { lines.push("-> open this address in Google Chrome"); done(); }
</script>"""


COOKIE = "botswain_key"
WRONG_LINK = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<body style="font:16px sans-serif;padding:16px"><h3>Botswain</h3>
<p>Open the full address shown on the Teleop page in Botswain (it ends with <code>?k=...</code>).
The key changes every time the phone server starts.</p>"""


def find_openssl() -> str | None:
    found = shutil.which("openssl")
    if found:
        return found
    for base in (os.environ.get("ProgramFiles", r"C:\Program Files"), os.environ.get("ProgramFiles(x86)", "")):
        candidate = Path(base) / "Git" / "usr" / "bin" / "openssl.exe"
        if base and candidate.exists():
            return str(candidate)
    return None


def make_certificate(cert_dir: Path = CERT_DIR) -> Path:
    """A self-signed certificate for this laptop, so the phone link does not use the teleop package's public key.

    Uses openssl (comes with Git for Windows). Valid for this laptop's current IPs and localhost.
    """
    openssl = find_openssl()
    if openssl is None:
        raise RuntimeError("openssl not found (it comes with Git for Windows)")
    cert_dir.mkdir(parents=True, exist_ok=True)
    san = ",".join(["DNS:localhost", "IP:127.0.0.1", *(f"IP:{ip}" for ip in local_ips())])
    subprocess.run([openssl, "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-keyout", str(cert_dir / "key.pem"),
                    "-out", str(cert_dir / "cert.pem"), "-days", "825", "-subj", "/CN=Botswain",
                    "-addext", f"subjectAltName={san}"], check=True, capture_output=True, timeout=60)
    return cert_dir / "cert.pem"


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
    numbers = [float(v) for v in (p["x"], p["y"], p["z"], o["w"], o["x"], o["y"], o["z"], data.get("scale", 1.0))]
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError("pose contains NaN or infinity")
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
        self.token = secrets.token_urlsafe(9)  # new pairing key every time the server starts
        self.refused = 0
        self.clients = 0
        self.samples = 0
        self.bad_messages = 0
        self._rate_window = (now_ns(), 0)
        self.hz = 0.0

        frontend = Path(teleop.__file__).parent
        app = FastAPI()
        app.mount("/assets", StaticFiles(directory=frontend / "assets"), name="assets")

        page = (frontend / "index.html").read_text(encoding="utf-8").replace("</body>", DIAGNOSE_JS + "</body>", 1)

        @app.get("/")
        async def index(request: Request) -> HTMLResponse:
            # Only someone who can read the address in the app gets the page and the session cookie.
            if not secrets.compare_digest(request.query_params.get("k", ""), self.token):
                return HTMLResponse(WRONG_LINK, status_code=403)
            response = HTMLResponse(page)
            response.set_cookie(COOKIE, self.token, httponly=True, secure=True, samesite="strict")
            return response

        @app.get("/check")
        async def check() -> HTMLResponse:
            return HTMLResponse(CHECK_PAGE)

        @app.websocket("/ws")
        async def ws(websocket: WebSocket) -> None:
            if not secrets.compare_digest(websocket.cookies.get(COOKIE, ""), self.token):
                self.refused += 1
                log.warning("phone connection refused: no valid pairing key")
                await websocket.close(code=1008)
                return
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
        hosts = local_ips() or ["<laptop-ip>"]
        return [f"https://{ip}:{self.port}/?k={self.token}" for ip in hosts]

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
