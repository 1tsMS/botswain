"""Phone teleop over WebXR: the `teleop` package's page (SpesRobotics, Apache-2.0) on our own paired server.

The package's own run() always uses its shared certificate and cannot be stopped, so we serve its page from
teleop.server.PairedServer (pairing key, cookie, per-laptop certificate) instead.
Phone message (websocket /ws): {"type": "pose", "data": {position{x,y,z} m, orientation{x,y,z,w}, move,
gripper "open"/"close", scale}}.
"""

from __future__ import annotations

import json
import logging
import math
from collections.abc import Callable
from dataclasses import dataclass

from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from botswain.core.clock import now_ns
from botswain.teleop.server import CERT_DIR, PACKAGE_DIR, PairedServer, local_ips, make_certificate

__all__ = ["CERT_DIR", "PhoneSample", "PhoneServer", "local_ips", "make_certificate", "parse_message"]

log = logging.getLogger(__name__)

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
function done() { o.textContent = lines.join("\n"); }
if (navigator.xr) navigator.xr.isSessionSupported("immersive-ar").then(function (ok) {
  lines.push("immersive-ar supported: " + ok + (ok ? "  -> go back to / and press Start" :
    "  -> install/update Google Play Services for AR, use Chrome")); done(); }, function (e) {
  lines.push("immersive-ar check failed: " + e); done(); });
else { lines.push("-> open this address in Google Chrome"); done(); }
</script>"""


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


class PhoneServer(PairedServer):
    def __init__(self, on_sample: Callable[[PhoneSample], None], port: int = 4443, host: str = "0.0.0.0") -> None:
        self.on_sample = on_sample
        self.samples = 0
        self.bad_messages = 0
        page = (PACKAGE_DIR / "index.html").read_text(encoding="utf-8").replace("</body>", DIAGNOSE_JS + "</body>", 1)
        super().__init__(page, self._handle, port, host, path="/", tab="Teleop")
        self.app.mount("/assets", StaticFiles(directory=PACKAGE_DIR / "assets"), name="assets")

        @self.app.get("/check")
        async def check() -> HTMLResponse:
            return HTMLResponse(CHECK_PAGE)

    def _handle(self, text: str) -> None:
        try:
            msg = json.loads(text)
            if msg.get("type") != "pose":
                return
            sample = parse_message(msg["data"], now_ns())
        except (ValueError, KeyError, TypeError):
            self.bad_messages += 1
            return
        self.samples += 1
        self.on_sample(sample)
