"""HTTPS + websocket server that a headset or phone browser pairs with. Shared by the phone and VR pages.

Pairing (phone): a short key in the URL (`?k=KEY`, new every start, shown in the app) gets the page and an
httponly session cookie; the websocket only opens with that cookie. After MAX_KEY_FAILURES wrong keys the server
accepts no key at all until it is restarted. require_key=False (VR) serves the page to anyone who can reach the
port: typing a key in a headset is painful, so the VR server relies on the network (MS's own phone hotspot). TLS uses assets/certs/{cert,key}.pem when present (per-laptop
certificate, see make_certificate), else the `teleop` package's shared one. Our own uvicorn server, so it
stops cleanly.
"""

from __future__ import annotations

import asyncio
import logging
import os
import re
import secrets
import shutil
import socket
import subprocess
import threading
import time
from collections.abc import Callable
from pathlib import Path

import teleop
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse

from botswain.core.clock import now_ns

log = logging.getLogger(__name__)

# FastAPI reads the endpoint annotations at runtime, so these imports must be module-level.
CERT_DIR = Path(__file__).resolve().parents[2] / "assets" / "certs"
PACKAGE_DIR = Path(teleop.__file__).parent

COOKIE = "botswain_key"
# Pairing key: short enough to type on a phone or in a headset, no lookalikes (0/O, 1/I/L). 31^6 = 887 million
# keys, and the server stops accepting keys after MAX_KEY_FAILURES wrong ones, so guessing it is hopeless.
KEY_ALPHABET = "ABCDEFGHJKMNPQRSTUVWXYZ23456789"
KEY_LENGTH = 6
MAX_KEY_FAILURES = 10
WRONG_LINK = """<!doctype html><meta name=viewport content="width=device-width,initial-scale=1">
<body style="font:18px sans-serif;padding:16px"><h3>Botswain</h3>
<p>{message}</p>
<form method=get action="{path}"><input name=k autocomplete=off autocapitalize=characters maxlength=6
 placeholder="KEY" style="font:28px monospace;width:7em;letter-spacing:.2em;text-transform:uppercase">
<button style="font:20px sans-serif;margin-left:8px">Open</button></form>
<p style="color:#666">The key is on the {tab} page in Botswain (or scan its QR code). It changes every time the
server starts.</p>"""
LOCKED = ("Too many wrong keys: this server no longer accepts any key. Stop and start it in Botswain to get a "
          "new one.")


def new_key() -> str:
    return "".join(secrets.choice(KEY_ALPHABET) for _ in range(KEY_LENGTH))


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
    """A self-signed certificate for this laptop, so the pairing link does not use the teleop package's public key.

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


def local_ips() -> list[str]:
    """Private IPv4 addresses of this laptop, robot LAN (192.168.5.x) last. No internet needed
    (the teleop package asks 8.8.8.8, which fails on a hotspot without internet)."""
    try:
        ips = {info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)}
    except OSError:
        ips = set()
    private = [ip for ip in ips if ip.startswith(("192.168.", "10.")) or re.match(r"172\.(1[6-9]|2\d|3[01])\.", ip)]
    return sorted(private, key=lambda ip: (ip.startswith("192.168.5."), ip))


class PairedServer:
    """Serves `page` at `path` to whoever has the key, and hands every websocket text message to on_text().

    on_text runs on the server's thread. on_disconnect runs when a client's websocket closes (a VR page that
    vanishes must stop the arm at once, not only after the stale timeout).
    one_client: a new connection replaces the old one (a headset that reconnects after sleep must not wait for
    its dead socket to time out, and two clients must never drive one session).
    outgoing: called outgoing_hz times a second per client; a str it returns is sent to that client.
    """

    def __init__(self, page: str, on_text: Callable[[str], None], port: int, host: str = "0.0.0.0",
                 path: str = "/", tab: str = "Teleop", on_disconnect: Callable[[], None] | None = None,
                 require_key: bool = True, one_client: bool = False,
                 outgoing: Callable[[], str | None] | None = None, outgoing_hz: float = 30.0) -> None:
        self.on_text = on_text
        self.on_disconnect = on_disconnect
        self.port = port
        self.path = path
        self.require_key = require_key
        self.one_client = one_client
        self._active: WebSocket | None = None  # one_client: the connection that counts
        self.token = new_key()  # new pairing key every time the server starts
        self.refused = 0
        self.key_failures = 0
        self.clients = 0
        self.messages = 0
        self._rate_window = (now_ns(), 0)
        self.hz = 0.0

        app = FastAPI()
        self.app = app

        @app.get(path)
        async def index(request: Request) -> HTMLResponse:
            if not self.require_key:
                return HTMLResponse(page)
            # Only someone who can read the key in the app gets the page and the session cookie.
            key = request.query_params.get("k", "").strip().upper()
            if self.key_failures >= MAX_KEY_FAILURES:
                return HTMLResponse(WRONG_LINK.format(message=LOCKED, path=path, tab=tab), status_code=403)
            if not secrets.compare_digest(key.encode(), self.token.encode()):
                if key:
                    self.key_failures += 1
                    log.warning("%s page: wrong pairing key (%d of %d)", tab, self.key_failures, MAX_KEY_FAILURES)
                message = "Wrong key, try again." if key else f"Type the key shown on the {tab} page in Botswain."
                return HTMLResponse(WRONG_LINK.format(message=message, path=path, tab=tab), status_code=403)
            response = HTMLResponse(page)
            response.set_cookie(COOKIE, self.token, httponly=True, secure=True, samesite="strict")
            return response

        @app.websocket("/ws")
        async def ws(websocket: WebSocket) -> None:
            if self.require_key and not secrets.compare_digest(websocket.cookies.get(COOKIE, ""), self.token):
                self.refused += 1
                log.warning("%s connection refused: no valid pairing key", tab)
                await websocket.close(code=1008)
                return
            await websocket.accept()
            if self.one_client and self._active is not None:
                log.info("%s: a new connection replaces the old one", tab)
                old, self._active = self._active, None
                await _close_quietly(old)
            self._active = websocket
            self.clients += 1
            log.info("%s client connected", tab)
            sender = asyncio.create_task(_send_loop(websocket, outgoing, outgoing_hz)) if outgoing else None
            try:
                while True:
                    text = await websocket.receive_text()
                    if self.one_client and websocket is not self._active:
                        break  # replaced: its last messages must not reach the session
                    self._count()
                    self.on_text(text)
            except (WebSocketDisconnect, RuntimeError):  # RuntimeError: receiving on a socket we closed
                pass
            finally:
                if sender is not None:
                    sender.cancel()
                if self._active is websocket:
                    self._active = None
                self.clients -= 1
                log.info("%s client disconnected", tab)
                if self.on_disconnect is not None:
                    self.on_disconnect()

        own = CERT_DIR / "cert.pem"
        cert_dir = CERT_DIR if own.exists() and (CERT_DIR / "key.pem").exists() else PACKAGE_DIR
        self.cert_is_shared = cert_dir == PACKAGE_DIR
        config = uvicorn.Config(app, host=host, port=port, ssl_certfile=str(cert_dir / "cert.pem"),
                                ssl_keyfile=str(cert_dir / "key.pem"), log_level="warning")
        self._server = uvicorn.Server(config)
        self._thread = threading.Thread(target=self._server.run, name=f"{tab.lower()}-server", daemon=True)

    @property
    def urls(self) -> list[str]:
        hosts = local_ips() or ["<laptop-ip>"]
        query = f"?k={self.token}" if self.require_key else ""
        return [f"https://{ip}:{self.port}{self.path}{query}" for ip in hosts]

    def start(self, timeout_s: float = 5.0) -> None:
        self._thread.start()
        end = time.perf_counter() + timeout_s
        while not self._server.started:
            if not self._thread.is_alive() or time.perf_counter() > end:
                raise RuntimeError(f"server did not start on port {self.port} (port in use?)")
            time.sleep(0.02)

    def stop(self) -> None:
        self._server.should_exit = True
        if self._thread.is_alive():
            self._thread.join(3.0)

    def _count(self) -> None:
        t = now_ns()
        start, count = self._rate_window
        if t - start >= 1_000_000_000:
            self.hz = count * 1e9 / (t - start)
            self._rate_window = (t, 0)
        else:
            self._rate_window = (start, count + 1)
        self.messages += 1


async def _send_loop(websocket: WebSocket, outgoing: Callable[[], str | None], hz: float) -> None:
    period = 1.0 / hz
    try:
        while True:
            text = outgoing()
            if text is not None:
                await websocket.send_text(text)
            await asyncio.sleep(period)
    except (WebSocketDisconnect, RuntimeError, OSError):
        pass  # the receive side notices the closed socket and cleans up
    except Exception:
        log.exception("status sender crashed")


async def _close_quietly(websocket: WebSocket) -> None:
    try:
        await websocket.close(code=4000)  # 4000: replaced by a newer connection (the page shows it)
    except (RuntimeError, OSError):
        pass
