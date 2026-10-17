"""Logging setup: one rotating file plus stderr. The GUI log dock is fed separately by the worker."""

from __future__ import annotations

import logging
from logging.handlers import RotatingFileHandler
from pathlib import Path

LEVELS = {"debug": logging.DEBUG, "info": logging.INFO, "warn": logging.WARNING, "error": logging.ERROR}


def setup_logging(log_dir: Path, level: int = logging.INFO) -> Path:
    """Send every 'botswain.*' logger to <log_dir>/botswain.log (1 MB x 3) and stderr. Safe to call twice."""
    log_dir.mkdir(parents=True, exist_ok=True)
    path = log_dir / "botswain.log"
    root = logging.getLogger("botswain")
    root.setLevel(level)
    if not root.handlers:
        fmt = logging.Formatter("%(asctime)s %(levelname)-7s %(name)s: %(message)s")
        file_handler = RotatingFileHandler(path, maxBytes=1_000_000, backupCount=3, encoding="utf-8")
        file_handler.setFormatter(fmt)
        root.addHandler(file_handler)
        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(fmt)
        root.addHandler(stream_handler)
    return path
