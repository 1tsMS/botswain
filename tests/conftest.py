"""Shared test setup."""

from pathlib import Path

import pytest

from botswain.core import config


@pytest.fixture(autouse=True)
def no_lab_overlays(monkeypatch, tmp_path):
    """Tests never see or touch this machine's lab edits (configs/**/*.local.yaml, e.g. a safety box taught in the
    lab): those are gitignored per-lab state, and the tests check the shared configs. A test that saves a lab edit
    for a shared config (a Settings save, a box switch) writes it to its own tmp_path instead. Copies already in
    tmp_path keep their own overlays."""
    real = config.overlay_path
    shared = config.CONFIGS.resolve()

    def overlay_path(path: Path) -> Path:
        local = real(Path(path))
        if Path(path).resolve().is_relative_to(shared):
            return tmp_path / "lab-overlays" / local.name
        return local

    (tmp_path / "lab-overlays").mkdir()
    monkeypatch.setattr(config, "overlay_path", overlay_path)
