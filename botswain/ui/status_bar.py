"""Status bar: link, feedback rate, latency, frame, safety box + clamp count, dataset path."""

from __future__ import annotations

from PySide6.QtWidgets import QLabel, QStatusBar, QWidget

from botswain.core.config import RobotConfig
from botswain.core.types import RobotState
from botswain.ui.theme import C


def _num(text: str) -> str:
    return f'<span style="color:{C["chrome_text"]}; font-family:\'IBM Plex Mono\'">{text}</span>'


class StatusBar(QStatusBar):
    def __init__(self, cfg: RobotConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setSizeGripEnabled(False)
        self.link = QLabel()
        self.hz = QLabel()
        self.latency = QLabel()
        self.frame = QLabel()
        self.box = QLabel()
        self.dataset = QLabel(f"Dataset {_num('–')}")
        for w in (self.link, self.hz, self.latency, self.frame, self.box):
            self.addWidget(w)
        self.addPermanentWidget(self.dataset)
        self.set_box(cfg.limits.box_mm)
        self.set_state(RobotState.disconnected(cfg.dof), {}, "base", None)

    def set_box(self, box_mm: tuple[tuple[float, float], ...]) -> None:
        spans = [hi - lo for lo, hi in box_mm]
        self._box_text = "×".join(f"{s:g}" for s in spans) + " mm"

    def set_dataset(self, path: str | None) -> None:
        self.dataset.setText(f"Dataset {_num(path or '–')}")

    def set_state(self, state: RobotState, gate_counts: dict[str, int], frame: str,
                  latency_ms: float | None) -> None:
        color = C["ok_text"] if state.link_ok else C["chrome_dim"]
        self.link.setText(f'<span style="color:{color}">● Link</span>')
        self.hz.setText(f"Feedback {_num(f'{state.feedback_hz:.0f} Hz')}")
        self.latency.setText(f"Latency {_num('–' if latency_ms is None else f'{latency_ms:.0f} ms')}")
        self.frame.setText(f"Frame {_num(frame)}")
        clamps = sum(gate_counts.values())
        self.box.setText(f"Safety box {_num(self._box_text)} · gate {_num(str(clamps))}")
