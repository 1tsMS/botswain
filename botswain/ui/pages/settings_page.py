"""Settings page: which robot and end effector Botswain runs with, and this lab's settings for them.

Left: pick a robot config and an end-effector config (or add new ones), then Load. Loading builds a fresh
window in the same app (every page is made for one robot's capabilities) and closes the old one; it is only
allowed while disconnected.
Right: this lab's connection, speeds and safety box for the robot that is running. They are saved to the
config's .local.yaml overlay (the shared config file is never rewritten). The safety box numbers can be typed
here; to teach it by moving the arm to its corners, use Robot view > Box on the Jog page. It applies at once.
"""

from __future__ import annotations

import shutil
from collections.abc import Callable
from dataclasses import replace
from pathlib import Path

import yaml
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from botswain.control.worker import RobotWorker
from botswain.core.config import (
    ROBOT_DIR,
    TOOL_DIR,
    ConfigError,
    RobotConfig,
    ToolConfig,
    discover,
    load_robot_config,
    load_tool_config,
    overlay_path,
    save_overlay,
)
from botswain.core.types import Mode
from botswain.drivers.registry import PROTOCOLS
from botswain.tools import TOOLS
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.panel import Panel

AXES = ("x", "y", "z")
MIN_BOX_MM = 20.0  # a box smaller than this on any axis is a typo, not a workspace


class SettingsPage(QWidget):
    def __init__(self, cfg: RobotConfig, tool_cfg: ToolConfig | None, worker: RobotWorker,
                 log: Callable[[str, str], None], switch: Callable[[Path, Path | None], None],
                 apply_box: Callable[[tuple, bool], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.cfg = cfg
        self.tool_cfg = tool_cfg
        self.worker = worker
        self._log = log
        self._switch = switch
        self._apply_box = apply_box

        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setSpacing(8)
        left, right = QVBoxLayout(), QVBoxLayout()
        left.setSpacing(8)
        right.setSpacing(8)
        left.addWidget(self._robot_panel())
        left.addWidget(self._tool_panel())
        left.addWidget(self._apply_panel())
        left.addStretch(1)
        if cfg.connection is not None:
            right.addWidget(self._connection_panel())
        right.addWidget(self._speed_panel())
        right.addWidget(self._box_panel())
        right.addStretch(1)
        grid.addLayout(left, 0, 0)
        grid.addLayout(right, 0, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self.fill_robots()
        self.fill_tools()
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()
        self._refresh()

    # ---- building ----

    @staticmethod
    def _button(text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(lambda: slot())
        return b

    @staticmethod
    def _details() -> QLabel:
        label = QLabel("")
        label.setFont(mono_font(12))
        label.setWordWrap(True)
        label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        return label

    def _robot_panel(self) -> Panel:
        panel = Panel("Robot", "configs/robots")
        row = QHBoxLayout()
        self.robot_box = QComboBox()
        self.robot_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.robot_box.currentIndexChanged.connect(self._update_details)
        row.addWidget(self.robot_box, 1)
        row.addWidget(self._button("Load robot config…", "btn", self.load_robot_config))
        panel.body_layout.addLayout(row)
        self.robot_details = self._details()
        panel.body_layout.addWidget(self.robot_details)
        return panel

    def _tool_panel(self) -> Panel:
        panel = Panel("End effector", "configs/tools")
        row = QHBoxLayout()
        self.tool_box = QComboBox()
        self.tool_box.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.tool_box.currentIndexChanged.connect(self._update_details)
        row.addWidget(self.tool_box, 1)
        row.addWidget(self._button("Load end-effector config…", "btn", self.load_tool_config))
        panel.body_layout.addLayout(row)
        self.tool_details = self._details()
        panel.body_layout.addWidget(self.tool_details)
        return panel

    def _apply_panel(self) -> Panel:
        panel = Panel("Apply")
        self.apply_note = set_role(QLabel(""), "lab")
        self.apply_note.setWordWrap(True)
        self.restart_btn = self._button("Load", "primary", self.load_selected)
        panel.body_layout.addWidget(self.apply_note)
        panel.body_layout.addWidget(self.restart_btn, 0, Qt.AlignmentFlag.AlignLeft)
        return panel

    def _connection_panel(self) -> Panel:
        c = self.cfg.connection
        panel = Panel("Connection", f"this lab · {self.cfg.name}")
        form = QGridLayout()
        form.setHorizontalSpacing(8)
        form.addWidget(set_role(QLabel("IP address"), "lab"), 0, 0)
        self.ip_edit = QLineEdit(c.ip)
        form.addWidget(self.ip_edit, 0, 1)
        self.port_boxes: dict[str, QSpinBox] = {}
        for row, (name, port) in enumerate(c.ports, start=1):
            form.addWidget(set_role(QLabel(f"{name} port"), "lab"), row, 0)
            box = QSpinBox()
            box.setRange(1, 65535)
            box.setValue(port)
            form.addWidget(box, row, 1)
            self.port_boxes[name] = box
        form.setColumnStretch(1, 1)
        panel.body_layout.addLayout(form)
        panel.body_layout.addWidget(self._button("Save for this lab", "btn", self.save_connection), 0,
                                    Qt.AlignmentFlag.AlignLeft)
        return panel

    def _speed_panel(self) -> Panel:
        lim = self.cfg.limits
        panel = Panel("Speed", "this lab")
        form = QGridLayout()
        form.setHorizontalSpacing(8)
        self.cap_box = self._pct_box(lim.speed_cap_pct)
        self.ctrl_box = self._pct_box(lim.controller_speed_max_pct)
        form.addWidget(set_role(QLabel("Speed cap (highest % the speed control allows)"), "lab"), 0, 0)
        form.addWidget(self.cap_box, 0, 1)
        form.addWidget(set_role(QLabel("Controller speed at 100 % (Dobot SpeedFactor)"), "lab"), 1, 0)
        form.addWidget(self.ctrl_box, 1, 1)
        form.setColumnStretch(0, 1)
        panel.body_layout.addLayout(form)
        panel.body_layout.addWidget(self._button("Save for this lab", "btn", self.save_speed), 0,
                                    Qt.AlignmentFlag.AlignLeft)
        return panel

    @staticmethod
    def _pct_box(value: int) -> QSpinBox:
        box = QSpinBox()
        box.setRange(1, 100)
        box.setSuffix(" %")
        box.setValue(value)
        return box

    def _box_panel(self) -> Panel:
        panel = Panel("Safety box", "TCP limits in the base frame, mm")
        form = QGridLayout()
        form.setHorizontalSpacing(8)
        form.addWidget(set_role(QLabel("min"), "lab"), 0, 1)
        form.addWidget(set_role(QLabel("max"), "lab"), 0, 2)
        self.box_fields: dict[str, tuple[QDoubleSpinBox, QDoubleSpinBox]] = {}
        for row, (axis, (lo, hi)) in enumerate(zip(AXES, self.cfg.limits.box_mm, strict=True), start=1):
            form.addWidget(set_role(QLabel(axis.upper()), "axis"), row, 0)
            pair = (self._mm_box(lo), self._mm_box(hi))
            form.addWidget(pair[0], row, 1)
            form.addWidget(pair[1], row, 2)
            self.box_fields[axis] = pair
        panel.body_layout.addLayout(form)
        self.box_on = QCheckBox("Enforce the box (the Robot view and the VR button switch it too; the last choice "
                                "is kept for the next start)")
        self.box_on.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.box_on.setChecked(self.cfg.limits.box_enabled)
        self._saved_box_on = self.cfg.limits.box_enabled  # what the lab file says
        panel.body_layout.addWidget(self.box_on)

        panel.body_layout.addWidget(set_role(QLabel(
            "To teach it by moving the arm to its corners: Jog page, Robot view > Box."), "lab"))
        panel.body_layout.addWidget(self._button("Apply && save box", "primary", self.save_box), 0,
                                    Qt.AlignmentFlag.AlignLeft)
        return panel

    @staticmethod
    def _mm_box(value: float) -> QDoubleSpinBox:
        box = QDoubleSpinBox()
        box.setRange(-3000.0, 3000.0)
        box.setDecimals(1)
        box.setSuffix(" mm")
        box.setValue(value)
        return box

    # ---- picking configs ----

    def fill_robots(self) -> None:
        self.robot_box.blockSignals(True)
        self.robot_box.clear()
        found = discover(ROBOT_DIR, load_robot_config)
        if all(p.resolve() != self.cfg.path.resolve() for p, _, _ in found):
            found.insert(0, (self.cfg.path, self.cfg, ""))  # started with a config from elsewhere (--robot)
        for path, cfg, error in found:
            self.robot_box.addItem(cfg.name if cfg else f"{path.name} (not valid)", (path, cfg, error))
            if path.resolve() == self.cfg.path.resolve():
                self.robot_box.setCurrentIndex(self.robot_box.count() - 1)
        self.robot_box.blockSignals(False)
        self._update_details()

    def fill_tools(self, pick: Path | None = None) -> None:
        current = pick or (self.tool_cfg.path if self.tool_cfg else None)
        self.tool_box.blockSignals(True)
        self.tool_box.clear()
        self.tool_box.addItem("None", (None, None, ""))
        for path, cfg, error in discover(TOOL_DIR, load_tool_config):
            self.tool_box.addItem(cfg.name if cfg else f"{path.name} (not valid)", (path, cfg, error))
            if current is not None and path.resolve() == Path(current).resolve():
                self.tool_box.setCurrentIndex(self.tool_box.count() - 1)
        self.tool_box.blockSignals(False)
        self._update_details()

    def picked(self) -> tuple[Path | None, RobotConfig | None, Path | None, ToolConfig | None, list[str]]:
        """(robot path, robot cfg, tool path, tool cfg, problems) for the current dropdown choice."""
        r_path, r_cfg, r_err = self.robot_box.currentData() or (None, None, "nothing picked")
        t_path, t_cfg, t_err = self.tool_box.currentData() or (None, None, "")
        problems = [p for p in (r_err, t_err) if p]
        if r_cfg is not None and r_cfg.protocol not in PROTOCOLS:
            problems.append(f"no driver for protocol {r_cfg.protocol!r} (known: {', '.join(sorted(PROTOCOLS))})")
        if t_cfg is not None:
            if t_cfg.driver not in TOOLS:
                problems.append(f"no driver for end effector {t_cfg.driver!r} (known: {', '.join(sorted(TOOLS))})")
            if r_cfg is not None and r_cfg.tool_port != t_cfg.port:
                problems.append(f"{t_cfg.name} needs an {t_cfg.port} tool port; {r_cfg.name} offers "
                                f"{r_cfg.tool_port or 'none'}")
        return r_path, r_cfg, t_path, t_cfg, problems

    def _update_details(self) -> None:
        _, r, _, t, _ = self.picked()
        if r is not None:
            where = f"{r.connection.ip}" if r.connection else "no network"
            self.robot_details.setText(
                f"{r.vendor} {r.model} · protocol {r.protocol} · {where}\n"
                f"{r.dof} joints · tool port {r.tool_port or 'none'} · model {r.model_urdf.name if r.model_urdf else '–'}"
                + (f"\nlab edits in {overlay_path(r.path).name}" if overlay_path(r.path).exists() else ""))
        else:
            self.robot_details.setText(self.robot_box.currentData()[2] if self.robot_box.currentData() else "")
        if t is not None:
            force = f"{t.force_range_pct[0]}-{t.force_range_pct[1]} %" if t.force_range_pct else "–"
            mass = f"{t.mass_kg:g} kg" if t.mass_kg is not None else "[ ] not filled in"
            tip = (", ".join(f"{v:g}" for v in t.tcp_offset_mm) + " mm" if t.tcp_offset_mm is not None
                   else "[ ] not measured")
            self.tool_details.setText(f"{t.vendor} {t.model} · driver {t.driver} · port {t.port} · force {force}\n"
                                      f"mass {mass} · tip from flange {tip}")
        else:
            data = self.tool_box.currentData()
            self.tool_details.setText(data[2] if data and data[2] else "no end effector")
        self._refresh()

    def _load(self, folder: Path, loader, title: str) -> Path | None:
        """Pick a YAML anywhere, check it, copy it into the configs folder. Returns the copy, or None."""
        name, _ = QFileDialog.getOpenFileName(self, title, str(Path.home()), "Config (*.yaml *.yml)")
        if not name:
            return None
        src = Path(name)
        try:
            cfg = loader(src)
        except ConfigError as e:
            self._log("error", f"{src.name} is not a valid config: {e}")
            return None
        dest = folder / src.name
        n = 2
        while dest.exists():
            dest = folder / f"{src.stem}_{n}{src.suffix}"
            n += 1
        shutil.copyfile(src, dest)
        urdf = getattr(cfg, "model_urdf", None)
        if urdf is not None:  # a relative model path would break in the new folder: pin it to the real file
            raw = yaml.safe_load(dest.read_text(encoding="utf-8"))
            if not Path(raw["urdf"]).is_absolute():
                raw["urdf"] = str(urdf)
                dest.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
        self._log("info", f"config added: {dest}")
        return dest

    def load_robot_config(self) -> None:
        if self._load(ROBOT_DIR, load_robot_config, "Load robot config") is not None:
            self.fill_robots()

    def load_tool_config(self) -> None:
        dest = self._load(TOOL_DIR, load_tool_config, "Load end-effector config")
        if dest is not None:
            self.fill_tools(pick=dest)

    def load_selected(self) -> None:
        r_path, _, t_path, _, problems = self.picked()
        if problems or r_path is None:
            return
        if self.worker.latest_state().mode is not Mode.DISCONNECTED:
            self._log("warn", "disconnect first, then Load")
            return
        self._switch(r_path, t_path)

    # ---- this lab's settings (saved to the overlay) ----

    def _save(self, updates: dict, what: str, restart_needed: bool) -> bool:
        """Write the overlay, then check the robot config still loads; undo the write if it does not."""
        local = overlay_path(self.cfg.path)
        before = local.read_text(encoding="utf-8") if local.exists() else None
        save_overlay(self.cfg.path, updates)
        try:
            load_robot_config(self.cfg.path)
        except ConfigError as e:
            if before is None:
                local.unlink()
            else:
                local.write_text(before, encoding="utf-8")
            self._log("error", f"{what} not saved: {e}")
            return False
        self._log("info", f"{what} saved to {local.name}" + (" (press Load to use it)" if restart_needed else ""))
        return True

    def save_connection(self) -> None:
        self._save({"connection": {"ip": self.ip_edit.text().strip(),
                                   "ports": {n: b.value() for n, b in self.port_boxes.items()}}},
                   "connection", restart_needed=True)

    def save_speed(self) -> None:
        self._save({"limits": {"speed_cap_pct": self.cap_box.value(),
                               "controller_speed_max_pct": self.ctrl_box.value()}}, "speed", restart_needed=True)

    def box_mm(self) -> tuple[tuple[float, float], ...]:
        return tuple((lo.value(), hi.value()) for lo, hi in self.box_fields.values())

    def save_box(self) -> None:
        box = self.box_mm()
        for axis, (lo, hi) in zip(AXES, box, strict=True):
            if hi - lo < MIN_BOX_MM:
                self._log("error", f"safety box {axis.upper()}: max must be at least {MIN_BOX_MM:g} mm above min")
                return
        updates = {"limits": {"box_mm": {a: [lo, hi] for a, (lo, hi) in zip(AXES, box, strict=True)},
                              "box_enabled": self.box_on.isChecked()}}
        if self._save(updates, "safety box", restart_needed=False):
            self._saved_box_on = self.box_on.isChecked()
            self._apply_box(box, self.box_on.isChecked())

    def remember_box_enabled(self, on: bool) -> None:
        """The box was switched on or off elsewhere (Robot view chip, VR button): keep that for the next start,
        and show it here so a later Save does not flip it back."""
        self.box_on.blockSignals(True)
        self.box_on.setChecked(on)
        self.box_on.blockSignals(False)
        if on == self._saved_box_on:
            return
        if self._save({"limits": {"box_enabled": on}}, f"safety box {'ON' if on else 'OFF'}", restart_needed=False):
            self._saved_box_on = on

    def save_box_values(self, box: tuple, enabled: bool) -> None:
        """A box from elsewhere (taught in the Robot view): show it here, then the same validated save."""
        for axis, (lo, hi) in zip(AXES, box, strict=True):
            self.box_fields[axis][0].setValue(lo)
            self.box_fields[axis][1].setValue(hi)
        self.box_on.setChecked(enabled)
        self.save_box()

    # ---- live state ----

    def _refresh(self) -> None:
        r_path, _, t_path, _, problems = self.picked()
        connected = self.worker.latest_state().mode is not Mode.DISCONNECTED
        changed = (r_path is not None and r_path.resolve() != self.cfg.path.resolve()) or (
            (t_path.resolve() if t_path else None) != (self.tool_cfg.path.resolve() if self.tool_cfg else None))
        if problems:
            note = "Can't use this choice: " + "; ".join(problems)
        elif connected:
            note = "Disconnect first, then Load."
        elif not changed:
            note = "This is what is running now. Load reloads it (picks up saved lab edits)."
        else:
            note = "Load switches to this robot and end effector (a few seconds, no restart)."
        self.apply_note.setText(note)
        self.restart_btn.setEnabled(not problems and not connected)


def box_limits(limits, box_mm: tuple, enabled: bool):
    """The limits with a new box (for applying a taught box live)."""
    return replace(limits, box_mm=box_mm, box_enabled=enabled)
