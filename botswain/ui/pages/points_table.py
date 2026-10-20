"""Points page: the saved points as a table you can rename, edit (within limits), reorder and run in order.

Columns: Name, Move (joint/linear), Wait, X..Rz, J1..Jn. Drag a row by its number to reorder.
Editing never moves the arm; Go to and Run all do, through the worker and the safety gate.
Edits stay within EDIT_LIMIT of what was taught and inside the joint limits; the editor will not accept more.
A joint edit recomputes the pose with the robot model (forward kinematics), a pose edit recomputes the joints
(inverse kinematics from the saved joints); a pose the model cannot reach is refused.
"""

from __future__ import annotations

import threading
from collections.abc import Callable

import numpy as np
from PySide6.QtCore import QModelIndex, Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QStyledItemDelegate,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from botswain.control.points import EDIT_LIMIT_DEG, EDIT_LIMIT_MM, MOVE_KINDS, Point, PointStore
from botswain.control.program import ProgramError, ProgramRunner
from botswain.core.config import RobotConfig
from botswain.core.kinematics import Kinematics, matrix_to_pose, pose_to_matrix
from botswain.core.types import AXES_CART, Command, Joints, MoveTo, Pose
from botswain.ui.theme import mono_font, set_role
from botswain.ui.widgets.panel import Panel

NAME, MOVE, WAIT = 0, 1, 2
POSE0 = 3  # X..Rz in columns 3..8, then J1..Jn


class _Editors(QStyledItemDelegate):
    """Each cell's editor knows its limits: numbers within the edit window, Move from a list."""

    def __init__(self, page: PointsTable) -> None:
        super().__init__(page)
        self.page = page

    def createEditor(self, parent, option, index: QModelIndex):
        col = index.column()
        if col == MOVE:
            box = QComboBox(parent)
            box.addItems(MOVE_KINDS)
            return box
        if col == NAME:
            return super().createEditor(parent, option, index)
        lo, hi, decimals = self.page.edit_range(index.row(), col)
        box = QDoubleSpinBox(parent)
        box.setRange(lo, hi)
        box.setDecimals(decimals)
        box.setSingleStep(0.5 if col == WAIT else 1.0)
        return box

    def setEditorData(self, editor, index: QModelIndex) -> None:
        if isinstance(editor, QComboBox):
            editor.setCurrentText(index.data())
        elif isinstance(editor, QDoubleSpinBox):
            editor.setValue(float(index.data()))
        else:
            super().setEditorData(editor, index)

    def setModelData(self, editor, model, index: QModelIndex) -> None:
        if isinstance(editor, QComboBox):
            self.page.edit(index.row(), index.column(), editor.currentText())
        elif isinstance(editor, QDoubleSpinBox):
            self.page.edit(index.row(), index.column(), editor.value())
        else:
            self.page.edit(index.row(), index.column(), editor.text())


class PointsTable(QWidget):
    changed = Signal()
    _run_done = Signal(str, str)  # level, message from the run thread
    _now_at = Signal(object)  # point name or None, from the run thread

    def __init__(self, store: PointStore, cfg: RobotConfig, runner: ProgramRunner,
                 submit: Callable[[Command], None], log: Callable[[str, str], None],
                 save: Callable[[], None], parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.store = store
        self.cfg = cfg
        self.runner = runner
        self._submit = submit
        self._log = log
        self._kin: Kinematics | None = None
        self._running_at: str | None = None
        lay = QVBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        panel = Panel("Points", f"edits within ±{EDIT_LIMIT_MM:g} mm / ±{EDIT_LIMIT_DEG:g}° of what was taught")
        lay.addWidget(panel)

        joints = [f"J{i + 1}" for i in range(cfg.dof)]
        self.columns = ["Name", "Move", "Wait s", *AXES_CART, *joints]
        self.table = QTableWidget(0, len(self.columns))
        self.table.setObjectName("pointsTable")
        self.table.setHorizontalHeaderLabels(self.columns)
        self.table.setFont(mono_font(12))
        self.table.setItemDelegate(_Editors(self))
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.DoubleClicked
                                   | QAbstractItemView.EditTrigger.EditKeyPressed)
        header = self.table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(NAME, QHeaderView.ResizeMode.ResizeToContents)
        rows = self.table.verticalHeader()
        rows.setSectionsMovable(True)  # drag a row by its number to reorder
        rows.sectionMoved.connect(self._row_moved)
        self.table.itemSelectionChanged.connect(self._update_buttons)
        panel.body_layout.addWidget(self.table, 1)
        self.empty = set_role(QLabel("No saved points yet. Connect, enable, move the arm, then Save current pose "
                                     "(or Save point in the toolbar)."), "lab")
        self.empty.setWordWrap(True)
        panel.body_layout.addWidget(self.empty)

        row = QHBoxLayout()
        self.save_btn = self._button("Save current pose", "btn", save)
        self.go_btn = self._button("Go to", "primary", self.go_to)
        self.delete_btn = self._button("Delete", "btn", self.delete)
        self.run_btn = self._button("Run all in order", "primary", self.run_all)
        self.stop_btn = self._button("Stop run", "btn", self.runner.abort)
        for b in (self.save_btn, self.go_btn, self.delete_btn, self.run_btn, self.stop_btn):
            row.addWidget(b)
        row.addStretch(1)
        panel.body_layout.addLayout(row)
        self.status = set_role(QLabel("Double-click a cell to edit. Drag a row number to reorder. Editing never "
                                      "moves the arm: Go to and Run all do."), "lab")
        self.status.setWordWrap(True)
        panel.body_layout.addWidget(self.status)
        self._run_done.connect(self._finished)
        self._now_at.connect(self._show_running)
        self.refresh()

    @staticmethod
    def _button(text: str, role: str, slot) -> QPushButton:
        b = set_role(QPushButton(text), role)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        b.clicked.connect(lambda: slot())
        return b

    # ---- table <-> store ----

    def refresh(self) -> None:
        selected = self.selected_name()
        self.table.blockSignals(True)
        self.table.verticalHeader().blockSignals(True)
        points = self.store.all()
        self.table.setRowCount(len(points))
        for r, p in enumerate(points):
            values = [p.name, p.move, f"{p.wait_s:g}", *(_fmt(v, 1) for v in p.pose.as_list()),
                      *(_fmt(v, 2) for v in p.joints.values)]
            for c, text in enumerate(values):
                item = QTableWidgetItem(text)
                if c != NAME:
                    item.setTextAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                self.table.setItem(r, c, item)
            self.table.setVerticalHeaderItem(r, QTableWidgetItem(str(r + 1)))
        self._reset_row_order()
        self.table.verticalHeader().blockSignals(False)
        self.table.blockSignals(False)
        if selected is not None:
            self.select(selected)
        self.empty.setVisible(not points)
        self._update_buttons()

    def _reset_row_order(self) -> None:
        """After a drag Qt keeps a visual order on top of the rows; the store is the order, so undo Qt's."""
        header = self.table.verticalHeader()
        for visual in range(header.count()):
            logical = header.logicalIndex(visual)
            if logical != visual:
                header.moveSection(header.visualIndex(visual), visual)

    def _row_moved(self, logical: int, _old_visual: int, new_visual: int) -> None:
        name = self.table.item(logical, NAME).text()
        self.store.move_to_index(name, new_visual)
        self._log("info", f"point {name} moved to position {new_visual + 1}")
        self.refresh()
        self.select(name)
        self.changed.emit()

    def select(self, name: str) -> None:
        for r in range(self.table.rowCount()):
            if self.table.item(r, NAME).text() == name:
                self.table.selectRow(r)
                return

    def selected_name(self) -> str | None:
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        if not rows:
            return None
        return self.table.item(rows[0].row(), NAME).text()

    def _point_at(self, row: int) -> Point:
        return self.store.get(self.table.item(row, NAME).text())

    # ---- editing ----

    def edit_range(self, row: int, col: int) -> tuple[float, float, int]:
        """(min, max, decimals) a cell may take: the taught value ± the edit limit, inside the joint limits."""
        if col == WAIT:
            return 0.0, 600.0, 1
        p = self._point_at(row)
        if col < POSE0 + 6:
            i = col - POSE0
            taught = p.origin_pose.as_list()[i]
            limit = EDIT_LIMIT_MM if i < 3 else EDIT_LIMIT_DEG
            return taught - limit, taught + limit, 1
        j = col - POSE0 - 6
        taught = p.origin_joints.values[j]
        lo, hi = self.cfg.limits.joints_deg[j]
        return max(lo, taught - EDIT_LIMIT_DEG), min(hi, taught + EDIT_LIMIT_DEG), 2

    def edit(self, row: int, col: int, value) -> None:
        p = self._point_at(row)
        try:
            if col >= WAIT:  # the editor already stops it; checked again here so no caller can skip it
                lo, hi = self.edit_range(row, col)[:2]
                if not lo - 1e-6 <= float(value) <= hi + 1e-6:
                    raise ValueError(f"{self.columns[col]} {float(value):g} is outside {lo:g}..{hi:g}")
            if col == NAME:
                renamed = self.store.rename(p.name, str(value))
                self._log("info", f"renamed point {p.name} -> {renamed.name}")
            elif col == MOVE:
                self.store.update(p.name, move=str(value))
            elif col == WAIT:
                self.store.update(p.name, wait_s=float(value))
            elif col < POSE0 + 6:
                self._edit_pose(p, col - POSE0, float(value))
            else:
                self._edit_joint(p, col - POSE0 - 6, float(value))
        except (ValueError, ProgramError) as e:
            self._log("error", f"point {p.name}: {e}")
        self.refresh()
        self.changed.emit()

    def _kinematics(self) -> Kinematics:
        if self._kin is None:
            if self.cfg.model_urdf is None:
                raise ValueError("this robot has no model file (URDF): pose and joints can't be kept in step")
            self._kin = Kinematics(self.cfg.model_urdf)
        return self._kin

    def _edit_joint(self, p: Point, j: int, value: float) -> None:
        q = list(p.joints.values)
        old, q[j] = q[j], value
        pose = matrix_to_pose(self._kinematics().fk(q))
        self.store.update(p.name, joints=Joints(tuple(q)), pose=pose)
        self._log("info", f"point {p.name}: J{j + 1} {old:.2f} -> {value:.2f} (pose recomputed with the model)")

    def _edit_pose(self, p: Point, i: int, value: float) -> None:
        values = p.pose.as_list()
        old, values[i] = values[i], value
        pose = Pose(*values)
        q = self._kinematics().ik(pose_to_matrix(pose), p.joints.values)
        if q is None:
            raise ValueError(f"{AXES_CART[i]} {value:.1f}: the robot model cannot reach that pose")
        q = np.asarray(q, dtype=float)
        for j, (lo, hi) in enumerate(self.cfg.limits.joints_deg):
            if not lo <= q[j] <= hi:
                raise ValueError(f"{AXES_CART[i]} {value:.1f} needs J{j + 1} {q[j]:.1f}, outside its limit")
        self.store.update(p.name, pose=pose, joints=Joints(tuple(float(v) for v in q)))
        self._log("info", f"point {p.name}: {AXES_CART[i]} {old:.1f} -> {value:.1f} (joints recomputed)")

    # ---- moving ----

    def go_to(self) -> None:
        name = self.selected_name()
        if name is None:
            return
        p = self.store.get(name)
        self._submit(MoveTo(p.pose, "linear") if p.move == "linear" else MoveTo(p.joints, "joint"))
        self._log("info", f"go to point {name} ({p.move})")

    def delete(self) -> None:
        name = self.selected_name()
        if name:
            self.store.remove(name)
            self._log("info", f"deleted point {name}")
            self.refresh()
            self.changed.emit()

    def run_all(self) -> None:
        points = self.store.all()
        if not points or self.runner.busy:
            return
        self._log("info", f"running {len(points)} points in order: {', '.join(p.name for p in points)}")

        def work() -> None:
            try:
                self.runner.run_points(points, on_point=self._now_at.emit)
                self._run_done.emit("info", "all points done")
            except ProgramError as e:
                self._run_done.emit("warn", f"run stopped: {e}")

        threading.Thread(target=work, name="points-run", daemon=True).start()
        self._update_buttons()

    def _show_running(self, name: str | None) -> None:
        self._running_at = name
        if name is not None:
            self.select(name)
            self.status.setText(f"running: at {name}. STOP (Space) or Stop run ends it.")
        self._update_buttons()

    def _finished(self, level: str, text: str) -> None:
        self._log(level, text)
        self.status.setText(text)
        self._update_buttons()

    def _update_buttons(self) -> None:
        has = self.selected_name() is not None
        busy = self.runner.busy
        self.go_btn.setEnabled(has and not busy)
        self.delete_btn.setEnabled(has and not busy)
        self.run_btn.setEnabled(bool(self.store.all()) and not busy)
        self.stop_btn.setEnabled(busy)
        self.table.setEnabled(not busy)  # no edits or reordering while the points are being run


def _fmt(v: float, decimals: int) -> str:
    """-0.0000004 shows as 0.00, not -0.00."""
    return f"{round(v, decimals) + 0.0:.{decimals}f}"
