"""3D robot view: URDF meshes posed by forward kinematics, plus the safety box and the target.

RobotModel is the pure part (yourdfpy FK, no graphics, testable headless). RobotView draws it with
pyvista inside Qt. The scene is in metres like the URDF; robot poses in mm are divided by 1000.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget

from botswain.core.types import Pose
from botswain.ui.theme import C, set_role

log = logging.getLogger(__name__)


class RobotModel:
    def __init__(self, urdf: Path) -> None:
        import yourdfpy  # imported here so the rest of the app starts without it

        self.urdf = yourdfpy.URDF.load(str(urdf), load_meshes=True, build_scene_graph=True)
        self.joint_names = list(self.urdf.actuated_joint_names)
        self.nodes = list(self.urdf.scene.graph.nodes_geometry)

    def meshes(self) -> dict:
        graph, geometry = self.urdf.scene.graph, self.urdf.scene.geometry
        return {node: geometry[graph.get(node)[1]] for node in self.nodes}

    def transforms(self, joints_deg: Sequence[float]) -> dict[str, np.ndarray]:
        """World transform (4x4) of every mesh node for these joint angles."""
        self.urdf.update_cfg(np.radians(np.asarray(joints_deg, dtype=float)))
        graph = self.urdf.scene.graph
        return {node: np.array(graph.get(node)[0]) for node in self.nodes}

    def link_transform(self, link: str, joints_deg: Sequence[float]) -> np.ndarray:
        self.urdf.update_cfg(np.radians(np.asarray(joints_deg, dtype=float)))
        return np.array(self.urdf.get_transform(link))


class RobotView(QWidget):
    box_toggled = Signal(bool)

    def __init__(self, urdf: Path | None, box_mm: tuple[tuple[float, float], ...] | None = None,
                 gripper: bool = False, box_on: bool = False, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._show_gripper = gripper
        self._gripper_actors: list = []
        self._gripper_open = 1.0
        self._flange = np.eye(4)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        chips = QHBoxLayout()
        chips.setSpacing(4)
        self.box_chip = self._chip("Safety box", chips)
        self.target_chip = self._chip("Target", chips)
        self.box_chip.setChecked(box_on)
        self.box_chip.setToolTip("Show and enforce the workspace box from the robot YAML")
        chips.addStretch(1)
        self.reset_btn = set_role(QPushButton("Reset view"), "chip")
        self.reset_btn.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.reset_btn.clicked.connect(self.reset_view)
        chips.addWidget(self.reset_btn)
        # Hooked up before the 3D scene: the box toggle must work even without a 3D view.
        self.box_chip.toggled.connect(lambda on: self._set_visible(self._box_actor, on))
        self.box_chip.toggled.connect(self.box_toggled.emit)
        self.target_chip.toggled.connect(lambda on: self._set_visible(self._target_actor, on))
        lay.addLayout(chips)

        self.model: RobotModel | None = None
        self.plotter = None
        self._actors: dict = {}
        self._box_actor = None
        self._target_actor = None
        self._last_joints: np.ndarray | None = None

        if urdf is None or not urdf.exists():
            lay.addWidget(self._message(f"No 3D model ({urdf.name if urdf else 'no URDF in the robot YAML'})"), 1)
            return
        try:
            self.model = RobotModel(urdf)
            self._build_scene(lay, box_mm)
        except Exception as e:
            log.exception("3D view failed")
            lay.addWidget(self._message(f"3D view failed: {e}"), 1)

    def _chip(self, text: str, row: QHBoxLayout) -> QPushButton:
        b = set_role(QPushButton(text), "chip")
        b.setCheckable(True)
        b.setChecked(True)
        b.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        row.addWidget(b)
        return b

    def _message(self, text: str) -> QLabel:
        label = set_role(QLabel(text), "placeholder")
        label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        return label

    def _build_scene(self, lay: QVBoxLayout, box_mm: tuple[tuple[float, float], ...] | None) -> None:
        import pyvista as pv
        from pyvistaqt import QtInteractor

        self.plotter = QtInteractor(self)
        self.plotter.setFocusPolicy(Qt.FocusPolicy.NoFocus)  # keyboard stays with the app (Space = STOP)
        self.plotter.clear_events_for_key("q")  # pyvista closes the view on 'q'
        self.plotter.set_background(C["bg"], top="#fbfbfa")
        self.plotter.add_mesh(pv.Plane(i_size=1.4, j_size=1.4, i_resolution=14, j_resolution=14),
                              style="wireframe", color=C["keyline"], line_width=1)
        for node, mesh in self.model.meshes().items():
            color = C["ink"] if node.startswith("base") else "#8a8d94"
            self._actors[node] = self.plotter.add_mesh(pv.wrap(mesh), color=color, smooth_shading=True)
        if box_mm is not None:
            bounds = [v / 1000.0 for axis in box_mm for v in axis]
            self._box_actor = self.plotter.add_mesh(pv.Box(bounds=bounds), style="wireframe", color=C["dim"],
                                                    line_width=1.5)
            self._box_actor.SetVisibility(self.box_chip.isChecked())
        self._target_actor = self.plotter.add_mesh(pv.Sphere(radius=0.012), color=C["sel"])
        self._target_actor.SetVisibility(False)
        if self._show_gripper:
            # Simple stand-in (the URDF has no gripper): a palm and two fingers on the flange.
            palm = pv.Box(bounds=(-0.03, 0.03, -0.02, 0.02, 0.0, 0.035))
            finger = pv.Box(bounds=(-0.005, 0.005, -0.012, 0.012, 0.035, 0.085))
            self._gripper_actors = [self.plotter.add_mesh(m, color=C["ink"]) for m in (palm, finger, finger.copy())]
        self.plotter.add_axes()
        self.plotter.enable_terrain_style(mouse_wheel_zooms=True)  # orbit about the vertical axis, never roll
        self.set_joints([0.0] * len(self.model.joint_names))
        self.reset_view()
        lay.addWidget(self.plotter, 1)

    def reset_view(self) -> None:
        if self.plotter is None:
            return
        self.plotter.camera_position = [(1.7, -1.7, 1.3), (0.0, 0.0, 0.35), (0.0, 0.0, 1.0)]
        self.plotter.render()

    def set_gripper(self, opening: float | None) -> None:
        opening = 1.0 if opening is None else float(opening)
        if opening != self._gripper_open:
            self._gripper_open = opening
            self._place_gripper()
            if self.plotter is not None:
                self.plotter.render()

    def _place_gripper(self) -> None:
        if not self._gripper_actors:
            return
        palm, left, right = self._gripper_actors
        palm.user_matrix = self._flange
        gap = 0.006 + 0.022 * self._gripper_open
        for actor, side in ((left, -1.0), (right, 1.0)):
            offset = np.eye(4)
            offset[0, 3] = side * gap
            actor.user_matrix = self._flange @ offset

    def set_joints(self, joints_deg: Sequence[float]) -> None:
        if self.model is None or self.plotter is None:
            return
        q = np.asarray(joints_deg, dtype=float)
        if self._last_joints is not None and np.allclose(q, self._last_joints, atol=1e-3):
            return
        self._last_joints = q
        for node, T in self.model.transforms(q).items():
            self._actors[node].user_matrix = T
        self._flange = np.array(self.model.urdf.get_transform("Link6"))
        self._place_gripper()
        self.plotter.render()

    def set_target(self, pose: Pose | None) -> None:
        if self._target_actor is None:
            return
        if pose is None:
            self._target_actor.SetVisibility(False)
            return
        self._target_actor.SetPosition(pose.x / 1000.0, pose.y / 1000.0, pose.z / 1000.0)
        self._target_actor.SetVisibility(self.target_chip.isChecked())

    def _set_visible(self, actor, on: bool) -> None:
        if actor is not None and self.plotter is not None:
            actor.SetVisibility(on)
            self.plotter.render()

    def close_view(self) -> None:
        if self.plotter is not None:
            self.plotter.close()
            self.plotter = None
