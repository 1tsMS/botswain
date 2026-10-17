"""3D robot view: URDF meshes posed by forward kinematics, plus the safety box and the target.

RobotModel is the pure part (yourdfpy FK, no graphics, testable headless). RobotView draws it with
pyvista inside Qt. The scene is in metres like the URDF; robot poses in mm are divided by 1000.
"""

from __future__ import annotations

import logging
from collections.abc import Sequence
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
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
    def __init__(self, urdf: Path | None, box_mm: tuple[tuple[float, float], ...] | None = None,
                 parent: QWidget | None = None) -> None:
        super().__init__(parent)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        chips = QHBoxLayout()
        chips.setSpacing(4)
        self.box_chip = self._chip("Safety box", chips)
        self.target_chip = self._chip("Target", chips)
        chips.addStretch(1)
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
        self.box_chip.toggled.connect(lambda on: self._set_visible(self._box_actor, on))
        self.target_chip.toggled.connect(lambda on: self._set_visible(self._target_actor, on))

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
        self._target_actor = self.plotter.add_mesh(pv.Sphere(radius=0.012), color=C["sel"])
        self._target_actor.SetVisibility(False)
        self.plotter.add_axes()
        self.plotter.view_isometric()
        self.set_joints([0.0] * len(self.model.joint_names))
        self.plotter.reset_camera()
        lay.addWidget(self.plotter, 1)

    def set_joints(self, joints_deg: Sequence[float]) -> None:
        if self.model is None or self.plotter is None:
            return
        q = np.asarray(joints_deg, dtype=float)
        if self._last_joints is not None and np.allclose(q, self._last_joints, atol=1e-3):
            return
        self._last_joints = q
        for node, T in self.model.transforms(q).items():
            self._actors[node].user_matrix = T
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
