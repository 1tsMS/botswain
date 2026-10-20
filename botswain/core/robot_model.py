"""The robot's URDF meshes posed by forward kinematics (yourdfpy). Pure: no graphics, no threads, testable headless.

Used by the 3D view on the Jog page and by the VR server (the headset's overlay). Not thread-safe: update_cfg
changes the model in place, so each user keeps its own instance. Units are the URDF's: metres.
"""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import numpy as np


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
