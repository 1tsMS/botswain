"""Writes episodes as a LeRobotDataset (lerobot 0.6.x). Imported lazily: lerobot pulls in torch.

Install only when recording is needed: pip install -e .[record]
"""

from __future__ import annotations

from pathlib import Path


class LeRobotWriter:
    def __init__(self, root: Path, repo_id: str, fps: int, features: dict, robot_type: str,
                 vcodec: str | None = None) -> None:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as e:
            raise RuntimeError("recording needs LeRobot: pip install -e .[record]") from e
        kwargs = {"vcodec": vcodec} if vcodec else {}
        self.dataset = LeRobotDataset.create(repo_id=repo_id, fps=fps, features=features, root=root,
                                             robot_type=robot_type, use_videos=True, **kwargs)

    def add(self, frame: dict) -> None:
        self.dataset.add_frame(frame)

    def save(self) -> None:
        self.dataset.save_episode()

    def discard(self) -> None:
        self.dataset.clear_episode_buffer()

    def finalize(self) -> None:
        self.dataset.finalize()
