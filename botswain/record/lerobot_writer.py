"""Writes episodes as a LeRobotDataset (lerobot 0.6.x). Imported lazily: lerobot pulls in torch.

Install only when recording is needed: pip install -e .[record]
"""

from __future__ import annotations

from pathlib import Path

IMAGE_THREADS_PER_CAMERA = 4  # LeRobot's own advice; a noisy 640x480 PNG takes ~22 ms to write


class LeRobotWriter:
    def __init__(self, root: Path, repo_id: str, fps: int, features: dict, robot_type: str,
                 vcodec: str | None = None) -> None:
        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as e:
            raise RuntimeError("recording needs LeRobot: pip install -e .[record]") from e
        kwargs = {"vcodec": vcodec} if vcodec else {}
        cameras = sum(1 for f in features.values() if f.get("dtype") == "video")
        # Camera frames go to PNG on background threads; without them add_frame wrote the PNGs on the recorder's
        # own thread (2 cameras ~45 ms per frame against a 33 ms budget) and the recorder could not hold 30 fps.
        # The video is still encoded from those PNGs at Save, between episodes. (Not streaming_encoding: its
        # encoder drops frames when it falls behind, and video and data would no longer line up.)
        self.dataset = LeRobotDataset.create(repo_id=repo_id, fps=fps, features=features, root=root,
                                             robot_type=robot_type, use_videos=True,
                                             image_writer_threads=IMAGE_THREADS_PER_CAMERA * cameras, **kwargs)

    def add(self, frame: dict) -> None:
        self.dataset.add_frame(frame)

    def save(self) -> None:
        self.dataset.save_episode()

    def discard(self) -> None:
        self.dataset.clear_episode_buffer()

    def finalize(self) -> None:
        self.dataset.finalize()
