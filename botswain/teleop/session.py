"""Glue: phone samples -> mapping -> worker.stream(). Runs on the phone server's thread."""

from __future__ import annotations

from botswain.control.worker import RobotWorker
from botswain.core.types import Gripper, StreamTarget
from botswain.teleop.mapping import TeleopMapper
from botswain.teleop.phone import PhoneSample


class TeleopSession:
    def __init__(self, worker: RobotWorker, mapper: TeleopMapper) -> None:
        self.worker = worker
        self.mapper = mapper
        self.last_sample: PhoneSample | None = None
        self._gripper_closed: bool | None = None
        worker.teleport_handlers["phone"] = mapper.request_reanchor

    def on_sample(self, sample: PhoneSample) -> None:
        self.last_sample = sample
        robot = self.worker.latest_state().tcp
        target = self.mapper.update(sample.position_m, sample.quat_wxyz, sample.hold, sample.scale, robot)
        # hold=False still goes to the worker: that is how a release stops the stream.
        self.worker.stream(StreamTarget(target or robot, sample.t_ns, "phone", sample.hold and target is not None))
        if self._gripper_closed is not None and sample.gripper_closed != self._gripper_closed:
            self.worker.submit(Gripper(open=not sample.gripper_closed))
        self._gripper_closed = sample.gripper_closed
