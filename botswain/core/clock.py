"""The one clock. Every sample (robot state, camera frame, phone pose, command) is stamped with now_ns()
the moment it arrives in our process, so recordings can line them up later."""

from __future__ import annotations

import time
from datetime import UTC, datetime, timedelta


def now_ns() -> int:
    """Monotonic time in ns. Never goes backwards, unlike wall time."""
    return time.perf_counter_ns()


class SessionClock:
    """Stores the wall time once at session start, so monotonic stamps can be mapped back to real dates."""

    def __init__(self) -> None:
        self.start_ns = now_ns()
        self.start_wall = datetime.now(UTC)

    def to_wall(self, t_ns: int) -> datetime:
        return self.start_wall + timedelta(microseconds=(t_ns - self.start_ns) / 1000)
