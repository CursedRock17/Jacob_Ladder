"""GoTo: slide a position setpoint from `start` to `end` in a straight line.

The setpoint moves at `speed`, so PX4 always has a small step to close; the
skill is done once the ramp is over and the drone has stayed within `tolerance`
of `end`, nearly stopped, for `settle` seconds. `start` is where the previous
move *commanded*, not where the drone measures, so a chain of moves does not
accumulate drift. The heading is held at `yaw`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Setpoint, Skill, SkillContext, Status, Vector3

ARRIVE_SPEED = 0.2  # m/s: slower than this counts as stopped


@dataclass(eq=False)
class GoTo(Skill):
    start: Vector3
    end: Vector3
    yaw: float  # radians clockwise from north, already wrapped
    speed: float = 0.5  # m/s
    tolerance: float = 0.15  # m
    settle: float = 0.4  # s the drone must stay arrived

    def __post_init__(self) -> None:
        if self.speed <= 0 or self.tolerance <= 0 or self.settle < 0:
            raise ValueError("speed and tolerance must be > 0 and settle >= 0")
        self._distance = math.dist(self.start, self.end)
        self.reset()

    @property
    def expected_s(self) -> float:
        """How long the ramp takes: the least a caller should wait."""
        return self._distance / self.speed

    def reset(self) -> None:
        self._since: float | None = None

    def step(self, ctx: SkillContext) -> Setpoint:
        if self._distance == 0:
            return Setpoint(position=self.end, yaw=self.yaw)
        f = min(self.speed * ctx.elapsed / self._distance, 1.0)
        s, e = self.start, self.end
        position = (
            s[0] + (e[0] - s[0]) * f,
            s[1] + (e[1] - s[1]) * f,
            s[2] + (e[2] - s[2]) * f,
        )
        return Setpoint(position=position, yaw=self.yaw)

    def status(self, ctx: SkillContext) -> Status:
        if ctx.elapsed < self.expected_s:
            return Status.RUNNING
        v = ctx.vehicle
        if (
            math.dist(v.position_ned, self.end) < self.tolerance
            and math.hypot(*v.velocity_ned) < ARRIVE_SPEED
        ):
            self._since = ctx.elapsed if self._since is None else self._since
            if ctx.elapsed - self._since >= self.settle:
                return Status.DONE
        else:
            self._since = None
        return Status.RUNNING
