"""Turn: rotate in place by `delta` radians (negative turns left) at `rate`.

The position is held at `position` while turning. Done once the ramp is over
and the drone's heading has stayed within `tolerance` of the new heading for
`settle` seconds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Setpoint, Skill, SkillContext, Status, Vector3


def wrap(angle: float) -> float:
    return math.remainder(angle, 2 * math.pi)


@dataclass(eq=False)
class Turn(Skill):
    position: Vector3  # NED, held while turning
    yaw: float  # radians clockwise from north: the heading when the turn starts
    delta: float  # radians; positive turns right, negative left
    rate: float = math.radians(90)  # rad/s
    tolerance: float = math.radians(5)  # rad
    settle: float = 0.4  # s

    def __post_init__(self) -> None:
        if self.rate <= 0 or self.tolerance <= 0 or self.settle < 0:
            raise ValueError("rate and tolerance must be > 0 and settle >= 0")
        self.reset()

    @property
    def final_yaw(self) -> float:
        return self.yaw + self.delta

    @property
    def expected_s(self) -> float:
        return abs(self.delta) / self.rate

    def reset(self) -> None:
        self._since: float | None = None

    def step(self, ctx: SkillContext) -> Setpoint:
        turned = min(self.rate * ctx.elapsed, abs(self.delta))
        yaw = self.yaw + math.copysign(turned, self.delta)
        return Setpoint(position=self.position, yaw=wrap(yaw))

    def status(self, ctx: SkillContext) -> Status:
        if ctx.elapsed < self.expected_s:
            return Status.RUNNING
        if abs(wrap(ctx.vehicle.yaw - self.final_yaw)) < self.tolerance:
            self._since = ctx.elapsed if self._since is None else self._since
            if ctx.elapsed - self._since >= self.settle:
                return Status.DONE
        else:
            self._since = None
        return Status.RUNNING
