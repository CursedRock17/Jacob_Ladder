"""Hold: stay where the drone is (or at `position`), for `duration` seconds."""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Setpoint, Skill, SkillContext, Status, Vector3


@dataclass(eq=False)
class Hold(Skill):
    duration: float = 0.0  # s; 0 means hold until whoever runs it stops it
    position: Vector3 | None = None  # NED; None holds where the skill started
    yaw: float | None = None  # radians; None holds the heading at the start

    def __post_init__(self) -> None:
        if self.duration < 0:
            raise ValueError("duration must not be negative")
        self.reset()

    def reset(self) -> None:
        self._position: Vector3 | None = self.position
        self._yaw: float | None = self.yaw

    def step(self, ctx: SkillContext) -> Setpoint:
        if self._position is None:
            self._position = ctx.vehicle.position_ned
        if self._yaw is None:
            self._yaw = ctx.vehicle.yaw
        return Setpoint(position=self._position, yaw=self._yaw)

    def status(self, ctx: SkillContext) -> Status:
        if self.duration > 0 and ctx.elapsed >= self.duration:
            return Status.DONE
        return Status.RUNNING
