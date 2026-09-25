"""hold: stay where the drone was when the step started."""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Mission, Setpoint, Status, StepContext, Vector3, block


@block("hold")
class Hold(Mission):
    @dataclass
    class Params:
        duration: float = 0.0  # s; 0 means hold until the step's until/timeout ends it

    def reset(self) -> None:
        self._position: Vector3 | None = None

    def step(self, ctx: StepContext) -> Setpoint:
        if self._position is None:
            self._position = ctx.vehicle.position_ned
        return Setpoint(position=self._position)

    def status(self, ctx: StepContext) -> Status:
        if self.params.duration > 0 and ctx.elapsed >= self.params.duration:
            return Status.DONE
        return Status.RUNNING
