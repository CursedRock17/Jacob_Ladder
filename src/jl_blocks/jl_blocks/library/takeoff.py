"""takeoff: arm and take off with PX4's own takeoff, to `height` above the ground.

The step is done once jl_mission reports the drone is at that height. Port of
the arm -> takeoff() calls in the TakeoffLand / TakeoffHold executors.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Action, ActionStatus, Mission, Status, StepContext, block


@block("takeoff")
class Takeoff(Mission):
    @dataclass
    class Params:
        height: float = 1.5  # m above the ground

        def __post_init__(self) -> None:
            if not 0 < self.height <= 10:
                raise ValueError(
                    f"height must be more than 0 and at most 10 m (got {self.height})"
                )

    def step(self, ctx: StepContext) -> Action | None:
        if ctx.action is ActionStatus.NONE:
            return Action("takeoff", self.params.height)
        return None

    def status(self, ctx: StepContext) -> Status:
        if ctx.action is ActionStatus.SUCCEEDED:
            return Status.DONE
        if ctx.action is ActionStatus.FAILED:
            return Status.FAILED
        return Status.RUNNING
