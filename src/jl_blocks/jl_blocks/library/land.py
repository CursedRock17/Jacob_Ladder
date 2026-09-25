"""land: land with PX4's own land mode. Done once the drone is on the ground."""

from __future__ import annotations

from ..core import Action, ActionStatus, Mission, Status, StepContext, block


@block("land")
class Land(Mission):
    def step(self, ctx: StepContext) -> Action | None:
        if ctx.action is ActionStatus.NONE:
            return Action("land")
        return None

    def status(self, ctx: StepContext) -> Status:
        if ctx.action is ActionStatus.SUCCEEDED:
            return Status.DONE
        if ctx.action is ActionStatus.FAILED:
            return Status.FAILED
        return Status.RUNNING
