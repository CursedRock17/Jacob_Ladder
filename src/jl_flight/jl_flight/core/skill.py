"""The three interfaces a flight is built from.

A **Skill** is one thing the drone can do over time ("go there", "turn", "follow
that tag"). The drone ticks it ~50 times a second: it returns the setpoint to
fly now and says whether it is finished. A **Target** answers "where is the
thing?". A **Controller** turns the setpoint a skill wants into what is sent
to PX4. All three are pure Python, so each can be tested with plain pytest.

A skill is a dataclass whose fields are its parameters:

    @dataclass(eq=False)
    class Wiggle(Skill):
        amplitude: float = 0.5  # m

        def step(self, ctx): ...
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from .types import Setpoint, Status, Vector3, VehicleState


class Target:
    """Answers "where is the thing?" in NED metres, or None if it is not seen."""

    # Seconds without an estimate before the target counts as lost.
    lost_after: ClassVar[float] = 3.0

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        raise NotImplementedError


class Controller:
    """Turns what a skill wants into the setpoint sent to PX4."""

    def reset(self) -> None:
        """Called when a skill using this controller starts."""

    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        raise NotImplementedError


@dataclass(frozen=True)
class SkillContext:
    vehicle: VehicleState
    target: Target | None
    # This tick's target estimate, so a skill does not call estimate() again.
    target_position: Vector3 | None
    elapsed: float  # s since the skill started
    dt: float  # s since the previous tick


class Skill:
    # True for skills that make no sense without a target (track, descend).
    needs_target: ClassVar[bool] = False

    def reset(self) -> None:
        """Called when the skill starts."""

    def step(self, ctx: SkillContext) -> Setpoint:
        raise NotImplementedError

    def status(self, ctx: SkillContext) -> Status:
        return Status.RUNNING
