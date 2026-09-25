"""Fake blocks registered in a private Registry, so tests never depend on the shipped library."""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

import pytest

from jl_blocks.core import (
    Action,
    ActionStatus,
    Controller,
    Mission,
    Registry,
    Setpoint,
    Status,
    StepContext,
    Target,
    VehicleState,
)


def make_registry() -> Registry:
    reg = Registry()

    class Beacon(Target):
        """Visible whenever the class-level switch says so."""

        visible = True

        @dataclass
        class Params:
            id: int = 0

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return (5.0, 0.0, -1.0) if Beacon.visible else None

    class Passthrough(Controller):
        def command(
            self, vehicle: VehicleState, desired: Setpoint, dt: float
        ) -> Setpoint:
            return desired

    class Doubler(Controller):
        """Doubles x, so a test can tell which controller ran."""

        def command(
            self, vehicle: VehicleState, desired: Setpoint, dt: float
        ) -> Setpoint:
            assert desired.position is not None
            x, y, z = desired.position
            return Setpoint(position=(2 * x, y, z))

    class Goto(Mission):
        """Flies to a fixed point; DONE after `ticks` ticks."""

        @dataclass
        class Params:
            x: float = 1.0
            ticks: int = 0  # 0 = never done

        def reset(self) -> None:
            self.count = 0

        def step(self, ctx: StepContext) -> Setpoint:
            self.count += 1
            return Setpoint(position=(self.params.x, 0.0, -1.0))

        def status(self, ctx: StepContext) -> Status:
            return (
                Status.DONE
                if self.params.ticks and self.count >= self.params.ticks
                else Status.RUNNING
            )

    class Broken(Mission):
        def step(self, ctx: StepContext) -> Setpoint:
            raise RuntimeError("boom")

    class Quitter(Mission):
        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -1.0))

        def status(self, ctx: StepContext) -> Status:
            return Status.FAILED

    class Needy(Mission):
        @dataclass
        class Params:
            height: float  # required: no default

        def step(self, ctx: StepContext) -> Setpoint | None:
            return None

    class BadReset(Mission):
        """Raises from reset(), so a step entry can be made to fail."""

        def reset(self) -> None:
            raise RuntimeError("reset boom")

        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -1.0))

    class Ranged(Mission):
        """Has a Params.__post_init__ range check, like a real block would."""

        @dataclass
        class Params:
            height: float = 1.0

            def __post_init__(self) -> None:
                if self.height <= 0:
                    raise ValueError("height must be > 0")

        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -self.params.height))

    class Asker(Mission):
        """Asks the executor for `kind` once, then reports the reply."""

        @dataclass
        class Params:
            kind: str = "takeoff"

        def step(self, ctx: StepContext) -> Action | None:
            if ctx.action is ActionStatus.NONE:
                return Action(self.params.kind, 1.5)
            return None

        def status(self, ctx: StepContext) -> Status:
            if ctx.action is ActionStatus.SUCCEEDED:
                return Status.DONE
            if ctx.action is ActionStatus.FAILED:
                return Status.FAILED
            return Status.RUNNING

    class Mutator(Mission):
        """Changes its own params, to prove each engine gets its own copy."""

        @dataclass
        class Params:
            x: float = 1.0

        def step(self, ctx: StepContext) -> Setpoint:
            self.params.x += 1.0
            return Setpoint(position=(self.params.x, 0.0, -1.0))

    class BadTarget(Target):
        def reset(self) -> None:
            raise RuntimeError("target reset boom")

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return None

    class Seeker(Mission):
        """Needs a target, so the loader must insist on one."""

        needs_target = True

        def step(self, ctx: StepContext) -> Setpoint:
            return Setpoint(position=(0.0, 0.0, -1.0))

    class Grumpy(Target):
        """Listens on a topic and raises on every detection."""

        def topic(self) -> str | None:
            return "/grumpy"

        def observe(
            self, position: tuple[float, float, float], vehicle: VehicleState
        ) -> None:
            raise RuntimeError("grumpy")

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return None

    class Counter(Target):
        """Counts reset() calls per instance, so a test can check how many times
        a particular target was reset."""

        instances: ClassVar[list[Counter]] = []

        def __init__(self, params: object | None = None) -> None:
            super().__init__(params)
            self.reset_count = 0
            Counter.instances.append(self)

        def reset(self) -> None:
            self.reset_count += 1

        def estimate(self, vehicle: VehicleState) -> tuple[float, float, float] | None:
            return None

    for name, cls in [
        ("beacon", Beacon),
        ("position", Passthrough),
        ("doubler", Doubler),
        ("goto", Goto),
        ("broken", Broken),
        ("quitter", Quitter),
        ("needy", Needy),
        ("badreset", BadReset),
        ("ranged", Ranged),
        ("counter", Counter),
        ("asker", Asker),
        ("mutator", Mutator),
        ("badtarget", BadTarget),
        ("seeker", Seeker),
        ("grumpy", Grumpy),
    ]:
        reg.register(name, cls)
    Beacon.visible = True
    return reg


@pytest.fixture
def registry() -> Registry:
    return make_registry()


@pytest.fixture
def vehicle() -> VehicleState:
    return VehicleState(position_ned=(0.0, 0.0, -1.0))
