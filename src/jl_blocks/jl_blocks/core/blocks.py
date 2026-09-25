"""The three kinds of block, and the registry that names them.

A block is one small class. Its parameters are a nested ``Params`` dataclass,
so the mission loader can check a mission file's params against it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, ClassVar, TypeVar

from .types import Action, ActionStatus, Setpoint, Status, Vector3, VehicleState


class Block:
    kind: ClassVar[str] = "block"
    name: ClassVar[str] = ""

    @dataclass
    class Params:
        pass

    def __init__(self, params: object | None = None) -> None:
        # Any: each block's Params dataclass is different, and YAML fills it in.
        self.params: Any = params if params is not None else self.Params()

    def reset(self) -> None:
        """Called when a step using this block starts."""


class Target(Block):
    """Answers "where is the thing?" in NED metres, or None if not seen."""

    kind = "target"
    # Seconds without an estimate before the target counts as lost.
    lost_after: ClassVar[float] = 3.0

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        raise NotImplementedError

    def topic(self) -> str | None:
        """The geometry_msgs/PoseStamped topic the runner feeds to observe(), or None."""
        return None

    def observe(self, position: Vector3, vehicle: VehicleState) -> None:
        """One detection, in the camera's own frame, and the vehicle state when it arrived."""


class Controller(Block):
    """Turns what the mission wants into the setpoint sent to PX4."""

    kind = "controller"

    def command(self, vehicle: VehicleState, desired: Setpoint, dt: float) -> Setpoint:
        raise NotImplementedError


@dataclass(frozen=True)
class StepContext:
    vehicle: VehicleState
    target: Target | None
    # This tick's target estimate, so missions don't call estimate() again.
    target_position: Vector3 | None
    controller: Controller
    elapsed: float
    dt: float
    # This step's executor request so far (see Action), and the executor's reply.
    action: ActionStatus = ActionStatus.NONE
    action_message: str = ""


class Mission(Block):
    """What the drone is trying to do in one step, and whether it's done."""

    kind = "mission"
    # True for blocks that make no sense without a target (track, precision_descend);
    # the loader then insists on one.
    needs_target: ClassVar[bool] = False

    def step(self, ctx: StepContext) -> Setpoint | Action | None:
        raise NotImplementedError

    def status(self, ctx: StepContext) -> Status:
        return Status.RUNNING


B = TypeVar("B", bound=type[Block])


class Registry:
    def __init__(self) -> None:
        self._blocks: dict[str, type[Block]] = {}

    def register(self, name: str, cls: type[Block]) -> None:
        if name in self._blocks:
            raise ValueError(f"block '{name}' is already registered")
        cls.name = name
        self._blocks[name] = cls

    def get(self, name: str) -> type[Block] | None:
        return self._blocks.get(name)

    def names(self, kind: str | None = None) -> list[str]:
        return sorted(
            n for n, c in self._blocks.items() if kind is None or c.kind == kind
        )


REGISTRY = Registry()


def block(name: str, registry: Registry = REGISTRY) -> Callable[[B], B]:
    """Class decorator: ``@block("pid")`` makes the class usable as ``pid:`` in a mission file."""

    def register(cls: B) -> B:
        registry.register(name, cls)
        return cls

    return register
