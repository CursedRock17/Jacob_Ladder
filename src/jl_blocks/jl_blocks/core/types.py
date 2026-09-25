"""Plain data passed between the step engine and blocks. No ROS types here."""

from __future__ import annotations

import enum
from dataclasses import dataclass

# North, east, down, in metres (PX4's local frame).
Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class VehicleState:
    position_ned: Vector3
    velocity_ned: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    # w, x, y, z; body FRD -> NED, as PX4 reports it
    attitude: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    landed: bool = False
    # When this state was sampled, in seconds on the same clock as the engine's
    # `now`, so a target can tell how old a detection is.
    stamp: float = 0.0


@dataclass(frozen=True)
class Setpoint:
    """What to send PX4 this tick. None on a field means "not controlled"."""

    position: Vector3 | None = None
    velocity: Vector3 | None = None
    yaw: float | None = None


class Status(enum.Enum):
    RUNNING = "running"
    DONE = "done"
    FAILED = "failed"


ACTION_KINDS = ("takeoff", "land")


@dataclass(frozen=True)
class Action:
    """A request for the executor instead of a setpoint: PX4's own takeoff or land.

    A mission block returns one from step(); the runner calls jl_mission and the
    reply shows up in the next StepContext.action.
    """

    kind: str
    height: float = 0.0  # m above the ground; takeoff only

    def __post_init__(self) -> None:
        if self.kind not in ACTION_KINDS:
            raise ValueError(f"an Action is takeoff or land, got {self.kind!r}")


class ActionStatus(enum.Enum):
    NONE = "none"  # this step hasn't asked for anything
    PENDING = "pending"  # asked, no reply yet
    SUCCEEDED = "succeeded"
    FAILED = "failed"
