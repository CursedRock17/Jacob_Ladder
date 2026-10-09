"""Plain data passed between the drone, skills and targets. No ROS types here."""

from __future__ import annotations

import enum
from dataclasses import dataclass

# North, east, down, in metres (PX4's local frame).
Vector3 = tuple[float, float, float]
# w, x, y, z, as PX4 reports attitude (body FRD -> NED).
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class VehicleState:
    position_ned: Vector3
    velocity_ned: Vector3 = (0.0, 0.0, 0.0)
    yaw: float = 0.0
    # w, x, y, z; body FRD -> NED, as PX4 reports it
    attitude: Quaternion = (1.0, 0.0, 0.0, 0.0)
    landed: bool = False
    # When this state was sampled, in seconds on the same clock as the drone's
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
