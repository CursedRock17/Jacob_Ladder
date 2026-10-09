"""What a Drone needs from the outside world. The ROS version lives in
jl_flight.ros.vehicle_link; tests use a fake. No ROS here."""

from __future__ import annotations

from collections.abc import Callable
from typing import Protocol

from .core import Setpoint, Vector3, VehicleState


class Backend(Protocol):
    def connect(self) -> None: ...

    def close(self) -> None: ...

    def now(self) -> float:
        """Seconds on a clock that only goes forward."""
        ...

    def sleep(self, seconds: float) -> None: ...

    def vehicle(self) -> VehicleState | None:
        """The latest state, or None if PX4 has not reported lately."""
        ...

    def is_active(self) -> bool:
        """True while the mission mode is selected, i.e. while the mission is in charge."""
        ...

    def selections(self) -> int:
        """How many times the mode has been selected since connecting (a mode
        that was already selected counts once)."""
        ...

    def publish(self, setpoint: Setpoint) -> None: ...

    def subscribe_pose(self, topic: str, callback: Callable[[Vector3], None]) -> None:
        """Call `callback` with the position of each geometry_msgs/PoseStamped on `topic`."""
        ...

    def event(self, text: str) -> None:
        """Say what the mission is doing, for logs and watchers."""
        ...

    # The three executor calls block until PX4 is done, and return (ok, message).
    def arm(self) -> tuple[bool, str]: ...

    def takeoff(self, height: float) -> tuple[bool, str]: ...

    def land(self) -> tuple[bool, str]: ...
