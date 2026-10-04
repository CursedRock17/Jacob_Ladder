"""What a Drone needs from the outside world. The ROS version lives in
jl_blocks.ros.fly_backend; tests use a fake. No ROS here."""

from __future__ import annotations

from typing import Protocol

from ..core import Setpoint, VehicleState


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
        """True while the mission mode is selected, i.e. while the script is in charge."""
        ...

    def publish(self, setpoint: Setpoint) -> None: ...

    # The three executor calls block until PX4 is done, and return (ok, message).
    def arm(self) -> tuple[bool, str]: ...

    def takeoff(self, height: float) -> tuple[bool, str]: ...

    def land(self) -> tuple[bool, str]: ...
