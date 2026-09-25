"""CameraTarget: the shared part of every camera-based target block.

Each detection arrives in the camera's own frame. It is turned into an NED
point immediately, using the vehicle state at that moment, and counts as
"seen" for max_age seconds. Detections with NaN, or before the vehicle's
position and attitude are known, are dropped.
"""

from __future__ import annotations

import math

from ..core import Target, Vector3, VehicleState
from ..core.geometry import add, is_finite, rotate, valid_quaternion


class CameraTarget(Target):
    def __init__(self, params: object | None = None) -> None:
        super().__init__(params)
        self.reset()

    def reset(self) -> None:
        self._ned: Vector3 | None = None
        self._seen_at = -math.inf

    def to_body(self, position: Vector3) -> Vector3:
        """Camera frame -> body FRD (forward, right, down)."""
        raise NotImplementedError

    def observe(self, position: Vector3, vehicle: VehicleState) -> None:
        if not (
            is_finite(position)
            and is_finite(vehicle.position_ned)
            and valid_quaternion(vehicle.attitude)
        ):
            return
        offset = rotate(vehicle.attitude, self.to_body(position))
        self._ned = add(vehicle.position_ned, offset)
        self._seen_at = vehicle.stamp

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        if self._ned is None or vehicle.stamp - self._seen_at > self.params.max_age:
            return None
        return self._ned
