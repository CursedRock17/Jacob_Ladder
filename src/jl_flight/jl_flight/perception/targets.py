"""Targets built from camera detections.

A detection arrives in the camera's own frame. It is turned into an NED point
right away, using the vehicle state at that moment, and counts as "seen" for
`max_age` seconds. Detections with NaN, or before the vehicle's position and
attitude are known, are dropped.
"""

from __future__ import annotations

from ..core import Target, Vector3, VehicleState
from ..core.geometry import add, is_finite, rotate, valid_quaternion
from .camera import CameraMount


class CameraTarget(Target):
    def __init__(self, max_age: float = 0.5) -> None:
        if max_age <= 0:
            raise ValueError("max_age must be > 0")
        self.max_age = max_age
        # One tuple, swapped in a single assignment: detections arrive on a
        # ROS thread while the drone's thread reads the estimate.
        self._seen: tuple[Vector3, float] | None = None

    def reset(self) -> None:
        """Forget the last detection."""
        self._seen = None

    def to_body(self, position: Vector3) -> Vector3:
        """Camera frame -> body FRD, metres from the vehicle origin."""
        raise NotImplementedError

    def observe(self, position: Vector3, vehicle: VehicleState) -> None:
        if not (
            is_finite(position)
            and is_finite(vehicle.position_ned)
            and valid_quaternion(vehicle.attitude)
        ):
            return
        offset = rotate(vehicle.attitude, self.to_body(position))
        self._seen = (add(vehicle.position_ned, offset), vehicle.stamp)

    def estimate(self, vehicle: VehicleState) -> Vector3 | None:
        seen = self._seen
        if seen is None or vehicle.stamp - seen[1] > self.max_age:
            return None
        return seen[0]


FRAMES = ("optical", "ranging")


def ranging_to_optical(point: Vector3) -> Vector3:
    """The YOLO pose node's ranging frame (+x left, +y up, +z forward) as OpenCV's
    optical frame (+x right, +y down, +z forward)."""
    x, y, z = point
    return (-x, -y, z)


class OpticalTarget(CameraTarget):
    """Detections from a camera with a known mount, in `frame`: "optical" (OpenCV,
    as aruco_tracker publishes) or "ranging" (as the YOLO pose node publishes)."""

    def __init__(
        self, mount: CameraMount, max_age: float = 0.5, frame: str = "optical"
    ) -> None:
        super().__init__(max_age)
        if frame not in FRAMES:
            raise ValueError(f"frame must be one of {FRAMES}, got {frame!r}")
        self.mount = mount
        self.frame = frame

    def to_body(self, position: Vector3) -> Vector3:
        if self.frame == "ranging":
            position = ranging_to_optical(position)
        return self.mount.to_body(position)
