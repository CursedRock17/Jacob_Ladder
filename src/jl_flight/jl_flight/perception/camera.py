"""Where a camera sits on the airframe, and what it is called on the wire."""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Vector3
from ..core.geometry import add

FACINGS = ("front", "down")


@dataclass(frozen=True)
class CameraMount:
    """A camera's position on the airframe and which way it looks.

    forward / right / down are metres from the vehicle origin (the point PX4
    reports as the position) in the body's FRD axes, so a camera mounted above
    the origin has a negative `down`. facing is "front" or "down"; a front
    camera may also be tilted down by `pitch_deg`.
    """

    forward: float = 0.0
    right: float = 0.0
    down: float = 0.0
    facing: str = "front"
    pitch_deg: float = 0.0  # front cameras only: positive = tilted down

    def __post_init__(self) -> None:
        if self.facing not in FACINGS:
            raise ValueError(f"facing must be one of {FACINGS}, got {self.facing!r}")
        if self.pitch_deg and self.facing != "front":
            raise ValueError("pitch_deg is for front cameras only")

    def to_body(self, point: Vector3) -> Vector3:
        """A point in OpenCV's optical frame (+x right, +y down, +z out of the
        lens) as body FRD metres from the vehicle origin, camera offset included."""
        x, y, z = point
        if self.facing == "down":
            body = (-y, x, z)
        else:
            # Optical -> FRD for a level camera, then tilted down by the mount pitch.
            fx, fy, fz = z, x, y
            p = math.radians(self.pitch_deg)
            body = (
                math.cos(p) * fx - math.sin(p) * fz,
                fy,
                math.sin(p) * fx + math.cos(p) * fz,
            )
        return add(body, (self.forward, self.right, self.down))


@dataclass(frozen=True)
class Camera:
    name: str
    image_topic: str  # sensor_msgs/Image
    info_topic: str  # sensor_msgs/CameraInfo
    mount: CameraMount = CameraMount()
