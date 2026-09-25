"""yolo_drogue: the drogue from the YOLO pipeline's pose_estimation_node.

It publishes the drogue on /tag_detections in a ranging frame (+x left,
+y up, +z forward). Port of DroneSmoothPlanner::drogueTargetNed: ranging ->
body FRD is (z, -x, -y), then tilted by the camera's mount pitch.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..core import Vector3, block
from .camera import CameraTarget


@block("yolo_drogue")
class YoloDrogue(CameraTarget):
    @dataclass
    class Params:
        camera_pitch_deg: float = 0.0  # deg, positive = tilted down
        max_age: float = 0.5  # s a detection counts as "seen"

        def __post_init__(self) -> None:
            if self.max_age <= 0:
                raise ValueError("max_age must be > 0")

    def topic(self) -> str | None:
        return "/tag_detections"

    def to_body(self, position: Vector3) -> Vector3:
        x_left, y_up, z_forward = position
        fx, fy, fz = z_forward, -x_left, -y_up
        pitch = math.radians(self.params.camera_pitch_deg)
        return (
            math.cos(pitch) * fx - math.sin(pitch) * fz,
            fy,
            math.sin(pitch) * fx + math.cos(pitch) * fz,
        )
