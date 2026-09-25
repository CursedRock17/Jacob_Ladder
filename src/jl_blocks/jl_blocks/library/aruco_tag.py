"""aruco_tag: an ArUco tag seen by aruco_tracker, on the front or down camera.

aruco_tracker reports the tag in OpenCV's optical frame (+x right, +y down,
+z out of the lens). The tag id and size are aruco_tracker's own parameters.
Front camera -> body: FrontApproach's _front_optical_to_body. Down camera ->
body: PrecisionLand::getTagWorld's R.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..core import Vector3, block
from .camera import CameraTarget

TOPICS = {"front": "/front/target_pose", "down": "/target_pose"}


@block("aruco_tag")
class ArucoTag(CameraTarget):
    @dataclass
    class Params:
        camera: str = "front"  # front or down
        max_age: float = 0.5  # s a detection counts as "seen"

        def __post_init__(self) -> None:
            if self.camera not in TOPICS:
                raise ValueError(
                    f"camera must be 'front' or 'down', got {self.camera!r}"
                )
            if self.max_age <= 0:
                raise ValueError("max_age must be > 0")

    def topic(self) -> str | None:
        return TOPICS[self.params.camera]

    def to_body(self, position: Vector3) -> Vector3:
        x, y, z = position
        if self.params.camera == "front":
            return (z, x, y)
        return (-y, x, z)
