"""What is on each airframe. Selected with JL_AIRFRAME (default "generic"), or
`Drone(airframe=...)`.

Everything about the hardware that more than one part of the system needs
lives here, once: the camera names, topics and where each one is mounted. A
test checks the simulated x500_rgbd against its Gazebo model, so they cannot
drift apart.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field

from .perception.camera import Camera, CameraMount

ENV_VAR = "JL_AIRFRAME"


@dataclass(frozen=True)
class Airframe:
    name: str
    cameras: dict[str, Camera] = field(default_factory=dict)

    def camera(self, name: str) -> Camera:
        if name not in self.cameras:
            known = ", ".join(sorted(self.cameras)) or "none"
            raise KeyError(
                f"airframe '{self.name}' has no camera '{name}' (it has: {known})"
            )
        return self.cameras[name]


# The names a realsense2_camera driver publishes, which the simulated camera copies.
REALSENSE_IMAGE = "/camera/color/image_raw"
REALSENSE_INFO = "/camera/color/camera_info"

# A forward camera at the vehicle origin: right for a first look, wrong for
# precise work, because the lever arm is ignored. Define your own airframe.
GENERIC = Airframe(
    "generic",
    {"front": Camera("front", REALSENSE_IMAGE, REALSENSE_INFO, CameraMount())},
)

# gazebo/models/x500_rgbd: a RealSense-style camera 0.12 m ahead of and 0.05 m
# above base_link, with its sensor 0.0125 m further forward in the link.
X500_RGBD = Airframe(
    "x500_rgbd",
    {
        "front": Camera(
            "front",
            REALSENSE_IMAGE,
            REALSENSE_INFO,
            CameraMount(forward=0.1325, down=-0.05),
        )
    },
)

# gazebo/models/x500_dual_cam: monocular front and downward cameras, bridged by
# gazebo/launch/dualcam_bridge.launch.py. The model puts each camera's offset on
# its fixed joint, which does not move the link, so both sit at the vehicle origin.
X500_DUAL_CAM = Airframe(
    "x500_dual_cam",
    {
        "front": Camera(
            "front",
            "/front/camera/image_raw",
            "/front/camera/camera_info",
            CameraMount(),
        ),
        "down": Camera(
            "down",
            "/downward/camera/image_raw",
            "/downward/camera/camera_info",
            CameraMount(facing="down"),
        ),
    },
)

# The real aircraft: an Intel RealSense D435i facing forward. Its colour stream
# is published by the VIO service (oak_d_visual_odometry, camera:=realsense; see
# its config/realsense_params.yaml), so `vio` must be in flight.yaml's helpers.
# The mount is the forward camera's lever arm VIO uses (t_body_cam in
# oak_d_visual_odometry/config/cuvslam_params.yaml, measured 2026-07-16 for the
# forward mount): 12.5 cm forward of the FC, centred, 2.5 cm above it, level.
REALSENSE_DRONE = Airframe(
    "realsense_drone",
    {
        "front": Camera(
            "front",
            "/front/camera/image_raw",
            "/front/camera/camera_info",
            CameraMount(forward=0.125, down=-0.025),
        )
    },
)

AIRFRAMES: dict[str, Airframe] = {
    a.name: a for a in (GENERIC, X500_RGBD, X500_DUAL_CAM, REALSENSE_DRONE)
}


def register(airframe: Airframe) -> Airframe:
    """Add your own airframe, so JL_AIRFRAME=<name> finds it."""
    if airframe.name in AIRFRAMES:
        raise ValueError(f"an airframe named '{airframe.name}' already exists")
    AIRFRAMES[airframe.name] = airframe
    return airframe


def get_airframe(name: str | None = None) -> Airframe:
    name = name or os.environ.get(ENV_VAR) or "generic"
    if name not in AIRFRAMES:
        raise KeyError(
            f"unknown airframe '{name}' (known: {', '.join(sorted(AIRFRAMES))})"
        )
    return AIRFRAMES[name]
