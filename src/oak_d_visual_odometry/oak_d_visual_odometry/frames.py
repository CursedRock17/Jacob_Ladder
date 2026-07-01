"""cuVSLAM/PX4 frame transform helpers.

cuVSLAM uses the OpenCV camera convention for the rig frame:
  X right, Y down, Z forward.

PX4 expects external vision body pose in:
  world NED: X north, Y east, Z down
  body FRD: X forward, Y right, Z down
"""

import numpy as np


# Camera optical (OpenCV: X right, Y down, Z forward) -> body FRD
# (X forward, Y right, Z down). Columns are the optical basis vectors
# expressed in body FRD, so this rotation depends only on how the camera is
# physically bolted to the airframe.
#
# Forward-facing: the lens looks along body-forward.
#   cam X (right)   -> body Y (right)
#   cam Y (down)    -> body Z (down)
#   cam Z (forward) -> body X (forward)
R_BODY_FROM_CAM_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)

# Down-facing: the lens looks along body-down (nadir), with the top of the
# image toward the front of the drone (image "up" == body forward).
#   cam X (right)   -> body Y (right)
#   cam Y (down)    -> body -X (aft, since image-up is forward)
#   cam Z (forward) -> body Z (down)
R_BODY_FROM_CAM_OPTICAL_DOWN = np.array(
    [
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
    ]
)


def r_ned_from_body_level(init_yaw_offset_rad: float) -> np.ndarray:
    """Rotation from a level body FRD frame to NED, for a given heading.

    Assumes the vehicle is upright (zero roll/pitch) at startup with its nose
    (body +X) pointing at `init_yaw_offset_rad`, measured clockwise from true
    north about the down axis.
    """
    c = np.cos(init_yaw_offset_rad)
    s = np.sin(init_yaw_offset_rad)
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )


def r_ned_from_cuvslam_world(
    init_yaw_offset_rad: float, r_body_cam: np.ndarray = R_BODY_FROM_CAM_OPTICAL
) -> np.ndarray:
    """Rotation from the cuVSLAM startup world frame to NED.

    cuVSLAM initializes its world frame to the rig (== camera optical) frame,
    so this is the composition of the fixed camera mounting
    (`r_body_cam`: optical -> body) with the initial vehicle attitude
    (`r_ned_from_body_level`). Passing the mounting explicitly keeps the
    world->NED alignment consistent with `transform_pose`'s `R_body_rig` for
    any orientation, including a down-facing camera.

    `init_yaw_offset_rad` is the vehicle-body (nose) heading at startup,
    clockwise from true north.
    """
    return r_ned_from_body_level(init_yaw_offset_rad) @ r_body_cam


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Hamilton quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array(
        [
            [
                1 - 2 * (qy * qy + qz * qz),
                2 * (qx * qy - qz * qw),
                2 * (qx * qz + qy * qw),
            ],
            [
                2 * (qx * qy + qz * qw),
                1 - 2 * (qx * qx + qz * qz),
                2 * (qy * qz - qx * qw),
            ],
            [
                2 * (qx * qz - qy * qw),
                2 * (qy * qz + qx * qw),
                1 - 2 * (qx * qx + qy * qy),
            ],
        ]
    )


def rot_to_quat_xyzw(rot: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> Hamilton quaternion (x, y, z, w)."""
    q_wxyz = rot_to_px4_quat(rot)
    return np.array([q_wxyz[1], q_wxyz[2], q_wxyz[3], q_wxyz[0]], dtype=np.float64)


def rot_to_px4_quat(rot: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> PX4 quaternion (w, x, y, z)."""
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif (rot[0, 0] > rot[1, 1]) and (rot[0, 0] > rot[2, 2]):
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float32)


def transform_pose(
    p_world_rig: np.ndarray,
    R_world_rig: np.ndarray,
    R_body_rig: np.ndarray,
    t_body_rig: np.ndarray,
    R_ned_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Compose body pose in NED from cuVSLAM rig pose.

    `t_body_rig` is the rig origin expressed in body FRD.
    """
    R_rig_body = R_body_rig.T
    R_world_body = R_world_rig @ R_rig_body
    p_world_body = p_world_rig - R_world_body @ t_body_rig

    R_ned_body = R_ned_world @ R_world_body
    p_ned_body = R_ned_world @ p_world_body
    return p_ned_body, R_ned_body
