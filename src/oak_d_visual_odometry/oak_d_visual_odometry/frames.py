"""cuVSLAM/PX4 frame transform helpers.

cuVSLAM uses the OpenCV camera convention for the rig frame:
  X right, Y down, Z forward.

PX4 expects external vision body pose in:
  world NED: X north, Y east, Z down
  body FRD: X forward, Y right, Z down
"""

import numpy as np


# Optical/OpenCV (X right, Y down, Z forward) -> body FRD
# (X forward, Y right, Z down).
# Columns are optical basis vectors expressed in body FRD.
R_BODY_FROM_CAM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def r_ned_from_cuvslam_world(init_yaw_offset_rad: float) -> np.ndarray:
    """Rotation from cuVSLAM startup world to NED.

    cuVSLAM initializes its world frame to the rig frame. For an upright
    forward-facing rig, OpenCV +Z is forward, +X is right, and +Y is down.

    `init_yaw_offset_rad` is the heading of camera-forward at startup,
    measured clockwise from true north.
    """
    c = np.cos(init_yaw_offset_rad)
    s = np.sin(init_yaw_offset_rad)
    return np.array([
        [-s, 0.0, c],
        [c,  0.0, s],
        [0.0, 1.0, 0.0],
    ])


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Hamilton quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


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
