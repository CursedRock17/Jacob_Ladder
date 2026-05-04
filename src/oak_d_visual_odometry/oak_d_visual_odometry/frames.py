"""Frame transform helpers (pure NumPy — no scipy dep).

Three frames in play:

  * SAI world  — gravity-aligned. SpectacularAI default: Y points up
                 (against gravity), X is the initial camera-forward
                 direction projected onto the horizontal plane,
                 Z = X x Y (initial right of the camera).
  * Camera optical — X right, Y down, Z forward (out of lens).
  * Body FRD   — X forward, Y right, Z down. PX4's body frame.
  * NED world  — X north, Y east, Z down. PX4's world frame.

We need to publish the body pose in NED, and the body angular velocity
in body FRD, given:
  - SAI's camera-in-SAI-world pose
  - SAI's camera velocity in SAI world
  - SAI's angular velocity in the camera optical frame
"""
import numpy as np


# Optical (X right, Y down, Z forward) -> FRD (X forward, Y right, Z down).
# Columns are optical basis vectors expressed in FRD.
R_BODY_FROM_CAM_OPTICAL = np.array([
    [0.0, 0.0, 1.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
])


def r_ned_from_sai_world(init_yaw_offset_rad: float) -> np.ndarray:
    """Rotation from SAI world (Y-up, X = initial-camera-forward) to NED.

    `init_yaw_offset_rad` is the heading of the camera-forward axis at VIO
    startup, measured clockwise from true North.
    """
    c = np.cos(init_yaw_offset_rad)
    s = np.sin(init_yaw_offset_rad)
    # Columns: SAI X (initial forward) -> (cos, sin, 0)_NED
    #          SAI Y (up)              -> (0, 0, -1)_NED
    #          SAI Z (initial right)   -> (-sin, cos, 0)_NED
    return np.array([
        [c,   0.0, -s],
        [s,   0.0,  c],
        [0.0, -1.0, 0.0],
    ])


def r_ned_from_basalt_world(init_yaw_offset_rad: float) -> np.ndarray:
    """Rotation from BasaltVIO world (FLU: X=forward, Y=left, Z=up) to NED.

    `init_yaw_offset_rad` is the heading of the camera-forward axis at VIO
    startup, measured clockwise from true North.

    Columns are Basalt basis vectors expressed in NED:
      Basalt X (forward at heading psi) -> ( cos psi,  sin psi,  0)
      Basalt Y (left  at heading psi)   -> ( sin psi, -cos psi,  0)
      Basalt Z (up)                     -> (       0,        0, -1)
    """
    c = np.cos(init_yaw_offset_rad)
    s = np.sin(init_yaw_offset_rad)
    return np.array([
        [c,   s,   0.0],
        [s,  -c,   0.0],
        [0.0, 0.0, -1.0],
    ])


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Hamilton quaternion (x, y, z, w) -> 3x3 rotation matrix.

    Returns R such that v_world = R @ v_local for a quaternion describing
    the local frame's orientation in the world frame.
    """
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n == 0.0:
        return np.eye(3)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw),     2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw),     1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw),     2 * (qy * qz + qx * qw),     1 - 2 * (qx * qx + qy * qy)],
    ])


def rot_to_px4_quat(rot: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> PX4 quaternion (w, x, y, z) as float32[4].

    Uses Shepperd's branch-selecting method for numerical stability across
    all rotations.
    """
    tr = rot[0, 0] + rot[1, 1] + rot[2, 2]
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0  # s = 4 * qw
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif (rot[0, 0] > rot[1, 1]) and (rot[0, 0] > rot[2, 2]):
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0  # s = 4 * qx
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0  # s = 4 * qy
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0  # s = 4 * qz
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return np.array([w, x, y, z], dtype=np.float32)


def transform_pose(
    p_world_cam: np.ndarray,        # camera origin in SAI world (3,)
    R_world_cam: np.ndarray,        # camera orientation in SAI world (3x3)
    R_body_cam: np.ndarray,         # camera optical -> body FRD (3x3)
    t_body_cam: np.ndarray,         # camera origin in body FRD (3,)
    R_ned_world: np.ndarray,        # SAI world -> NED (3x3)
):
    """Compose body pose in NED.

    Returns (p_ned_body, R_ned_body).

    Math:
        T_world_body = T_world_cam * T_body_cam^-1
        T_ned_body   = T_ned_world * T_world_body
    """
    R_cam_body = R_body_cam.T
    R_world_body = R_world_cam @ R_cam_body
    p_world_body = p_world_cam - R_world_body @ t_body_cam

    R_ned_body = R_ned_world @ R_world_body
    p_ned_body = R_ned_world @ p_world_body
    return p_ned_body, R_ned_body


def transform_velocity_world(
    v_world_cam: np.ndarray,
    R_ned_world: np.ndarray,
) -> np.ndarray:
    """Camera linear velocity in SAI world -> body linear velocity in NED.

    Lever-arm correction (omega x r_body_cam) is neglected; for typical
    OAK-D mount distances this is sub-cm/s and EKF2 absorbs it.
    """
    return R_ned_world @ v_world_cam


def transform_angular_velocity(
    omega_optical: np.ndarray,
    R_body_cam: np.ndarray,
) -> np.ndarray:
    """Angular velocity in camera optical frame -> body FRD frame."""
    return R_body_cam @ omega_optical
