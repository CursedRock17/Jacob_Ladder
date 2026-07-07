# oak_d_visual_odometry

ROS 2 publishers for an OAK-D Lite running NVIDIA cuVSLAM on the host. The
package publishes cuVSLAM pose, raw OAK IMU, optional RGB/stereo debug images,
and an optional PX4 `VehicleOdometry` bridge over uXRCE-DDS.

NVIDIA cuVSLAM is the only VSLAM backend in this package.

## Python Dependencies

`depthai`, `numpy`, and NVIDIA's `cuvslam` wheel are not ROS packages and will
not be installed by `rosdep`. Use the repo root uv environment; it must be
created with system site packages enabled so ROS 2 Python modules, `cv_bridge`,
and JetPack OpenCV remain visible.

The repo root `pyproject.toml` pins the current target PyCuVSLAM wheel:

```bash
uv venv --python 3.10 --system-site-packages .venv
uv sync
source .venv/bin/activate
```

`pyproject.toml` pins `numpy<2` because many ROS 2 `cv_bridge` builds are
compiled against NumPy 1.x. Do not install a separate `opencv-python` wheel for
this node; use the JetPack/system OpenCV that matches `cv_bridge`.

## USB Permissions

DepthAI needs a udev rule so non-root users can open the OAK-D:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the OAK-D after adding the rule.

## Build

```bash
colcon build --packages-select oak_d_visual_odometry
source install/setup.bash
```

## Launch

```bash
ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py
```

## Executables

Run one OAK-D process at a time. DepthAI claims the device exclusively.

### `rgb_publisher`

Simple CAM_A color camera utility.

| Topic | Type | Notes |
|---|---|---|
| `/rgb/image` | `sensor_msgs/Image` | BGR8, 640x400 @ 30 fps |

```bash
ros2 run oak_d_visual_odometry rgb_publisher
```

### `cuvslam_publisher_node`

Runs host-side PyCuVSLAM using synchronized CAM_B/CAM_C mono frames. CAM_A/RGB
is the cuVSLAM rig origin, matching NVIDIA's OAK-D sample. Raw OAK IMU is
published for inspection; cuVSLAM IMU fusion is disabled by default until IMU
extrinsics and noise parameters are calibrated.

| Topic | Type | Notes |
|---|---|---|
| `/slam/odometry` | `geometry_msgs/PoseStamped` | rig-in-`cuvslam_world`, OpenCV axes |
| `/tf` | `tf2_msgs/TFMessage` | `cuvslam_world` -> `oak_camera` |
| `/imu/data` | `sensor_msgs/Imu` | raw OAK IMU passthrough, 200 Hz |
| `/rgb/image` | `sensor_msgs/Image` | optional CAM_A, BGR8 |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | CAM_A intrinsics |
| `/left/image`, `/right/image` | `sensor_msgs/Image` | optional mono debug images |

```bash
ros2 run oak_d_visual_odometry cuvslam_publisher_node \
  --ros-args \
  --params-file install/oak_d_visual_odometry/share/oak_d_visual_odometry/config/cuvslam_params.yaml
```

### `cuvslam_publisher_px4_node`

Adds PX4 output on top of `cuvslam_publisher_node`. It converts cuVSLAM's
OpenCV startup world into NED body-FRD and publishes
`/fmu/in/vehicle_visual_odometry`.

```bash
ros2 run oak_d_visual_odometry cuvslam_publisher_px4_node \
  --ros-args \
  --params-file install/oak_d_visual_odometry/share/oak_d_visual_odometry/config/cuvslam_params.yaml \
  -p init_yaw_offset_deg:=0.0 \
  -p t_body_cam:='[0.10, 0.0, 0.05]'
```

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `width`, `height` | `640`, `400` | Stereo image size requested from DepthAI |
| `camera_fps` | `30.0` | CAM_B/CAM_C frame rate for cuVSLAM |
| `rgb_fps` | `15.0` | CAM_A RGB frame rate |
| `publish_rgb` | `true` | Publish CAM_A color frames |
| `publish_stereo_images` | `false` | Publish mono stereo debug images |
| `publish_imu` | `true` | Publish raw OAK IMU on `/imu/data` |
| `enable_imu_fusion` | `false` | Feed OAK IMU samples into cuVSLAM |
| `init_yaw_offset_deg` | `0.0` | Heading of camera-forward at startup, deg CW from north |
| `camera_mounting` | `forward` | Mounting preset: `forward`, `down`, or `down45` (45° depression) |
| `t_body_cam` | `[0,0,0]` | cuVSLAM rig origin in body FRD, meters |
| `R_body_cam_override` | `[]` | 9-element row-major override of rig/OpenCV -> body-FRD |
| `position_variance` | `[0.01,0.01,0.02]` | NED position variance for PX4 |
| `orientation_variance` | `[0.01,0.01,0.02]` | Orientation variance for PX4 |
| `velocity_variance` | `[0.1,0.1,0.1]` | Velocity variance for PX4 finite-difference velocity |

## Frames

cuVSLAM publishes in an OpenCV startup world:

- `+X`: camera right
- `+Y`: camera down
- `+Z`: camera forward

The PX4 node converts that world to NED using `init_yaw_offset_deg`, then
converts the rig pose to body FRD using `t_body_cam` and
`R_body_cam_override`. See `frames.py` for the exact math.

## PX4 Notes

`px4_msgs` is imported conditionally. The PX4 publisher activates when
`from px4_msgs.msg import VehicleOdometry` succeeds in the sourced workspace.

Velocity is estimated by finite-differencing cuVSLAM NED position. Tune
`velocity_variance` upward if EKF2 over-trusts it.

Before flight, use QGroundControl MAVLink Inspector to confirm the received
ODOMETRY axes:

1. Forward motion increases `x`.
2. Right motion increases `y`.
3. Lifting the vehicle decreases `z`.
4. Yaw alignment can reach approximately unit quaternion without roll/pitch.

## Files

```text
oak_d_visual_odometry/
├── config/cuvslam_params.yaml
└── oak_d_visual_odometry/
    ├── __init__.py
    ├── cuvslam_publisher_node.py
    ├── cuvslam_publisher_px4_node.py
    ├── frames.py
    └── rgb_publisher.py
```
