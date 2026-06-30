# oak_d_visual_odometry

ROS 2 publishers for an OAK-D Lite running on-device BasaltVIO. Provides
visual-inertial odometry, IMU, and RGB streams, with an optional PX4 bridge
over uXRCE-DDS.

## Python dependencies

`depthai`, `numpy`, and `opencv-python` are not ROS packages and won't be
installed by `rosdep`. You must pip-install them before running any node.

**Recommended — venv with system-site-packages:**

The `--system-site-packages` flag lets the venv see ROS packages (`rclpy`,
`cv_bridge`, etc.) while keeping the DepthAI stack isolated.

```bash
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install -r requirements.txt
```

Activate the venv in every terminal before running a node:

```bash
source venv/bin/activate
ros2 run oak_d_visual_odometry <executable>
```

**Alternative — install into the system Python:**

If you'd rather not manage a venv, install directly (this is the Python ROS
uses):

```bash
pip install -r requirements.txt
```

**USB device permissions (first-time only):**

DepthAI needs a udev rule so non-root users can open the OAK-D:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the OAK-D after running this once.

**numpy pin:** `requirements.txt` pins `numpy<2`. This is intentional —
`cv_bridge` on most ROS 2 distros is compiled against numpy 1.x and will
segfault at runtime if numpy 2.x is loaded. Do not remove the pin.

## Build

```bash
colcon build --packages-select oak_d_visual_odometry
source install/setup.bash
```

## Executables

Run **one at a time** — DepthAI only allows a single host process to claim
the OAK-D device.

### `rgb_publisher`

Streams CAM_A (color) only.

| Topic | Type | Notes |
|---|---|---|
| `/rgb/image` | `sensor_msgs/Image` | BGR8, 640x400 @ 30 fps |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | intrinsics from device flash, same stamp as image |

```bash
ros2 run oak_d_visual_odometry rgb_publisher
```

### `vo_publisher_node`

Stereo + IMU + BasaltVIO + CAM_A. Pose is in BasaltVIO's own
gravity-aligned world frame. Use this for visualization in Foxglove.

| Topic | Type | Notes |
|---|---|---|
| `/slam/odometry` | `geometry_msgs/PoseStamped` | camera-in-world |
| `/tf` | `tf2_msgs/TFMessage` | `world` → `oak_camera` |
| `/imu/data` | `sensor_msgs/Imu` | raw accel + gyro, 200 Hz |
| `/rgb/image` | `sensor_msgs/Image` | CAM_A, BGR8, 30 fps |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | intrinsics from device flash, same stamp as image |

```bash
ros2 run oak_d_visual_odometry vo_publisher_node
```

### `vo_publisher_px4_node`

Same publishers as `vo_publisher_node` **plus** a PX4 bridge that converts
the BasaltVIO pose into NED body-FRD and publishes
`/fmu/in/vehicle_visual_odometry` for PX4 EKF2 fusion. Velocity is
estimated by finite-difference of the NED position.

| Topic | Type | Notes |
|---|---|---|
| (everything from `vo_publisher_node`) | | |
| `/rgb/camera_info` | `sensor_msgs/CameraInfo` | same as above |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/VehicleOdometry` | NED pose, NED velocity, BEST_EFFORT QoS |

```bash
ros2 run oak_d_visual_odometry vo_publisher_px4_node \
  --ros-args \
  -p init_yaw_offset_deg:=0.0 \
  -p t_body_cam:='[0.10, 0.0, 0.05]'
```

**Parameters:**

| Name | Default | Meaning |
|---|---|---|
| `init_yaw_offset_deg` | `0.0` | Heading of camera-forward at startup, deg CW from North |
| `t_body_cam` | `[0,0,0]` | Camera origin in body FRD (m) |
| `R_body_cam_override` | `[]` | 9-elem row-major override of camera-optical→body-FRD; empty = canonical mount |
| `position_variance` | `[0.01,0.01,0.02]` | NED position covariance (m²) |
| `orientation_variance` | `[0.01,0.01,0.02]` | Orientation covariance (rad²) |
| `velocity_variance` | `[0.1,0.1,0.1]` | Velocity covariance ((m/s)²) — bump if EKF2 over-trusts the noisy finite-diff |

## Need-to-knows

**`px4_msgs` is a soft dependency.** It's intentionally not listed in
`package.xml`. The PX4 publisher activates if `from px4_msgs.msg import
VehicleOdometry` succeeds at runtime (the package is in your overlay), and
silently no-ops otherwise. Build `px4_msgs` separately and source the
overlay before running `vo_publisher_px4_node`.

**Frames.** BasaltVIO publishes in FLU world (X-forward, Y-left, Z-up at
startup). The PX4 node converts to NED via `init_yaw_offset_deg` and to
body FRD via `t_body_cam` + `R_body_cam_override`. See `frames.py` for
the math.

**PX4 verification before flight** (per the PX4 VIO guide):
1. Set `MAV_ODOM_LP=1`, watch `ODOMETRY` in QGC MAVLink Inspector.
2. Yaw the drone until the message quaternion → unit (w=1, x=y=z=0). If
   you can't get there without rolling/pitching, your camera mount math
   is wrong.
3. Lift drone → z decreases. Forward → x increases. Right → y increases.
4. Set `MAV_ODOM_LP=0`.

**EKF2 tuning.** Set `EKF2_EV_CTRL` (enable horizontal+vertical position
+ velocity + yaw fusion), `EKF2_HGT_REF=Vision`, `EKF2_EV_DELAY` (ms
between measurement timestamp and capture — start at ~50, tune from
logs), and `EKF2_EV_POS_X/Y/Z` to the camera position on the body.

**Velocity quality.** The PX4 node sends a finite-difference of NED
position (~30 Hz). It's noisy by control-loop standards. EKF2 fuses it
with the FCU IMU; raise `velocity_variance` if it's over-trusted.

**Single device claim.** All three executables open the OAK-D
exclusively. Stop one before starting another. If a previous run left
the device in a bad state, unplug/replug.

**Numpy ABI.** `cv_bridge` is built against ROS's numpy. If you run
inside a venv with a different numpy, `cv2_to_imgmsg` can segfault. Pin
`numpy<2` in the venv or run with the venv deactivated. Each node prints
`shape=... dtype=...` on the first frame to help diagnose this.

## Files

```
oak_d_visual_odometry/
├── __init__.py
├── frames.py                    # rotation helpers (FLU/SAI → NED, optical → FRD)
├── rgb_publisher.py             # CAM_A only
├── vo_publisher_node.py         # stereo + IMU + RGB + BasaltVIO
└── vo_publisher_px4_node.py     # ↑ + /fmu/in/vehicle_visual_odometry
```
