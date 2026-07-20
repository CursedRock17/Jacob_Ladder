# oak_d_visual_odometry

ROS 2 publishers for an OAK-D S2 running NVIDIA cuVSLAM on the host. The
package publishes cuVSLAM pose, raw OAK IMU, optional RGB/stereo debug images,
and an optional PX4 `VehicleOdometry` bridge over uXRCE-DDS.

NVIDIA cuVSLAM is the only VSLAM backend in this package.

## Quick Start

```bash
colcon build --packages-select oak_d_visual_odometry
source install/setup.bash
ros2 launch oak_d_visual_odometry cuvslam_px4.launch.py
```

The launch file loads `config/cuvslam_params.yaml` — that file is the single
source of truth for the flight configuration (device pin, mounting, gates,
topics). Edit it, rebuild, relaunch; nothing needs to be passed on the
command line. For remote viewing over WiFi run
`launch_scripts/foxglove_wifi.sh` (see
`general_docs/reviewing_flight_data.md`), or pass `foxglove_bridge:=true` to
the launch file to start a local bridge instead.

Since the OAK-D Lite was removed (2026-07-16) the S2 is the only camera: its
CAM_B/CAM_C stereo pair drives cuVSLAM while CAM_A color is published on
`/front/camera/image_raw` (10 fps) for the YOLO drogue detector — one device,
one process, both pipelines.

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

Build through the **venv's** colcon, not the system one:

```bash
python3 -m colcon build --packages-select oak_d_visual_odometry --symlink-install
source install/setup.bash
```

`/usr/bin/colcon` has a `#!/usr/bin/python3` shebang, so it generates console
scripts pointed at the *system* interpreter — which cannot see `depthai` or
`cuvslam` (both live only in `.venv`). The node then dies at launch with
`ModuleNotFoundError: No module named 'depthai'`. Running colcon as
`python3 -m colcon` with the venv on `PATH` writes
`#!/usr/bin/env python3` instead, which resolves correctly.

Do **not** `source .venv/bin/activate` before building: that combination
writes package metadata the launch-time interpreter cannot find, and fails
differently (`PackageNotFoundError: No package metadata was found for
oak-d-visual-odometry`). The venv is already on `PATH` from the shell
profile; no activation is needed.

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
| `/features/image` | `sensor_msgs/Image` | left frame with tracked features drawn on it |
| `~/diagnostics` | `diagnostic_msgs/DiagnosticArray` | measured publish rates + latency at 1 Hz — see [Frame rate](#frame-rate) |

All topic names are parameters. The shipped `cuvslam_params.yaml` renames the
CAM_A topics to `/front/camera/image_raw` and `/front/camera/camera_info` so
the color stream feeds the drogue detector directly.

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
| `device_id` | `""` | DepthAI deviceId (MXID) to open; empty opens the first available device. The config pins the OAK-D S2 (`1944301041EB1B1300`) so VIO never grabs the OAK-D Lite drogue camera |
| `width`, `height` | `640`, `480` | Stereo image size requested from DepthAI (the shipped config uses 640x400) |
| `camera_fps` | `30.0` | CAM_B/CAM_C frame rate for cuVSLAM (the shipped PX4 config uses `40.0` — read [Frame rate](#frame-rate) before changing it) |
| `rgb_fps` | `30.0` | CAM_A RGB frame rate, also the drogue detector's input rate (the shipped PX4 config uses `10.0`; every RGB frame costs ~1.1 stereo frames — see [Frame rate](#frame-rate)) |
| `publish_rgb` | `true` | Publish CAM_A color frames |
| `features_max_fps` | `10.0` | Rate cap on the `/features/image` debug overlay. The overlay costs a colorspace convert, a draw per feature, and a ~768 KB publish on every frame, all inside the tracking loop; it is also skipped entirely when nobody is subscribed |
| `publish_stereo_images` | `false` | Publish mono stereo debug images |
| `publish_imu` | `true` | Publish raw OAK IMU on `/imu/data` |
| `enable_imu_fusion` | `false` | Feed OAK IMU samples into cuVSLAM (Inertial odometry mode) |
| `imu_extrinsics_source` | `calibration` | IMU->CAM_A extrinsics for fusion: `calibration` reads the device EEPROM and falls back to the `rig_from_imu_*` params; `params` uses them directly. The identity placeholder is refused (fusion falls back to stereo-only) because it is never correct for an OAK-D |
| `rig_from_imu_translation`, `rig_from_imu_rotation_xyzw` | identity | Manual IMU->rig extrinsics fallback |
| `init_yaw_offset_deg` | `0.0` | Heading of camera-forward at startup, deg CW from north |
| `camera_mounting` | `forward` | Mounting preset: `forward`, `down` (nadir), or `down<degrees>` for a tilted mount, where `<degrees>` is the angle of depression of the boresight below body-horizontal (e.g. `down30`, `down45`) |
| `t_body_cam` | `[0,0,0]` | cuVSLAM rig origin in body FRD, meters |
| `R_body_cam_override` | `[]` | 9-element row-major override of rig/OpenCV -> body-FRD |
| `position_variance` | `[0.01,0.01,0.02]` | NED position variance floor for PX4 |
| `orientation_variance` | `[0.01,0.01,0.02]` | Orientation variance floor for PX4 |
| `velocity_variance` | `[0.1,0.1,0.1]` | Velocity variance for the PX4 windowed velocity |
| `use_cuvslam_covariance` | `true` | Report cuVSLAM's per-sample pose covariance to PX4 wherever it exceeds the variance floors |
| `velocity_window_frames` | `10` | Rolling NED pose window: PX4 velocity is the least-squares slope over it (a two-frame diff at 30 Hz amplifies ~5 mm pose noise into ~0.3 m/s velocity noise) |
| `use_empirical_covariance` | `true` | Report the residual scatter about the window's constant-velocity fit as position variance wherever it exceeds the floors — unlike cuVSLAM's covariance it actually grows when tracking degrades |
| `slow_track_warn_ms` | `60.0` | Warn when one `track()` call blocks the pipeline loop longer than this |
| `velocity_reset_gap_s` | `0.5` | Stereo-frame gap beyond which velocity is not differentiated across the gap and `reset_counter` is bumped |
| `max_vio_speed_mps` | `4.0` | Frame-to-frame speed above which the pose is withheld from PX4 as a mistracking/dead-reckoning runaway (must exceed the real flight envelope) |
| `reacquire_stable_frames` | `6` | Consecutive sane frames required after a tracking loss before PX4 publishing resumes (prevents lost/re-acquired thrash) |
| `min_landmarks_3d` | `10` | Scene-quality gate: poses constrained by fewer triangulated 3D landmarks are withheld from PX4 as tracking loss. Blocks the covered-lens failure where self-consistent noise "tracking" (hundreds of 2D tracks, zero landmarks) passes the speed gate and reacquire hysteresis |
| `debug_dump_directory` | `""` | When set, cuVSLAM dumps an EDEX scene file + TGA images there for offline replay with NVIDIA's `tracker`/`result_visualizer` tools (see cuVSLAM `TROUBLESHOOTING.md`); heavy disk I/O, leave empty for normal flights |
| `use_denoising` | `false` | cuVSLAM input-image denoising (try when features die without reason on noisy images) |
| `use_motion_model` | `true` | cuVSLAM 2D-observation motion prediction (try toggling if repetitive structure causes outliers) |
| `max_frame_delta_s` | `-1.0` | cuVSLAM's broken-stream threshold; `<= 0` auto-selects 3 frame intervals from `camera_fps` (0.1 s at 30 fps) so the tracker treats a stall as a discontinuity instead of LK-tracking across it |
| `multicam_mode` | `""` | cuVSLAM multicamera engine: `moderate`, `performance`, or `precision`; empty keeps the PyCuVSLAM default (`precision` in 16.0, Isaac ROS ships `performance`) |
| `debug_imu_mode` | `false` | NVIDIA's IMU-extrinsics check: visual tracking off, translation locked, rotation from the IMU alone — rotate the rig and compare. Requires `enable_imu_fusion`; bench only, never fly |

The PX4 publisher also stops publishing while cuVSLAM reports an invalid
pose and increments the `VehicleOdometry.reset_counter` when tracking is
re-acquired, so EKF2 re-references instead of fusing the jump as motion.

## Frame rate

What EKF2 fuses is the rate `/slam/odometry` actually *publishes* and how
stale each pose is when it arrives — neither follows from `camera_fps`.

**The binding constraint is a device-wide budget of ~50 delivered frames per
second, shared between the stereo pair and CAM_A.** It is not the sensors
(the OV7251 stereo pair does 640x480 at 117 fps) and not host compute
(`track()` means 4–6 ms, a ~200 Hz ceiling). Once `camera_fps + rgb_fps`
exceeds ~50 the sync queue backlogs, pose age explodes, and the stereo rate
plateaus or *goes backwards*.

Measured on this rig, 2026-07-20:

| `camera_fps` | `rgb_fps` | total | tracked Hz | `frame_gaps` | `ev_lat` |
|---|---|---|---|---|---|
| 30 | 10 | 40 | 30.0 | 0 | 39 ms |
| **40** | **10** | **50** | **40.8** | **0** | **60 ms** |
| 40 | 15 | 55 | 40.1 | 0 | 230 ms |
| 45 | 10 | 55 | 44.0 | 0 | 242 ms |
| 50 | 10 | 60 | 42.5 | 0 | 222 ms |
| 60 | 10 | 70 | 38.4 | 17 | 245 ms |
| 60 | 30 | 90 | 22.4 | 70 | 405 ms |

Latency beats raw rate here, because `ev_lat` is what `EKF2_DELAY_MAX` has to
cover — a 405 ms pose age against a 200 ms replay buffer is fusion
mis-registration, which is the failure the 40/10 setting exists to avoid.

### Reading the real rate

**Do not trust a rate read off a Foxglove panel.** foxglove_bridge drops
messages under WiFi bandwidth pressure, and `foxglove_wifi.sh` republishes
images through a JPEG encoder, so a panel can show ~10 Hz while
`/slam/odometry` genuinely runs at 40. Measured on the bench 2026-07-20 with
the shipped 40/10 config:

| topic | true rate |
|---|---|
| `/slam/odometry` | 40.0 Hz |
| `/fmu/in/vehicle_visual_odometry` | 40.2 Hz |
| `/front/camera/image_raw` | 7.7 Hz (`rgb_fps` 10) |
| `/features/image` | 7.3 Hz (`features_max_fps` 10) |

The node publishes ground truth on `~/diagnostics` (a
`diagnostic_msgs/DiagnosticArray` at 1 Hz) — view it with Foxglove's
**Diagnostics** panel. Rates are counted at the `publish()` call, upstream of
any transport, so they are the node's real output regardless of what a viewer
receives. Keys: `tracked_fps`, `camera_fps_requested`, `odometry_fps`,
`px4_fps`, `rgb_fps`, `features_fps`, `ev_latency_ms`, `landmarks_3d`,
`reset_counter`. The status goes ERROR when the stereo rate falls more than
20 % short of `camera_fps` (the frame budget is oversubscribed) and WARN when
`ev_latency_ms` exceeds 150.

From a terminal, `ros2 topic hz /slam/odometry` is the other authoritative
check.

### Verifying a change

Verify any change against the 10 s stats line:

```
track() stats over 10s: n=... mean=..ms max=..ms slow=.. frame_gaps=.. ... ev_lat=..ms
```

- `n / 10` must **equal** `camera_fps`. If it is below, the request exceeds
  the budget — lower it rather than raising it.
- `frame_gaps` must be `0`.
- `ev_lat` should stay under ~100 ms.
- `mean` is track() time; it has never been the limit on this rig.

If `ev_lat` climbs, spend budget rather than adding it: lower `rgb_fps`
first (it buys roughly 1.1 stereo frames each), then `camera_fps`.
`publish_features` and `publish_imu` measured free — they are not the lever.

## Mounting

`camera_mounting` selects the fixed camera-optical -> body-FRD rotation:

| Preset | Meaning |
|---|---|
| `forward` | boresight along body +X, image top toward the sky. **Current mount.** |
| `down` | nadir, image top toward the nose |
| `down<degrees>` | forward camera pitched down by that angle of depression (e.g. `down30`, `down45`) |

A 9-element row-major `R_body_cam_override` beats the preset for any other
orientation, and `t_body_cam` (rig origin in body FRD, meters) accounts for
the lever arm between the camera and the flight controller.

Mount history on this airframe: the 30° and 45° depressed mounts both
produced in-flight scale errors; the forward mount (2026-07-16) ground-tests
near-perfect on `/slam/odometry`. If the mount ever changes:

1. Set the new preset (or override) here **and** re-verify the physical
   angle: log a static period and check where gravity sits in the camera
   optical frame via the EEPROM IMU->CAM_A extrinsics (the 30° mount was
   confirmed at 29.9° this way).
2. Re-run the MAVLink Inspector axis checks under PX4 Notes below before
   trusting a flight.

## Frames

cuVSLAM publishes in an OpenCV startup world:

- `+X`: camera right
- `+Y`: camera down
- `+Z`: camera forward

The PX4 node converts that world to NED using `init_yaw_offset_deg`, then
converts the rig pose to body FRD using the mounting rotation above. See
`frames.py` for the exact math.

## PX4 Notes

`px4_msgs` is imported conditionally. The PX4 publisher activates when
`from px4_msgs.msg import VehicleOdometry` succeeds in the sourced workspace.

Velocity is the least-squares slope of the last `velocity_window_frames`
NED positions (see the parameter table); the window is cleared on tracking
loss and stream gaps so velocity is never fit across a pose jump. Tune
`velocity_variance` upward if EKF2 over-trusts it.

`VehicleOdometry.timestamp_sample` is the camera capture time (publish time
minus the measured capture->publish latency via DepthAI's host-synced
`getTimestamp`), so `EKF2_EV_DELAY` should be ~0 and only cover residual
transport lag. `quality` scales with the triangulated 3D landmark count
(2x landmarks, capped at 100) so `EKF2_EV_QMIN` can de-weight marginal
scenes; it is 100 only when the landmark API is unavailable.

**Flight-proven EKF2 configuration (2026-07-16, forward mount):**
`EKF2_EV_CTRL = 15` — all four fusion bits are required. Position-only
fusion drifted 10–20 m (the windowed EV velocity is what pins the drift to
~5 cm), and without EV yaw PX4 never declares horizontal position stable
enough to arm in Position mode. See `config/params/README.md` at the repo
root for the full parameter rationale.

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
