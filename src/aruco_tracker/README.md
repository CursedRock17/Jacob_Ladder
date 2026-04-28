# ArUco Tracker

A ROS 2 node that detects [ArUco markers](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) in a camera image and publishes the marker's 3D pose relative to the camera. This is the perception layer used by the precision landing and front-approach flight modes in Jacob's Ladder.

## How It Works

1. Subscribes to a camera image topic and a camera info topic
2. Detects ArUco markers using OpenCV's `ArucoDetector`
3. Solves the marker's 6-DoF pose with `cv::solvePnP` using the camera intrinsics and known marker size
4. Publishes the result as a `geometry_msgs/msg/PoseStamped` on `/target_pose`
5. Publishes an annotated image (with drawn axes and XYZ overlay) on `/image_proc`

Once valid camera intrinsics are received, the node unsubscribes from `/camera_info` since the calibration doesn't change mid-flight.

## Parameters

Configured via `cfg/params.yaml` or overridden at launch:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `aruco_id` | `int` | `0` | The ID of the marker to track. All other detected markers are ignored. |
| `dictionary` | `int` | `2` | The OpenCV ArUco dictionary to use. `2` = `DICT_4X4_250`. See [OpenCV ArUco dictionaries](https://docs.opencv.org/4.x/d1/d21/aruco__dictionary_8hpp.html) for the full list of integer codes. |
| `marker_size` | `double` | `0.5` | Physical side length of the marker in **meters**. Must match the real printed marker — an incorrect value will scale the pose estimate proportionally. |

## Topics

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/camera` | `sensor_msgs/msg/Image` | Raw camera image (BGR8) |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsics (focal length, principal point, distortion coefficients) |

### Publications

| Topic | Type | Description |
|---|---|---|
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | 3D pose of the detected marker relative to the camera |
| `/image_proc` | `sensor_msgs/msg/Image` | Annotated image with drawn marker borders, coordinate axes, and XYZ text |

All topics use **Best Effort** QoS with a depth of 1, matching the default for Gazebo camera bridges and most USB camera drivers.

## Understanding the Output Pose

The `/target_pose` message is a `geometry_msgs/msg/PoseStamped` in the **camera optical frame** (`camera_frame`):

```
         Z (forward, into scene)
        /
       /
      /_________ X (right)
      |
      |
      | Y (down)
```

- **position.x** — How far the marker is to the **right** of the camera center (meters)
- **position.y** — How far the marker is **below** the camera center (meters)
- **position.z** — How far the marker is **in front of** the camera (meters, always positive when visible)
- **orientation** — Quaternion representing the marker's rotation relative to the camera

For a **downward-facing camera**, `z` is the altitude above the marker, `x` is the lateral offset, and `y` is the forward/backward offset. The flight modes (PrecisionLand, FrontApproach) convert this camera-frame pose into NED body-frame coordinates to compute setpoints.

## Running

### In Simulation (Gazebo)

The launch files handle Gazebo bridge setup and parameter loading:

```bash
# Downward camera (default ArUco world, PX4 v1.16)
ros2 launch aruco_tracker v1_16_tracker.launch.py

# Front camera
ros2 launch aruco_tracker front_camera_aruco.launch.py

# Downward camera (standalone, for dual-camera setups)
ros2 launch aruco_tracker downward_camera_aruco.launch.py

# Both cameras at once
ros2 launch aruco_tracker dualcam_tracker.launch.py

# Moving platform world
ros2 launch aruco_tracker moving_aruco.launch.py

# PX4 v1.15
ros2 launch aruco_tracker aruco_tracker.launch.py
```

### On Real Hardware

When running with a real camera (e.g. OAK-D USB camera), the camera driver publishes on its own topic names. Use **remappings** to connect them to what the tracker expects:

```bash
ros2 run aruco_tracker aruco_tracker \
  --ros-args \
  -r /camera:=/oak/rgb/image_raw \
  -r /camera_info:=/oak/rgb/camera_info
```

You can also remap the output topics if you're running multiple tracker instances (e.g. one per camera):

```bash
ros2 run aruco_tracker aruco_tracker \
  --ros-args \
  -r /camera:=/front/camera/image_raw \
  -r /camera_info:=/front/camera/camera_info \
  -r /target_pose:=/front/target_pose \
  -r /image_proc:=/front/image_proc \
  -p aruco_id:=1
```

### Verifying Detection

View the annotated image to confirm markers are being detected:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/image_proc` (or `/front/image_proc` if remapped) from the dropdown. You should see green marker outlines, RGB coordinate axes drawn on the marker, and XYZ distance text in the corner.
