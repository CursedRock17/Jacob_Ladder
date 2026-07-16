# ROS 2 YOLO Image Processing 

This is a ROS2 package in conjunction with Ultralytics YOLO models 
in order to process video stream, identify a KC-130 drogue and
estimate pose of the camera.

## Setup
Use the repo root uv environment; do not create a nested package-local venv.
The YOLO dependencies live in the root `pyproject.toml` and are installed by
default, so a plain `uv sync` is all you need.

```shell
cd /path/to/Jacob_Ladder
uv venv --python 3.10 --system-site-packages .venv
uv sync
source .venv/bin/activate
source /opt/ros/humble/setup.bash
colcon build --packages-select ros2_yolo_image_processing
source install/setup.bash
```


## Running
Source the environment and open 2 windows, in the first:
```shell
ros2 run ros2_yolo_image_processing drogue_detection_node
```
This will create our bounding boxes.
In the second:
```shell
ros2 run ros2_yolo_image_processing pose_estimation_node
```

Or run the launch command:
```shell
ros2 launch ros2_yolo_image_processing auto_nodes.launch.py
```

## Model

`drogue_detection_node` loads the YOLO26n Coupler/Drogue model
(`src/drogue_flight/models/yolov26.pt`, trained at imgsz 416, classes
`{0: Coupler, 1: Drogue}`) by default. Override with the `model_path`
parameter to swap models — the older `best.pt` uses the same class IDs:

```shell
ros2 run ros2_yolo_image_processing drogue_detection_node --ros-args -p model_path:=/path/to/other.pt
```

## Pose estimation

`pose_estimation_node` publishes `PoseStamped` on `tag_dectections` in
**meters**, in a camera-aligned frame: **+x left, +y up, +z forward**.

Parameters:

| Parameter | Default | Description |
| --- | --- | --- |
| `fx`, `fy`, `cx`, `cy` | DFK 23UM021 calibration | Camera intrinsics; automatically overridden by the first `camera_info` message, so remap `camera_info` to your camera driver's topic and these defaults are moot |
| `use_attitude_correction` | `false` | Subscribe to `/fmu/out/vehicle_attitude` (px4_msgs) and rotate the estimate into a level, north-referenced frame (same +x left/+y up/+z forward layout) |
