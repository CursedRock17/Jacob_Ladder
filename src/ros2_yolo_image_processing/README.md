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
