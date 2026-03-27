# ROS 2 YOLO Image Processing 

This is a ROS2 package in conjunction with Ultralytics YOLO models 
in order to process video stream, identify a KC-130 drogue and
estimate pose of the camera.

## Setup
This is a python package within a ROS 2 package, meaning we need 
a virtual environment to process any python specific packages

1) Add this package to your local ROS 2 workspace
2) Enter the head of this package: `cd ~/ros2_ws/ros2_yolo_image_processing`
3) Create a virtual environment: `virtualenv -p python3 ./venv`
4) Source the virtual environment: `source ./venv/bin/activate`
5) Add a directive such that the colcon buildchain ignores the directory:
`touch ./venv/COLCON_IGNORE`
6) Use pip to locally install the necessary packages:
`python3 -m pip install -r requirements.txt`
7) Source your ROS environment and build:
`source /opt/ros/{ROS-DISTRO}/setup.bash && colcon build && source install/setup.bash`


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
