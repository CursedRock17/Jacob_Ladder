#!/bin/bash
# Source our Environment
source /opt/ros/humble/setup.bash
cd /home/jacob/Jacob_Ladder
source install/setup.bash
source ./src/oak_d_visual_odometry/venv/bin/activate

# Run the ROS2 usb_cam node with specified parameters
ros2 run oak_d_visual_odometry vo_publisher_px4_node
