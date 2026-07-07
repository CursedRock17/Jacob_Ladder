#!/bin/bash
# Source our Environment
source /opt/ros/humble/setup.bash
cd /home/usmsm/Jacob_Ladder
source install/setup.bash

# Run the ROS 2 PX4 translation_node
ros2 run translation_node translation_node_bin
