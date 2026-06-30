#!/bin/bash
# Source our Environment
source /opt/ros/humble/setup.bash
cd /home/oem/ARK/tracktor-beam
source install/setup.bash

# Source our Environment
ros2 run aruco_tracker aruco_tracker
