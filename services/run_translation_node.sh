#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jacob/Jacob_Ladder/install/setup.bash
#export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH

# Run the ROS2 usb_cam node with specified parameters
ros2 run translation_node translation_node_bin
