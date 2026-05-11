#!/bin/bash
# Source our Environment
source /opt/ros/humble/setup.bash
cd /home/jacob/Jacob_Ladder
source install/setup.bash

# Run the Micro XRCE DDS Agent
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
