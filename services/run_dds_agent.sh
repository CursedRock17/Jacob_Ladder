#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/jacob/Jacob_Ladder/install/setup.bash
#export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH

# Run the MicroDDS Agent
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
