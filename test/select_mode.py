"""Select an external mode by slot, the way a ground station does: `select_mode.py ext2`.

PX4's shell (`commander mode`) only knows ext1. A mission is not always in
slot 1 (see jl_mission's main.cpp), so the SITL harnesses select through
VEHICLE_CMD_SET_NAV_STATE on /fmu/in/vehicle_command instead.
"""

from __future__ import annotations

import sys
import time

import rclpy
from px4_msgs.msg import VehicleCommand, VehicleStatus
from rclpy.qos import qos_profile_sensor_data

slot = sys.argv[1]
if not slot.startswith("ext") or not slot[3:].isdigit() or not 1 <= int(slot[3:]) <= 8:
    sys.exit(f"select_mode: expected ext1 .. ext8, got {slot!r}")

rclpy.init()
node = rclpy.create_node("select_mode")
pub = node.create_publisher(
    VehicleCommand, "/fmu/in/vehicle_command", qos_profile_sensor_data
)
cmd = VehicleCommand()
cmd.command = VehicleCommand.VEHICLE_CMD_SET_NAV_STATE
cmd.param1 = float(VehicleStatus.NAVIGATION_STATE_EXTERNAL1 + int(slot[3:]) - 1)
cmd.target_system = 1
cmd.target_component = 1
cmd.source_system = 1
cmd.source_component = 1
cmd.from_external = True
# Send it once: selecting the mode again mid-takeoff restarts the executor.
# Wait for the agent to match first, so the one message is not lost.
deadline = time.monotonic() + 5.0
while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    time.sleep(0.05)
time.sleep(0.2)
cmd.timestamp = 0  # PX4 stamps it
pub.publish(cmd)
time.sleep(0.5)  # let it go out before the node closes
rclpy.shutdown()
