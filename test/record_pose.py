"""Record PX4's local position to a CSV, to see where a SITL flight really went.

    record_pose.py OUT.csv [TAG_TOPIC]

Columns: seconds since start, north, east, down (metres from where the drone
spawned), heading in degrees, and the latest reading on TAG_TOPIC (default
/front/target_pose; empty if none arrived in the last 0.3 s): the detection's
x, y, z in metres, in the detector's own frame. 5 Hz. Started by test/sitl_flight.sh; stops on SIGTERM.
"""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data

rclpy.init()
node = rclpy.create_node("record_pose")
latest: list[VehicleLocalPosition] = []  # holds the newest message only


def remember(msg: VehicleLocalPosition) -> None:
    latest[:] = [msg]


node.create_subscription(
    VehicleLocalPosition,
    "/fmu/out/vehicle_local_position_v1",
    remember,
    qos_profile_sensor_data,
)
tag: list[tuple[float, PoseStamped]] = []  # (when it arrived, the reading)


def remember_tag(msg: PoseStamped) -> None:
    tag[:] = [(time.monotonic(), msg)]


TAG_TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/front/target_pose"
node.create_subscription(PoseStamped, TAG_TOPIC, remember_tag, qos_profile_sensor_data)
start = time.monotonic()
with open(sys.argv[1], "w") as out:
    out.write("t,north,east,down,heading_deg,tag_x,tag_y,tag_z\n")
    next_write = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic() - start
        if latest and now >= next_write:
            p = latest[-1]
            seen = ",,"  # no reading in the last 0.3 s
            if tag and time.monotonic() - tag[0][0] < 0.3:
                q = tag[0][1].pose.position
                seen = f"{q.x:.2f},{q.y:.2f},{q.z:.2f}"
            out.write(
                f"{now:.1f},{p.x:.2f},{p.y:.2f},{p.z:.2f},"
                f"{math.degrees(p.heading):.0f},{seen}\n"
            )
            out.flush()
            next_write = now + 0.2
