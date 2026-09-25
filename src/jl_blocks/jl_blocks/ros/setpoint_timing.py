"""setpoint_timing: repeat spec section 6's timing measurement on this machine.

Listens to /jl/NAME/setpoint (the runner's output) for --seconds and prints the
inter-arrival p50 / p99 / max. Run it on the Jetson during a flight with VIO
and YOLO up, and compare with the spec's desktop numbers (p99 22.8 ms, max 27.6 ms).
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ..timing import summary


def main() -> None:
    parser = argparse.ArgumentParser(prog="setpoint_timing")
    parser.add_argument("--name", required=True, help="the mission name (QGC name)")
    parser.add_argument("--seconds", type=float, default=60.0)
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    rclpy.init()
    node = Node("setpoint_timing")
    stamps: list[float] = []
    node.create_subscription(
        TrajectorySetpoint,
        f"/jl/{args.name}/setpoint",
        lambda _msg: stamps.append(time.monotonic()),
        qos_profile_sensor_data,
    )
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    s = summary(stamps)
    print(
        f"setpoint inter-arrival over {int(s['count'])} messages: "
        f"p50 {s['p50_ms']:g} ms, p99 {s['p99_ms']:g} ms, max {s['max_ms']:g} ms"
    )
    sys.exit(0 if s["count"] >= 10 else 1)


if __name__ == "__main__":
    main()
