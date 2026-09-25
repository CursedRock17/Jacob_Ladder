#!/usr/bin/env python3
"""Stand-in for a pilot's RC/joystick, used only by the takeover flight in
sitl_jl_mission.sh.

Headless SITL has no RC or joystick attached, so PX4's own posctl arming
check (manual_control_signal_lost) never clears and `commander mode posctl`
is silently denied (no log line: PX4 only prints the rejection for MAVLink
Mission Planner callers). This publishes a constant, valid
ManualControlSetpoint on /fmu/in/manual_control_input so PX4 sees a live
manual control source, exactly as a real RC/joystick would, letting the
takeover flight prove a real posctl switch rather than one that silently no-ops.

The timestamp fields must be wall-clock microseconds (not zero, not a ROS
monotonic clock): PX4's uXRCE-DDS time sync converts an incoming DDS
timestamp to its own hrt clock using the wall-clock offset it negotiated
with the agent, and a stale/zero timestamp is dropped as too old.
"""

import time

import rclpy
from px4_msgs.msg import ManualControlSetpoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class FakePilot(Node):
    def __init__(self):
        super().__init__("fake_pilot")
        self.pub = self.create_publisher(
            ManualControlSetpoint,
            "/fmu/in/manual_control_input",
            qos_profile_sensor_data,
        )
        self.create_timer(0.05, self.tick)

    def tick(self):
        msg = ManualControlSetpoint()
        now_us = int(time.time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.valid = True
        msg.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
        msg.roll = 0.0
        msg.pitch = 0.0
        msg.yaw = 0.0
        msg.throttle = 0.0
        msg.flaps = 0.0
        msg.sticks_moving = True
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(FakePilot())


if __name__ == "__main__":
    main()
