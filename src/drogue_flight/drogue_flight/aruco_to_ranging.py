#!/usr/bin/env python3
"""SITL-only: feed ArUco detections to DroneSmoothPlanner in place of the YOLO drogue.

aruco_tracker publishes the tag in OpenCV's optical frame (+x right, +y down,
+z forward). The planner expects the YOLO pose node's ranging frame (+x left,
+y up, +z forward) on /tag_detections, so flip x and y and republish. The
planner itself runs unmodified, exactly as it flies on the real drone.
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def optical_to_ranging(x, y, z):
    return -x, -y, z


class ArucoToRanging(Node):
    def __init__(self):
        super().__init__("aruco_to_ranging")
        self._pub = self.create_publisher(PoseStamped, "/tag_detections", qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, "/front/target_pose", self._on_pose, qos_profile_sensor_data
        )

    def _on_pose(self, msg):
        p = msg.pose.position
        p.x, p.y, p.z = optical_to_ranging(p.x, p.y, p.z)
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ArucoToRanging())


if __name__ == "__main__":
    main()
