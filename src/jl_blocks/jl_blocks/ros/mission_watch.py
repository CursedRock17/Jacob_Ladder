"""mission_watch: check a mission flown in SITL (used by test/sitl_mission.sh).

Listens to the runner's /jl/NAME/events and, if asked, to where a camera sees
the target. Exits 0 as soon as every check passes, 1 on an abort or a timeout.
The checks themselves are jl_blocks.testing.expect, tested without ROS.
"""

from __future__ import annotations

import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from ..testing.expect import BandHold, StepOrder, feed_gated, report

# Numbers may be written as 5 or 5.0 on the command line.
ANY_NUMBER = ParameterDescriptor(dynamic_typing=True)


class MissionWatch(Node):
    def __init__(self) -> None:
        super().__init__("mission_watch")
        name = str(self.declare_parameter("mission_name", "").value)
        expect = str(self.declare_parameter("expect", "").value).split()
        self.need_finished = bool(self.declare_parameter("finished", False).value)
        camera_topic = str(
            self.declare_parameter("camera_topic", "/front/target_pose").value
        )
        target = str(self.declare_parameter("camera_target", "").value).split()
        tolerance = float(self.declare_parameter("tolerance", 0.25, ANY_NUMBER).value)
        hold_s = float(self.declare_parameter("hold_s", 5.0, ANY_NUMBER).value)
        self.timeout_s = float(
            self.declare_parameter("timeout_s", 120.0, ANY_NUMBER).value
        )

        self.order = StepOrder(expect)
        self.band: BandHold | None = None
        if target:
            x, y, z = (float(v) for v in target)
            self.band = BandHold((x, y, z), tolerance, hold_s)
            self.create_subscription(
                PoseStamped, camera_topic, self.on_pose, qos_profile_sensor_data
            )
        self.create_subscription(String, f"/jl/{name}/events", self.on_event, 10)
        self.start = time.monotonic()
        self.result: int | None = None
        self.create_timer(0.1, self.check)

    def on_event(self, msg: String) -> None:
        print(f"event: {msg.data}", flush=True)
        self.order.feed(msg.data)

    def on_pose(self, msg: PoseStamped) -> None:
        assert self.band is not None
        p = msg.pose.position
        feed_gated(
            self.order,
            self.band,
            (float(p.x), float(p.y), float(p.z)),
            time.monotonic(),
        )

    def check(self) -> None:
        if self.result is not None:
            return
        passed = (
            self.order.ok
            and (self.order.finished or not self.need_finished)
            and (self.band is None or self.band.held)
        )
        timed_out = time.monotonic() - self.start > self.timeout_s
        if not (passed or timed_out or self.order.over):
            return
        lines, ok = report(self.order, self.need_finished, self.band)
        if timed_out and not ok:
            lines.append(f"FAIL all checks passed within {self.timeout_s:g} s")
        for line in lines:
            print(line, flush=True)
        self.result = 0 if ok else 1


def main() -> None:
    rclpy.init()
    node = MissionWatch()
    try:
        while rclpy.ok() and node.result is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.result = 1
    finally:
        result = 1 if node.result is None else node.result
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(result)


if __name__ == "__main__":
    main()
