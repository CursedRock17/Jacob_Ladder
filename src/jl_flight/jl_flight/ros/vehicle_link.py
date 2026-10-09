"""RosBackend: connects a jl_flight.Drone to jl_mission over ROS 2.

The mission's process is the runner: it reads PX4's position, publishes
/jl/NAME/setpoint, and calls /jl/NAME/arm, /takeoff and /land. It also reads
detections from perception helpers and reports what it is doing on
/jl/NAME/events. Run only one process per mission name. rclpy spins in a
background thread, so the mission's own code can block.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import rclpy
from geometry_msgs.msg import PoseStamped
from jl_mission_interfaces.srv import Takeoff
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLandDetected,
    VehicleLocalPosition,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from ..core import Setpoint, Vector3, VehicleState

NAN = float("nan")


def trajectory_fields(setpoint: Setpoint) -> tuple[list[float], list[float], float]:
    """Position, velocity and yaw for px4_msgs/TrajectorySetpoint (NaN = not controlled)."""
    position = list(setpoint.position) if setpoint.position is not None else [NAN] * 3
    velocity = list(setpoint.velocity) if setpoint.velocity is not None else [NAN] * 3
    yaw = setpoint.yaw if setpoint.yaw is not None else NAN
    return position, velocity, yaw


LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
STALE_S = 0.5  # PX4 data older than this counts as missing
SERVICE_WAIT_S = 5.0
ARM_TIMEOUT_S = 15.0
TAKEOFF_TIMEOUT_S = 60.0  # jl_mission gives up on its own after 30 s
LAND_TIMEOUT_S = 180.0


class RosBackend:
    def __init__(
        self,
        name: str,
        local_position_topic: str = "/fmu/out/vehicle_local_position_v1",
        attitude_topic: str = "/fmu/out/vehicle_attitude",
        land_detected_topic: str = "/fmu/out/vehicle_land_detected",
    ) -> None:
        self._name = name
        self._topics = (local_position_topic, attitude_topic, land_detected_topic)
        self._lock = threading.Lock()
        self._local: VehicleLocalPosition | None = None
        self._attitude: VehicleAttitude | None = None
        self._landed: bool | None = None
        self._local_at = 0.0
        self._attitude_at = 0.0
        self._active = False
        self._selections = 0
        self._owns_rclpy = False
        self._thread: threading.Thread | None = None

    # ── Backend ──

    def connect(self) -> None:
        if not rclpy.ok():
            rclpy.init()
            self._owns_rclpy = True
        prefix = f"/jl/{self._name}"
        local, attitude, land = self._topics
        self._node = Node(f"jl_fly_{self._name.lower()}")
        self._publisher = self._node.create_publisher(
            TrajectorySetpoint, f"{prefix}/setpoint", qos_profile_sensor_data
        )
        self._events = self._node.create_publisher(String, f"{prefix}/events", 10)
        self._clients = {
            "arm": self._node.create_client(Trigger, f"{prefix}/arm"),
            "takeoff": self._node.create_client(Takeoff, f"{prefix}/takeoff"),
            "land": self._node.create_client(Trigger, f"{prefix}/land"),
        }
        self._node.create_subscription(
            Bool, f"{prefix}/active", self._on_active, LATCHED
        )
        self._node.create_subscription(
            VehicleLocalPosition, local, self._on_local, qos_profile_sensor_data
        )
        self._node.create_subscription(
            VehicleAttitude, attitude, self._on_attitude, qos_profile_sensor_data
        )
        self._node.create_subscription(
            VehicleLandDetected, land, self._on_land, qos_profile_sensor_data
        )
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="jl_fly_rclpy"
        )
        self._thread.start()

    def close(self) -> None:
        if self._thread is None:
            return
        self._executor.shutdown()
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
        if self._owns_rclpy:
            rclpy.try_shutdown()
        self._thread = None

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(max(seconds, 0.0))

    def vehicle(self) -> VehicleState | None:
        with self._lock:
            local, attitude, landed = self._local, self._attitude, self._landed
            stamp = min(self._local_at, self._attitude_at)
        if local is None or attitude is None or time.monotonic() - stamp > STALE_S:
            return None
        q = attitude.q
        return VehicleState(
            position_ned=(float(local.x), float(local.y), float(local.z)),
            velocity_ned=(float(local.vx), float(local.vy), float(local.vz)),
            yaw=float(local.heading),
            attitude=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
            # Until PX4 says otherwise, assume on the ground.
            landed=landed is not False,
            stamp=stamp,
        )

    def is_active(self) -> bool:
        with self._lock:
            return self._active

    def selections(self) -> int:
        with self._lock:
            return self._selections

    def subscribe_pose(self, topic: str, callback: Callable[[Vector3], None]) -> None:
        def on_pose(msg: PoseStamped) -> None:
            p = msg.pose.position
            callback((float(p.x), float(p.y), float(p.z)))

        self._node.create_subscription(
            PoseStamped, topic, on_pose, qos_profile_sensor_data
        )

    def event(self, text: str) -> None:
        self._node.get_logger().info(f"event: {text}")
        self._events.publish(String(data=text))

    def publish(self, setpoint: Setpoint) -> None:
        position, velocity, yaw = trajectory_fields(setpoint)
        msg = TrajectorySetpoint()
        msg.timestamp = int(self._node.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = yaw
        msg.yawspeed = NAN
        self._publisher.publish(msg)

    def arm(self) -> tuple[bool, str]:
        return self._call("arm", Trigger.Request(), ARM_TIMEOUT_S)

    def takeoff(self, height: float) -> tuple[bool, str]:
        request = Takeoff.Request()
        request.height = float(height)
        return self._call("takeoff", request, TAKEOFF_TIMEOUT_S)

    def land(self) -> tuple[bool, str]:
        return self._call("land", Trigger.Request(), LAND_TIMEOUT_S)

    # ── inside ──

    def _call(self, kind: str, request, timeout: float) -> tuple[bool, str]:
        client = self._clients[kind]
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False, (
                f"/jl/{self._name}/{kind} is not available (is jl_mission running "
                f"with mission_name:={self._name}, and is it built with the arm service?)"
            )
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            return False, f"no reply to {kind} after {timeout:g} s"
        error = future.exception()
        if error is not None:
            return False, f"service call failed: {error}"
        reply = future.result()
        return bool(reply.success), str(reply.message)

    def _on_active(self, msg: Bool) -> None:
        with self._lock:
            self._active = bool(msg.data)
            if msg.data:
                self._selections += 1

    def _on_local(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._local = msg
            self._local_at = time.monotonic()

    def _on_attitude(self, msg: VehicleAttitude) -> None:
        with self._lock:
            self._attitude = msg
            self._attitude_at = time.monotonic()

    def _on_land(self, msg: VehicleLandDetected) -> None:
        # Only the landed flag: it must not make stale position data look fresh.
        with self._lock:
            self._landed = bool(msg.landed)
