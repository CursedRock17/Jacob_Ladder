"""mission_runner: flies one mission file through jl_mission.

A thin ROS adapter around jl_blocks.core.Session. It turns PX4 topics into a
VehicleState, feeds camera detections to the mission's targets, publishes the
session's setpoints on /jl/NAME/setpoint, and calls /jl/NAME/takeoff and
/jl/NAME/land when a block asks. Every decision lives in Session, which is
tested without ROS. Whenever something goes wrong here, the runner goes quiet
and jl_mission's watchdog holds, then lands.
"""

from __future__ import annotations

import sys
import traceback

import rclpy
from geometry_msgs.msg import PoseStamped
from jl_mission_interfaces.srv import Takeoff
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLandDetected,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .. import library  # noqa: F401  (registers the shipped blocks)
from ..core import (
    MissionError,
    Request,
    Session,
    Setpoint,
    VehicleState,
    load_block_files,
    load_mission,
    trajectory_fields,
)

NAN = float("nan")
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MissionRunner(Node):
    def __init__(self) -> None:
        super().__init__("mission_runner")
        mission_file = self.declare_parameter("mission_file", "").value
        blocks = self.declare_parameter("blocks", "").value
        rate_hz = self.declare_parameter("rate_hz", 50.0).value
        # translation_node (running in every SITL harness and at boot on the real
        # drone) republishes PX4's vehicle_local_position in this workspace's v1
        # message layout on this topic, so the runner keeps working even against
        # firmware whose native vehicle_local_position is still the older v0 layout.
        local_topic = self.declare_parameter(
            "local_position_topic", "/fmu/out/vehicle_local_position_v1"
        ).value
        attitude_topic = self.declare_parameter(
            "attitude_topic", "/fmu/out/vehicle_attitude"
        ).value
        land_topic = self.declare_parameter(
            "land_detected_topic", "/fmu/out/vehicle_land_detected"
        ).value

        errors = load_block_files(
            [p.strip() for p in str(blocks).split(",") if p.strip()]
        )
        if errors:
            raise MissionError(errors)
        if not mission_file:
            raise MissionError(["mission_file parameter is required"])
        spec = load_mission(str(mission_file))
        self.session = Session(spec)
        self.name = spec.name
        prefix = f"/jl/{self.name}"

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, f"{prefix}/setpoint", qos_profile_sensor_data
        )
        self.state_pub = self.create_publisher(String, f"{prefix}/state", LATCHED)
        self.events_pub = self.create_publisher(String, f"{prefix}/events", 10)
        self.executor_clients = {
            "takeoff": self.create_client(Takeoff, f"{prefix}/takeoff"),
            "land": self.create_client(Trigger, f"{prefix}/land"),
        }
        self.create_subscription(Bool, f"{prefix}/active", self.on_active, LATCHED)
        self.create_subscription(
            VehicleLocalPosition, local_topic, self.on_local, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleAttitude, attitude_topic, self.on_attitude, qos_profile_sensor_data
        )
        self.create_subscription(
            VehicleLandDetected, land_topic, self.on_land, qos_profile_sensor_data
        )
        for topic in self.session.topics():
            self.create_subscription(
                PoseStamped,
                topic,
                lambda msg, t=topic: self.on_detection(t, msg),
                qos_profile_sensor_data,
            )

        self._local: VehicleLocalPosition | None = None
        self._attitude: VehicleAttitude | None = None
        self._landed: bool | None = None  # None until the first message
        self._local_at: float | None = None
        self._attitude_at: float | None = None
        self._started = self.now()
        self._warned = False
        self._last_tick: float | None = None
        self._last_state: str | None = None
        self._tick_failing = False
        self.create_timer(1.0 / float(rate_hz), self.tick)
        self.publish_state()
        self.get_logger().info(
            f"flying {mission_file} as '{self.name}'; waiting for {prefix}/active"
        )

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ── inputs ──

    def on_active(self, msg: Bool) -> None:
        self.session.set_active(msg.data, self.now())
        self.flush()

    def on_local(self, msg: VehicleLocalPosition) -> None:
        self._local = msg
        self._local_at = self.now()
        self.update_vehicle()

    def on_attitude(self, msg: VehicleAttitude) -> None:
        self._attitude = msg
        self._attitude_at = self.now()
        self.update_vehicle()

    def on_land(self, msg: VehicleLandDetected) -> None:
        # Only updates the landed flag: land_detected must never refresh the
        # VehicleState's stamp, or a live land_detected topic could mask a
        # stalled local_position/attitude and hide staleness from Session.
        self._landed = bool(msg.landed)
        self.update_vehicle()

    def update_vehicle(self) -> None:
        local, attitude = self._local, self._attitude
        if local is None or attitude is None:
            return
        q = attitude.q
        self.session.update_vehicle(
            VehicleState(
                position_ned=(float(local.x), float(local.y), float(local.z)),
                velocity_ned=(float(local.vx), float(local.vy), float(local.vz)),
                yaw=float(local.heading),
                attitude=(float(q[0]), float(q[1]), float(q[2]), float(q[3])),
                # Until PX4 says otherwise, assume on the ground: send nothing.
                landed=self._landed is not False,
                # The older of the two inputs' receive times: if either one
                # stops arriving (e.g. translation_node dies, EKF stalls) the
                # stamp goes stale even though the other keeps updating, so
                # Session's 0.5 s staleness guard still fires.
                stamp=min(self._local_at, self._attitude_at),
            )
        )

    def on_detection(self, topic: str, msg: PoseStamped) -> None:
        p = msg.pose.position
        self.session.observe(topic, (float(p.x), float(p.y), float(p.z)))

    # ── the control loop ──

    def tick(self) -> None:
        now = self.now()
        dt = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        self.warn_if_vehicle_topics_missing(now)
        try:
            out = self.session.tick(now, dt)
        except Exception as exc:
            # Publish nothing: jl_mission holds, then lands on silence. Log
            # the full traceback once; if the same tick keeps failing (it can
            # fire dozens of times a second), throttle to a one-liner so the
            # log isn't flooded.
            if not self._tick_failing:
                self._tick_failing = True
                self.get_logger().error(traceback.format_exc())
            else:
                self.get_logger().error(
                    f"tick still failing: {exc}", throttle_duration_sec=1.0
                )
            return
        self._tick_failing = False
        if out.setpoint is not None:
            self.publish_setpoint(out.setpoint)
        for request in out.requests:
            self.send(request)
        self.flush()

    def publish_setpoint(self, setpoint: Setpoint) -> None:
        position, velocity, yaw = trajectory_fields(setpoint)
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = [NAN, NAN, NAN]
        msg.jerk = [NAN, NAN, NAN]
        msg.yaw = yaw
        msg.yawspeed = NAN
        self.setpoint_pub.publish(msg)

    def send(self, request: Request) -> None:
        kind = request.action.kind
        client = self.executor_clients[kind]
        if not client.service_is_ready():
            self.session.action_done(
                request.token,
                False,
                f"/jl/{self.name}/{kind} is not available (is jl_mission running?)",
            )
            return
        if kind == "takeoff":
            call = Takeoff.Request()
            call.height = float(request.action.height)
        else:
            call = Trigger.Request()
        future = client.call_async(call)
        future.add_done_callback(lambda f, token=request.token: self.on_reply(token, f))

    def on_reply(self, token: int, future) -> None:
        error = future.exception()
        if error is not None:
            self.session.action_done(token, False, f"service call failed: {error}")
        else:
            reply = future.result()
            self.session.action_done(token, bool(reply.success), str(reply.message))
        self.flush()

    # ── outputs for people ──

    def flush(self) -> None:
        for event in self.session.drain_events():
            self.get_logger().info(f"event: {event}")
            self.events_pub.publish(String(data=event))
        for error in self.session.drain_errors():
            self.get_logger().error(error)
        self.publish_state()

    def publish_state(self) -> None:
        state = self.session.state
        if state != self._last_state:
            self._last_state = state
            self.state_pub.publish(String(data=state))

    def warn_if_vehicle_topics_missing(self, now: float) -> None:
        if self._warned or now - self._started < 5.0:
            return
        missing = [
            name
            for name, value in (
                ("local_position_topic", self._local),
                ("attitude_topic", self._attitude),
                ("land_detected_topic", self._landed),
            )
            if value is None
        ]
        if missing:
            self._warned = True
            self.get_logger().warn(
                "no data yet on " + ", ".join(missing) + ": the mission sends "
                "nothing until they arrive (check the topic names)"
            )


def main() -> None:
    rclpy.init()
    try:
        node = MissionRunner()
    except MissionError as err:
        for line in err.errors:
            print(f"mission_runner: {line}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    except OSError as err:
        print(f"mission_runner: cannot read the mission file: {err}", file=sys.stderr)
        rclpy.try_shutdown()
        sys.exit(1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
