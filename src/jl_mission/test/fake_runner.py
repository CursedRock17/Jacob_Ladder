#!/usr/bin/env python3
"""Stand-in for the Phase 3 mission_runner, used by sitl_jl_mission.sh.

Waits for /jl/NAME/active, asks for a takeoff, then streams a scripted
sequence of setpoints (seconds after the takeoff reply):
   0-8   hold the takeoff position
   8-...  1 m north of it

What happens after the 1 m step depends on the third argument, `end`
("silence", the default, or "land"):
  silence: 8-16  1 m north
           16-18  an invalid setpoint (infinite x): must be rejected, so
                  jl_mission holds
           18-    nothing at all: jl_mission holds, then lands 5 s after the
                  last valid one
  land:    8-12   1 m north
           12-15  velocity-only, 0.5 m/s east: no position is relayed, so
                  the F1 speed limiter's "last sent" point must not go stale
           15-17  position setpoints frozen at wherever the vehicle was when
                  this phase started: tracking error is watched here and the
                  worst value logged as `MAXERR <value>`, to prove the
                  vehicle is commanded from where it actually is rather than
                  snapping back toward the position held before the
                  velocity-only phase
           17     stop sending setpoints and call /jl/NAME/land

Prints every /jl/NAME/state change, the takeoff reply, and (in the `land`
scenario) the land reply.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from jl_mission_interfaces.srv import Takeoff
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

NAN = float("nan")


class FakeRunner(Node):
    def __init__(self, name, height, end="silence"):
        super().__init__("fake_runner")
        self.height = height
        self.end = end
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(
            TrajectorySetpoint, f"/jl/{name}/setpoint", qos_profile_sensor_data
        )
        self.client = self.create_client(Takeoff, f"/jl/{name}/takeoff")
        self.land_client = self.create_client(Trigger, f"/jl/{name}/land")
        self.create_subscription(Bool, f"/jl/{name}/active", self.on_active, latched)
        self.create_subscription(
            String, f"/jl/{name}/mode_state", self.on_state, latched
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.on_pos,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Vector3Stamped, "/tracking_error", self.on_tracking_error, 10
        )
        self.pos = None
        self.ground = None
        self.base = None
        self.t0 = None
        self.state = None
        self.active = None
        self.asked = False
        self.asked_land = False
        self.asked_land_after_takeover = False
        self.last_z_log = None
        # F1 regression check (land flight only): tracking error watched
        # during the frozen-position phase, and the vehicle position it was
        # frozen at.
        self.tracking_active = False
        self.max_err = None
        self.freeze_pos = None
        self.create_timer(0.02, self.tick)

    def log(self, text):
        print(f"[{time.monotonic():.1f}] {text}", flush=True)

    def on_pos(self, msg):
        self.pos = (msg.x, msg.y, msg.z)
        if self.ground is None:
            self.ground = msg.z

    def on_state(self, msg):
        if msg.data != self.state:
            self.state = msg.data
            self.log(f"STATE {msg.data}")

    def on_active(self, msg):
        was_active = self.active
        if msg.data != self.active:
            self.active = msg.data
            self.log(f"ACTIVE {'true' if msg.data else 'false'}")
        if was_active and not msg.data and not self.asked_land_after_takeover:
            # Went from active to inactive after takeoff (e.g. the pilot took
            # over): the mission must refuse a land request while not in
            # charge (F5). Prove it rather than just asserting silently.
            self.asked_land_after_takeover = True
            self.log("takeover detected: requesting land")
            self.land_client.wait_for_service()
            self.land_client.call_async(Trigger.Request()).add_done_callback(
                self.on_land_after_takeover
            )
        if msg.data and not self.asked:
            self.asked = True
            self.log("active: requesting takeoff")
            req = Takeoff.Request()
            req.height = self.height
            self.client.wait_for_service()
            self.client.call_async(req).add_done_callback(self.on_takeoff)

    def on_takeoff(self, future):
        res = future.result()
        z = self.pos[2] if self.pos else NAN
        ground = self.ground if self.ground is not None else NAN
        self.log(
            f"TAKEOFF success={res.success} message='{res.message}' "
            f"ground_z={ground:.2f} z={z:.2f}"
        )
        if res.success:
            self.base = self.pos
            self.t0 = time.monotonic()

    def on_land(self, future):
        res = future.result()
        self.log(f"LAND success={res.success} message='{res.message}'")

    def on_tracking_error(self, msg):
        if not self.tracking_active:
            return
        err = math.sqrt(msg.vector.x**2 + msg.vector.y**2 + msg.vector.z**2)
        if self.max_err is None or err > self.max_err:
            self.max_err = err

    def on_land_after_takeover(self, future):
        res = future.result()
        self.log(f"LAND_AFTER_TAKEOVER success={res.success} message='{res.message}'")

    def tick(self):
        if self.pos is not None:
            now = time.monotonic()
            if self.last_z_log is None or now - self.last_z_log >= 1.0:
                self.last_z_log = now
                self.log(f"z={self.pos[2]:.2f}")
        if self.t0 is None:
            return
        t = time.monotonic() - self.t0
        if self.end == "land" and t >= 17.0:
            if not self.asked_land:
                self.asked_land = True
                self.tracking_active = False
                if self.max_err is not None:
                    self.log(f"MAXERR {self.max_err:.3f}")
                self.log("requesting land")
                self.land_client.wait_for_service()
                self.land_client.call_async(Trigger.Request()).add_done_callback(
                    self.on_land
                )
            return
        if self.end == "land" and t >= 15.0:
            # Position phase, frozen at wherever the vehicle was when this
            # phase started: proves the F1 speed limiter's "from" point is
            # current after the velocity-only phase below, not stale.
            if self.freeze_pos is None:
                self.freeze_pos = self.pos
                self.tracking_active = True
            msg = TrajectorySetpoint()
            msg.position = list(self.freeze_pos)
            msg.velocity = [NAN, NAN, NAN]
            msg.yaw = NAN
            self.pub.publish(msg)
            return
        if self.end == "land" and t >= 12.0:
            # Velocity-only phase: no position is relayed for 3 s.
            msg = TrajectorySetpoint()
            msg.position = [NAN, NAN, NAN]
            msg.velocity = [0.0, 0.5, 0.0]  # NED: 0.5 m/s east
            msg.yaw = NAN
            self.pub.publish(msg)
            return
        if self.end == "silence" and t >= 18.0:
            return
        msg = TrajectorySetpoint()
        msg.velocity = [NAN, NAN, NAN]
        msg.yaw = NAN
        x, y, z = self.base
        if t < 8.0:
            msg.position = [x, y, z]
        elif self.end == "silence" and t >= 16.0:
            msg.position = [math.inf, y, z]
        else:
            msg.position = [x + 1.0, y, z]
        self.pub.publish(msg)


def main():
    rclpy.init()
    end = sys.argv[3] if len(sys.argv) > 3 else "silence"
    rclpy.spin(FakeRunner(sys.argv[1], float(sys.argv[2]), end))


if __name__ == "__main__":
    main()
