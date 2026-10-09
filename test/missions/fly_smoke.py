"""Flown by test/sitl_all.sh: the Drone's commands against real PX4 SITL. An
assertion that fails ends the mission with an error (and it still lands), which
sitl_flight.sh reports as a failure."""

import math

from jl_flight import mission


def near(actual: float, expected: float, tolerance: float, what: str) -> None:
    assert abs(actual - expected) <= tolerance, (
        f"{what}: {actual:.2f}, wanted {expected}"
    )
    print(f"ok {what}: {actual:.2f}", flush=True)


@mission("FlySmoke")
def fly_smoke(drone):
    drone.arm()
    start = drone.position
    for _ in drone.every(2):  # armed, on the ground: no lift-off from setpoints
        drone.send_position(start.with_up(start.up + 1.5))
    near(drone.position.up - start.up, 0.0, 0.3, "armed and still on the ground")

    drone.take_off(1.5)  # after arm(): PX4's own takeoff
    near(drone.position.up - start.up, 1.5, 0.3, "took off to 1.5 m")
    hover = drone.position
    for _ in drone.every(3):  # the low-level loop holds the height
        drone.send_position(hover)
    near(drone.position.up - start.up, 1.5, 0.3, "hovered by sending positions")

    # forward / turn are relative to the heading, which depends on how the model
    # spawned (Gazebo's yaw 0 faces east), so work out where "forward" points.
    h0 = math.radians(drone.heading)
    print(f"heading at start: {drone.heading:.0f} degrees", flush=True)
    before = drone.position
    drone.forward(1.0)
    after = drone.position
    near(after.north - before.north, math.cos(h0), 0.3, "forward 1 m: north part")
    near(after.east - before.east, math.sin(h0), 0.3, "forward 1 m: east part")

    drone.turn_right(90)
    near(
        math.remainder(drone.heading - math.degrees(h0), 360),
        90.0,
        10.0,
        "turned right 90 degrees",
    )
    before = drone.position
    drone.forward(1.0)  # facing 90 degrees further round now
    after = drone.position
    near(
        after.north - before.north,
        math.cos(h0 + math.pi / 2),
        0.3,
        "forward after turning: north part",
    )
    near(
        after.east - before.east,
        math.sin(h0 + math.pi / 2),
        0.3,
        "forward after turning: east part",
    )

    drone.up(0.5)
    near(drone.position.up - start.up, 2.0, 0.3, "up 0.5 m")
    drone.land()
    print("fly_smoke done", flush=True)
