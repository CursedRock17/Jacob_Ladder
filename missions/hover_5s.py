"""Take off, hover for 5 seconds by sending the position yourself, land.

The low-level style: `every` runs the loop body 50 times a second, and between
iterations the last setpoint keeps being sent, so the drone holds.
"""

from jl_flight import mission


@mission("HoverTest")
def hover_5s(drone):
    drone.arm()
    drone.take_off(1.5)  # PX4's own takeoff: position setpoints alone cannot lift off
    hover = drone.position  # where we are now, 1.5 m up
    for _t in drone.every(5):
        drone.send_position(hover)
    drone.land()
