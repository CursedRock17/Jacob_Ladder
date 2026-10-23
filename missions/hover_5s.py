"""Take off, hover for 5 seconds by sending the position yourself, land.

The low-level style: `every` runs the loop body 50 times a second, and between
iterations the last setpoint keeps being sent, so the drone holds.
"""

from jl_flight import mission


@mission("HoverTest")
def hover_5s(drone):
    desired_z = 1.5
    # drone.take_off(1.)
    drone.arm()
    hover = drone.position  # where we are now, 1.5 m up
    for _t in drone.every(5):
        desired_position = [hover[0], hover[1], hover[2] + desired_z]
        drone.send_position(desired_position)
        print(f"Current: {drone.position}, Desired: {desired_position}")
    drone.land()
