"""Arm, take off, hover for 5 seconds, land.

The low-level style: you send the position yourself, every step.

In the sim:  ./launch_scripts/sim_script.sh flights/hover_5s.py HoverTest
"""

from jl_blocks.fly import Drone

with Drone("HoverTest", take_off_on_enter=False) as drone:
    drone.arm()
    drone.take_off(1.5)  # PX4's own takeoff: position setpoints alone cannot lift off
    hover = drone.position  # where we are now, 1.5 m up
    for t in drone.every(5):  # runs the body 50 times a second for 5 s
        drone.send_position(hover)
    drone.land()
