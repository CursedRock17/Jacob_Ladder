"""Take off, fly a 1.5 m square, land (the block lands for you when it ends).

The high-level style, like a Crazyflie MotionCommander: relative moves that
wait until the drone gets there.

In the sim:  ./launch_scripts/sim_script.sh flights/square.py SquareFlight
"""

from jl_blocks.fly import Drone

with Drone("SquareFlight", default_height=1.5) as drone:  # takes off here
    for _ in range(4):
        drone.forward(1.5)
        drone.turn_left(90)
    drone.hover(2)
