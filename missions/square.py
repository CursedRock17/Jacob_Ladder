"""Take off, fly a 1.5 m square, land. Relative moves that wait until the drone
gets there, like a Crazyflie MotionCommander program."""

from jl_flight import mission


@mission("SquareFlight")
def square(drone):
    drone.take_off(1.5)
    for _ in range(4):
        drone.forward(1.5)
        drone.turn_left(90)
    drone.hover(2)
    drone.land()
