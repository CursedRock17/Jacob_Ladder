"""The smallest mission: take off, hover for 5 s, land."""

from jl_flight import mission


@mission("TakeoffHoldLand")
def takeoff_hold_land(drone):
    drone.take_off(1.5)
    drone.hover(5)
    drone.land()
