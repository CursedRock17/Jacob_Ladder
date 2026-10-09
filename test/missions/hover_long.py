"""Take off and hover for a minute. test/sitl_mission_dies.sh kills it mid-hover."""

from jl_flight import mission


@mission("MissionDies")
def hover_long(drone):
    drone.take_off(1.5)
    drone.hover(60)
    drone.land()
