"""Close to 3 m behind the refuelling drogue and hold station there.

The Python version of drogue_flight's DroneSmoothPlanner. The drogue is assumed
to be in view of the front camera after takeoff. The drone holds 3 m behind it,
facing it and following it as it moves, until the pilot takes over. If YOLO
loses the drogue for 3 s, it lands. The camera's pitch and offset come from
the airframe (config/flight.yaml).
"""

from jl_flight import TargetLost, YoloDrogue, mission

DROGUE = YoloDrogue(camera="front")


@mission("DrogueApproach", needs=[DROGUE])
def drogue_approach(drone):
    drone.take_off(1.75)
    try:
        drone.follow(DROGUE, standoff=3.0)
    except TargetLost:
        drone.say("lost the drogue: landing")
        drone.land()
