"""Close to 3 m behind the refuelling drogue, then land there.

The Python version of drogue_flight's DroneSmoothPlanner. The drogue is assumed
to be in view of the front camera after takeoff. The drone flies to 3 m behind
it, facing it, and lands once it has settled within 0.1 m of that point (on
every axis, nearly still for half a second). If YOLO loses the drogue for 3 s,
or it has not settled after 60 s, it lands where it is. The camera's pitch and
offset come from the airframe (config/flight.yaml).
"""

from jl_flight import TargetLost, Timeout, YoloDrogue, mission

DROGUE = YoloDrogue(camera="front")


@mission("DrogueApproach", needs=[DROGUE])
def drogue_approach(drone):
    drone.take_off(0.5)
    try:
        drone.approach(DROGUE, standoff=3.0, tolerance=0.1, timeout=60)
        drone.say("at the standoff: landing")
    except TargetLost:
        drone.say("lost the drogue: landing")
    except Timeout:
        drone.say("did not settle within 0.1 m in 60 s: landing")
    drone.land()
