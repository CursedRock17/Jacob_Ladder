"""Keep 2 m in front of an ArUco tag as it moves, steering with a PID controller
(FrontApproach's) instead of PX4's own position control.

The tag is assumed to be in view of the front camera after takeoff. If it is
lost for 3 s, the drone lands.
"""

from jl_flight import ArucoTag, TargetLost, mission
from jl_flight.controllers import PID

TAG = ArucoTag(id=0, size=0.15, dictionary_name="4x4_250", camera="front")


@mission("TrackMovingAruco", needs=[TAG])
def track_moving_aruco(drone):
    drone.take_off(1.5)
    try:
        drone.follow(TAG, standoff=2.0, controller=PID(kp=0.8, kd=0.2, max_speed=1.0))
    except TargetLost:
        drone.say("lost the tag: landing")
        drone.land()
