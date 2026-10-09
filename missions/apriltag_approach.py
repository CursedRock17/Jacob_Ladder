"""The drogue approach with an AprilTag standing in for the drogue.

Flies in the simulator's apriltag_wall world, where YOLO has no drogue to see.
The tag is assumed to be in view of the front camera after takeoff. The drone
holds 3 m in front of it, facing it, until the pilot takes over. If the tag is
lost for 3 s, it lands.
"""

from jl_flight import AprilTag, TargetLost, mission

TAG = AprilTag(id=0, size=0.30, family="36h11")


@mission("AprilTagApproach", needs=[TAG])
def apriltag_approach(drone):
    drone.take_off(1.75)
    try:
        drone.follow(TAG, standoff=3.0)
    except TargetLost:
        drone.say("lost the tag: landing")
        drone.land()
