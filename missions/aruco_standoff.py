"""Keep 3 m in front of the vertical ArUco tag in the aruco_dual_ids world.

The tag is assumed to be in view of the front camera after takeoff. The drone
holds 3 m in front of it until the pilot takes over. If the tag is lost for 3 s,
it lands.
"""

from jl_flight import ArucoTag, TargetLost, mission

TAG = ArucoTag(id=0, size=0.15, dictionary_name="4x4_250", camera="front")


@mission("ArucoStandoff", needs=[TAG])
def aruco_standoff(drone):
    drone.take_off(1.0)
    try:
        drone.follow(TAG, standoff=3.0)
    except TargetLost:
        drone.say("lost the tag: landing")
        drone.land()
