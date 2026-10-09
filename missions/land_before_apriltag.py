"""Fly up to an AprilTag on a wall and land on the ground in front of it.

The tag is assumed to be in view of the front camera after takeoff. The drone
flies to a point 1.5 m short of it, facing the wall, holds, and lands. If the
tag is lost, the error ends the mission and the drone lands where it is.
"""

from jl_flight import AprilTag, mission

TAG = AprilTag(id=0, size=0.30, family="36h11")


@mission("LandBeforeAprilTag", needs=[TAG])
def land_before_apriltag(drone):
    drone.take_off(1.5)
    drone.approach(TAG, standoff=1.5, timeout=60)
    drone.hover(2)
    drone.land()
