"""Jacob's Ladder flight library: missions written as plain Python.

from jl_flight import AprilTag, mission

TAG = AprilTag(id=0, size=0.30)

@mission("LandBeforeAprilTag", needs=[TAG])
def fly(drone):
    drone.take_off(1.5)
    drone.search(TAG, timeout=30)
    drone.approach(TAG, standoff=1.5)
    drone.land()
"""

from .drone import Drone, Position
from .errors import FlightError, SkillFailed, TargetLost, Timeout
from .missions import MissionSpec, mission
from .perception import AprilTag, ArucoTag, YoloDrogue

__all__ = [
    "AprilTag",
    "ArucoTag",
    "Drone",
    "FlightError",
    "MissionSpec",
    "Position",
    "SkillFailed",
    "TargetLost",
    "Timeout",
    "YoloDrogue",
    "mission",
]
