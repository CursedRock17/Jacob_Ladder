"""Everything that can stop a flight. Catch these in a mission to handle it:

    try:
        drone.approach(TAG, standoff=1.5)
    except TargetLost:
        drone.land()

Whatever is not caught ends the flight, and the `with Drone(...)` block lands the drone.
"""

from __future__ import annotations


class FlightError(RuntimeError):
    """The flight cannot continue."""


class Timeout(FlightError):
    """Something took longer than allowed."""


class SkillFailed(FlightError):
    """A skill gave up."""


class TargetLost(SkillFailed):
    """The target the drone was following has not been seen for too long."""
