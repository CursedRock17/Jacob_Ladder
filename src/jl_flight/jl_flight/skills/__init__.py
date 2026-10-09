"""The skills shipped with Jacob's Ladder. Each is a small class: construct it
with keyword arguments and run it with `drone.run(skill)` (or use the Drone's
own shortcuts like `drone.forward()` and `drone.approach()`)."""

from .goto import GoTo
from .hold import Hold
from .precision_descend import PrecisionDescend
from .search import Search
from .track import Track
from .turn import Turn

__all__ = ["GoTo", "Hold", "PrecisionDescend", "Search", "Track", "Turn"]
