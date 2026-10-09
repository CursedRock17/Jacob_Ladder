"""The camera missions in missions/, flown end to end against the simulated drone.

The AprilTag (optical frame) and the YOLO drogue (ranging frame) must fly the
same: the approach is the same code, only the perception differs. The fake
publishes detections from a camera mounted like x500_rgbd's.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest
from fake_backend import FakeBackend

from jl_flight import FlightError
from jl_flight.airframes import X500_RGBD
from jl_flight.missions import load, run

REPO = Path(__file__).resolve().parents[3]
MOUNT = X500_RGBD.camera("front").mount
TARGET = (6.0, 0.0, -1.2)  # NED: 6 m north (the drone starts facing north), 1.2 m up


class Pilot(FakeBackend):
    """Places the target, moves it by `drift` (north, east) over 10 s from 15 s
    in, hides it from `hide_at`, and takes over (deselects the mode) at `takeover_at`."""

    def __init__(
        self, spec, drift=(0.0, 0.0), hide_at=math.inf, takeover_at=60.0
    ) -> None:
        super().__init__(camera_offset=(MOUNT.forward, MOUNT.right, MOUNT.down))
        (self.need,) = spec.needs
        self.drift, self.hide_at, self.takeover_at = drift, hide_at, takeover_at

    def _advance(self, dt: float) -> None:
        f = min(max(self.t - 15.0, 0.0) / 10.0, 1.0)
        where = (
            TARGET[0] + f * self.drift[0],
            TARGET[1] + f * self.drift[1],
            TARGET[2],
        )
        self.put_tag(
            self.need.topic, None if self.t >= self.hide_at else where, self.need.frame
        )
        if self.t >= self.takeover_at:
            self.active = False
        super()._advance(dt)


def fly(name: str, **pilot) -> Pilot:
    spec = load(REPO / "missions" / name)
    backend = Pilot(spec, **pilot)
    if backend.takeover_at < math.inf and backend.hide_at == math.inf:
        with pytest.raises(FlightError, match="pilot has control"):
            run(spec, airframe=X500_RGBD, backend=backend, log=lambda m: None)
    else:
        run(spec, airframe=X500_RGBD, backend=backend, log=lambda m: None)
    return backend


@pytest.mark.parametrize("name", ["apriltag_approach.py", "drogue_approach.py"])
def test_the_approach_holds_3_m_short_until_the_pilot_takes_over(name: str) -> None:
    b = fly(name)
    north, east, down = b.position
    assert north == pytest.approx(TARGET[0] - 3.0, abs=0.25)
    assert east == pytest.approx(0.0, abs=0.2)
    assert down == pytest.approx(TARGET[2], abs=0.2)  # at the target's height
    assert b.calls == ["connect", "takeoff 1.75"]  # the pilot has it now: no landing


def test_the_drogue_is_followed_as_it_moves_away() -> None:
    b = fly("drogue_approach.py", drift=(1.0, 0.0))
    assert b.position[0] == pytest.approx(TARGET[0] + 1.0 - 3.0, abs=0.25)


def test_a_drogue_moving_sideways_is_held_on_the_same_ring_and_faced() -> None:
    # Like DroneSmoothPlanner: the standoff is measured along the drone's own line
    # to the target, so the drone turns to face it rather than sliding with it.
    b = fly("drogue_approach.py", drift=(0.0, 1.0))
    north, east, _ = b.position
    assert math.hypot(TARGET[0] - north, 1.0 - east) == pytest.approx(3.0, abs=0.25)
    bearing = math.degrees(math.atan2(1.0 - east, TARGET[0] - north))
    assert math.degrees(b.yaw) == pytest.approx(bearing, abs=5)


@pytest.mark.parametrize("name", ["apriltag_approach.py", "drogue_approach.py"])
def test_losing_the_target_lands(name: str) -> None:
    b = fly(name, hide_at=30.0)
    assert b.calls[-1] == "land"
    assert any(e.startswith("lost the") for e in b.events)


def test_land_before_apriltag_lands_1_5_m_short() -> None:
    b = fly("land_before_apriltag.py", takeover_at=math.inf)
    assert b.calls == ["connect", "takeoff 1.5", "land"]
    assert b.position[0] == pytest.approx(TARGET[0] - 1.5, abs=0.2)
