from __future__ import annotations

import pytest

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import Action, Engine, MissionError, VehicleState, parse_mission

DT = 0.02
GROUND = VehicleState(position_ned=(0.0, 0.0, 0.0), landed=True)
AIR = VehicleState(position_ned=(0.0, 0.0, -2.0))


def started(text):
    engine = Engine(parse_mission(text))
    engine.start(0.0)
    return engine


def test_takeoff_asks_for_the_height_then_moves_on_when_it_is_reached():
    engine = started("name: M\nsteps:\n  - takeoff: {height: 2.0}\n  - hold: {}\n")
    assert engine.tick(GROUND, DT, DT) is None
    entry, action = engine.take_action()
    assert action == Action("takeoff", 2.0)
    engine.action_done(entry, True, "reached 2.00 m")
    engine.tick(AIR, 2 * DT, DT)
    assert engine.state == "hold"


def test_takeoff_default_height_is_one_and_a_half_metres():
    engine = started("name: M\nsteps:\n  - takeoff: {}\n")
    engine.tick(GROUND, DT, DT)
    assert engine.take_action()[1] == Action("takeoff", 1.5)


def test_a_failed_takeoff_fails_the_step():
    engine = started("name: M\nsteps:\n  - takeoff: {}\n")
    engine.tick(GROUND, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, False, "arming denied")
    engine.tick(GROUND, 2 * DT, DT)
    assert engine.aborted
    assert "takeoff: failed (takeoff failed: arming denied)" in engine.events[-1]


@pytest.mark.parametrize("height", [0, -1, 10.5])
def test_takeoff_height_must_be_sensible(height):
    with pytest.raises(MissionError, match="height"):
        parse_mission(f"name: M\nsteps:\n  - takeoff: {{height: {height}}}\n")


def test_land_asks_to_land_and_ends_the_mission():
    engine = started("name: M\nsteps:\n  - land: {}\n")
    engine.tick(AIR, DT, DT)
    entry, action = engine.take_action()
    assert action == Action("land")
    engine.action_done(entry, True, "landed")
    engine.tick(GROUND, 2 * DT, DT)
    assert engine.finished
