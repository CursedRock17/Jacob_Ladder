"""Setpoint reports: what the mission is commanding, printed for checking by eye."""

from __future__ import annotations

from pathlib import Path

from fake_backend import FakeBackend

from jl_flight.core import Setpoint, VehicleState
from jl_flight.missions import load, run
from jl_flight.report import describe

REPO = Path(__file__).resolve().parents[3]


def test_describe_shows_what_is_unknown() -> None:
    v = VehicleState(position_ned=(1.0, 2.0, -3.0), yaw=0.0)
    text = describe(4.0, "takeoff", None, v, {"tag": None})
    assert "setpoint none (PX4 is flying its own takeoff)" in text
    assert "drone    N  +1.00 E  +2.00 U  3.00 hdg    0" in text
    assert "tag  not seen" in text


def test_describe_shows_velocity_when_commanded() -> None:
    sp = Setpoint(position=(0.0, 0.0, -1.0), velocity=(0.5, 0.0, 0.0), yaw=0.0)
    assert "vel N +0.50 E +0.00 U +0.00" in describe(0.0, "Track", sp, None, {})


def test_report_every_prints_in_a_normal_flight_too() -> None:
    spec = load(REPO / "missions" / "takeoff_hold_land.py")
    lines: list[str] = []
    run(spec, backend=FakeBackend(), report_every=1.0, log=lines.append)
    holds = [line for line in lines if " s Hold " in line]
    assert 4 <= len(holds) <= 7  # a 5 s hover, reported once a second
