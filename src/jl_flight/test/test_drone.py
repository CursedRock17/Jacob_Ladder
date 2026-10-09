"""The Drone: moves, heading, landing on exit, and the safety behaviour."""

from __future__ import annotations

import math
import time

import pytest
from fake_backend import FakeBackend

from jl_flight import Drone, FlightError, Position, Timeout


def make(backend: FakeBackend | None = None, **kwargs):
    backend = backend or FakeBackend()
    kwargs.setdefault("keepalive", False)
    kwargs.setdefault("log", lambda message: None)
    return backend, Drone("Test", backend=backend, **kwargs)


def test_enter_takes_off_and_exit_lands() -> None:
    backend, drone = make()
    with drone as d:
        assert d.position.up == pytest.approx(1.5, abs=0.05)
    assert backend.calls == ["connect", "takeoff 1.5", "land"]
    assert backend.closed


def test_takeoff_height_is_configurable() -> None:
    backend, drone = make(default_height=2.5)
    with drone:
        pass
    assert "takeoff 2.5" in backend.calls


class SelectedAfterOneSecond(FakeBackend):
    def sleep(self, seconds: float) -> None:
        super().sleep(seconds)
        if self.t > 1.0:
            self.active = True  # the pilot selects the mode after a second


def test_waits_until_the_mode_is_selected() -> None:
    backend = SelectedAfterOneSecond(active=False)
    messages: list[str] = []
    drone = Drone("Test", backend=backend, keepalive=False, log=messages.append)
    with drone:
        assert backend.t > 1.0
        assert "takeoff 1.5" in backend.calls
    assert any("select it in QGC" in m for m in messages)


def test_never_selected_times_out() -> None:
    _, drone = make(FakeBackend(active=False), select_timeout=2.0)
    with pytest.raises(FlightError, match="not selected"):
        drone.__enter__()


def test_forward_moves_along_the_heading() -> None:
    _, drone = make()
    with drone as d:
        d.forward(2.0)
        assert d.position.north == pytest.approx(2.0, abs=0.2)
        assert d.position.east == pytest.approx(0.0, abs=0.2)


def test_moves_follow_the_heading_after_a_turn() -> None:
    _, drone = make()
    with drone as d:
        d.turn_right(90)
        assert d.heading == pytest.approx(90.0, abs=6)
        d.forward(1.0)  # facing east now
        assert d.position.east == pytest.approx(1.0, abs=0.2)
        assert d.position.north == pytest.approx(0.0, abs=0.2)
        d.left(1.0)  # left of east is north
        assert d.position.north == pytest.approx(1.0, abs=0.2)


def test_turn_left_decreases_heading() -> None:
    _, drone = make()
    with drone as d:
        d.turn_left(90)
        assert d.heading == pytest.approx(-90.0, abs=6)


def test_up_down_back_right() -> None:
    _, drone = make()
    with drone as d:
        d.up(1.0)
        assert d.position.up == pytest.approx(2.5, abs=0.2)
        d.down(0.5)
        assert d.position.up == pytest.approx(2.0, abs=0.2)
        d.back(1.0)
        assert d.position.north == pytest.approx(-1.0, abs=0.2)
        d.right(1.0)  # facing north: right is east
        assert d.position.east == pytest.approx(1.0, abs=0.2)


def test_move_distance_and_goto() -> None:
    _, drone = make()
    with drone as d:
        d.move_distance(1.0, 1.0, 0.5)
        assert d.position.north == pytest.approx(1.0, abs=0.2)
        assert d.position.east == pytest.approx(-1.0, abs=0.2)
        assert d.position.up == pytest.approx(2.0, abs=0.2)
        d.goto(0.0, 0.0, 1.5)
        assert d.position == pytest.approx(Position(0.0, 0.0, 1.5), abs=0.2)


def test_a_move_never_outruns_the_speed_or_goes_quiet() -> None:
    backend, drone = make(velocity=0.5)
    with drone as d:
        before = len(backend.published)
        d.forward(2.0)
    moving = backend.published[before:]
    for (t0, a), (t1, b) in zip(moving, moving[1:], strict=False):
        assert t1 - t0 < 0.1  # the 0.5 s silence watchdog has plenty of margin
        if a.position and b.position:
            step = math.dist(a.position, b.position)
            assert step <= 0.5 * (t1 - t0) + 1e-6


def test_start_and_stop() -> None:
    _, drone = make()
    with drone as d:
        d.start_forward(0.5)
        for _ in d.every(2.0):  # moving at 0.5 m/s for 2 s
            pass
        d.stop()
        stopped_at = d.position.north
        assert stopped_at == pytest.approx(1.0, abs=0.2)
        d.hover(1.0)
        assert d.position.north == pytest.approx(stopped_at, abs=0.1)


def test_low_level_arm_send_position_land() -> None:
    backend, drone = make(take_off_on_enter=False)
    with drone as d:
        d.arm()
        assert backend.calls[-1] == "arm"
        hover = d.position.with_up(1.5)
        for _ in d.every(5.0):
            d.send_position(hover)
        assert d.position.up == pytest.approx(1.5, abs=0.1)
        d.land()
    assert backend.calls[-1] == "land"
    assert backend.calls.count("land") == 1  # leaving the block does not land twice
    assert not any(c.startswith("takeoff") for c in backend.calls)


def test_every_runs_for_the_duration() -> None:
    backend, drone = make(take_off_on_enter=False)
    with drone as d:
        d.arm()
        start = d.now()
        ticks = sum(1 for _ in d.every(2.0))
        assert d.now() - start == pytest.approx(2.0, abs=0.05)
        assert ticks == pytest.approx(100, abs=2)
        d.land()


def test_a_crash_in_the_script_still_lands() -> None:
    backend, drone = make()
    with pytest.raises(ZeroDivisionError):
        with drone as d:
            d.forward(0.5)
            1 / 0  # noqa: B018
    assert backend.calls[-1] == "land"


def test_ctrl_c_lands() -> None:
    backend, drone = make()
    with pytest.raises(KeyboardInterrupt):
        with drone:
            raise KeyboardInterrupt
    assert backend.calls[-1] == "land"


def test_the_pilot_taking_over_stops_the_script_without_landing() -> None:
    backend, drone = make()
    with pytest.raises(FlightError, match="pilot has control"):
        with drone as d:
            backend.active = False  # mode switched away in QGC
            d.forward(1.0)
    assert "land" not in backend.calls


def test_failed_takeoff_is_an_error() -> None:
    backend, drone = make()
    backend.fail["takeoff"] = "arm failed: denied"
    with pytest.raises(FlightError, match="takeoff failed: arm failed: denied"):
        drone.__enter__()
    assert backend.closed


def test_a_move_that_never_arrives_fails() -> None:
    backend, drone = make()
    with pytest.raises(Timeout, match="GoTo did not finish"):
        with drone as d:
            # the vehicle stops following setpoints, e.g. a blocked drone
            backend._advance = lambda dt: setattr(backend, "t", backend.t + dt)  # type: ignore[method-assign]
            d.forward(1.0)
    assert backend.calls[-1] == "land"


def test_refuses_to_start_while_already_flying() -> None:
    backend = FakeBackend()
    backend.landed = False
    _, drone = make(backend)
    with pytest.raises(
        FlightError, match="already selected while the drone is in the air"
    ):
        drone.__enter__()


def test_no_vehicle_data_is_an_error() -> None:
    backend = FakeBackend()
    backend.stale = True
    _, drone = make(backend)
    with pytest.raises(FlightError, match="no position from PX4"):
        drone.__enter__()


@pytest.mark.parametrize("bad", [0.0, -1.0, 1.5])
def test_bad_velocity_is_rejected(bad: float) -> None:
    with pytest.raises(ValueError, match="velocity"):
        Drone("Test", backend=FakeBackend(), velocity=bad)
    _, drone = make()
    with drone as d, pytest.raises(ValueError, match="velocity"):
        d.forward(1.0, velocity=bad)


def test_using_a_drone_outside_with_is_a_clear_error() -> None:
    with pytest.raises(FlightError, match="inside"):
        Drone("Test").now()


class RealClock(FakeBackend):
    """Real time, so the keepalive thread can be checked."""

    def now(self) -> float:
        return time.monotonic()

    def sleep(self, seconds: float) -> None:
        time.sleep(max(seconds, 0.0))

    def publish(self, setpoint) -> None:
        self.published.append((time.monotonic(), setpoint))


def test_keepalive_sends_setpoints_while_the_script_sleeps() -> None:
    backend = RealClock()
    drone = Drone("Test", backend=backend, take_off_on_enter=False, log=lambda m: None)
    with drone as d:
        d.arm()
        count = len(backend.published)
        time.sleep(0.5)  # the script is busy: no tick() for half a second
        sent = len(backend.published) - count
        times = [t for t, _ in backend.published[count:]]
        assert sent >= 10
        assert max(b - a for a, b in zip(times, times[1:], strict=False)) < 0.15
        d.land()
