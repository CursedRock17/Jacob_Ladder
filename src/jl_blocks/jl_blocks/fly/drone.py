"""Fly a drone from a plain Python script, like a Crazyflie MotionCommander.

    from jl_blocks.fly import Drone

    with Drone("SquareFlight") as drone:   # waits for you to select the mode in QGC,
        drone.forward(2)                   # arms and takes off, ...
        drone.turn_left(90)
        drone.forward(2)
    # ... and lands when the block ends, even if the script crashed.

Distances are metres and angles are degrees. "Up" is positive. forward / left /
turn_* are relative to where the drone is facing. north / east / up (Position)
are measured from the spot where PX4's position estimate started.

Everything goes through the same jl_mission mode as a mission file, so its
safety limits still apply: no setpoint for 0.5 s holds, 5 s lands, and more
than 1 m/s is clamped. Switching modes in QGC stops the script: the pilot wins.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Iterator
from typing import NamedTuple

from ..core import Setpoint, Vector3, VehicleState
from .backend import Backend

MAX_SPEED = 1.0  # m/s: jl_mission's default limit; it clamps anything faster
RATE_HZ = 50.0
ARRIVE_M = 0.15  # a move is done once the drone is this close to its goal
ARRIVE_DEG = 5.0
ARRIVE_SPEED = 0.2  # m/s
SETTLE_S = 0.4  # ... and has stayed there this long
MOVE_TIMEOUT_S = 10.0  # extra time allowed beyond what the move should take


class FlightError(RuntimeError):
    """The flight cannot continue. Leaving the `with` block still lands the drone."""


class Position(NamedTuple):
    """North, east and up in metres, from where PX4's position estimate started."""

    north: float
    east: float
    up: float

    def with_up(self, up: float) -> Position:
        return Position(self.north, self.east, up)


def _wrap(angle: float) -> float:
    return math.remainder(angle, 2 * math.pi)


class Drone:
    def __init__(
        self,
        name: str,
        *,
        backend: Backend | None = None,
        default_height: float = 1.5,
        velocity: float = 0.5,
        take_off_on_enter: bool = True,
        select_timeout: float | None = None,
        rate_hz: float = RATE_HZ,
        keepalive: bool = True,
        log: Callable[[str], None] = print,
    ) -> None:
        """name: the mode name shown in QGC; jl_mission must be running with it.

        take_off_on_enter=False leaves the drone on the ground, for scripts that
        call arm() and send_position() themselves. keepalive keeps re-sending
        the last setpoint while the script is busy (say, in time.sleep), so the
        0.5 s silence watchdog does not fire.
        """
        if not 0 < default_height <= 10:
            raise ValueError(
                f"default_height must be in (0, 10] m, got {default_height}"
            )
        if not 0 < velocity <= MAX_SPEED:
            raise ValueError(
                f"velocity must be in (0, {MAX_SPEED}] m/s, got {velocity}"
            )
        self.name = name
        self._backend = backend
        self._default_height = default_height
        self._velocity = velocity
        self._take_off_on_enter = take_off_on_enter
        self._select_timeout = select_timeout
        self._period = 1.0 / rate_hz
        self._keepalive = keepalive
        self._log = log

        self._lock = threading.Lock()
        self._setpoint: Setpoint | None = None
        self._target: Vector3 | None = None  # commanded position, NED
        self._yaw = 0.0  # commanded heading, radians clockwise from north
        self._published_at = -math.inf
        self._next_tick: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._thread_error: str | None = None
        self._in_charge = False  # the mode was selected and has not been taken away
        self._landed = False

    # ── the with block ──

    def __enter__(self) -> Drone:
        if self._backend is None:
            from ..ros.fly_backend import RosBackend  # needs ROS, so only now

            self._backend = RosBackend(self.name)
        self._backend.connect()
        try:
            self._wait_for_vehicle()
            self._wait_until_selected()
            vehicle = self._vehicle()
            if not vehicle.landed:
                raise FlightError(
                    f"'{self.name}' was already selected while the drone is in the "
                    "air. Select another mode, then select it again."
                )
            self._in_charge = True
            self._target = vehicle.position_ned
            self._yaw = vehicle.yaw
            self._set(Setpoint(position=self._target, yaw=self._yaw))
            self._start_keepalive()
            if self._take_off_on_enter:
                self.take_off()
        except BaseException:
            self._shutdown()
            raise
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        try:
            if self._in_charge and not self._landed:
                if exc_type is not None:
                    self._log(f"{exc_type.__name__}: landing")
                try:
                    self.land()
                except FlightError as err:
                    if exc_type is None:
                        raise
                    self._log(f"landing failed: {err}")
        finally:
            self._shutdown()
        return False

    # ── the basic commands ──

    def arm(self) -> None:
        """Arm and stay on the ground, with the script in control.

        This does not take off: PX4 only lifts off from position setpoints in
        Offboard mode, never in an external mode like this one. Call take_off()
        next (it arms by itself if you skip this).
        """
        self._call("arm", self.backend.arm)
        self._hold_here()

    def take_off(self, height: float | None = None) -> None:
        """Arm and climb with PX4's own takeoff, to `height` metres (default_height)."""
        height = self._default_height if height is None else height
        if not 0 < height <= 10:
            raise ValueError(f"height must be in (0, 10] m, got {height}")
        self._call("takeoff", lambda: self.backend.takeoff(height))
        self._hold_here()

    def land(self) -> None:
        """Land with PX4's own land mode, and wait until the drone is down."""
        self._call("land", self.backend.land)
        self._landed = True

    # ── moving: blocking, relative to where the drone faces ──

    def forward(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(distance, 0.0, 0.0, velocity)

    def back(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(-distance, 0.0, 0.0, velocity)

    def left(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(0.0, distance, 0.0, velocity)

    def right(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(0.0, -distance, 0.0, velocity)

    def up(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(0.0, 0.0, distance, velocity)

    def down(self, distance: float, velocity: float | None = None) -> None:
        self.move_distance(0.0, 0.0, -distance, velocity)

    def move_distance(
        self,
        forward: float,
        left: float,
        up: float,
        velocity: float | None = None,
    ) -> None:
        """Move in a straight line by this much, relative to the drone's heading."""
        north, east, down = self._position_target()
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._fly_to(
            (north + forward * c + left * s, east + forward * s - left * c, down - up),
            self._speed(velocity),
        )

    def goto(
        self, north: float, east: float, up: float, velocity: float | None = None
    ) -> None:
        """Fly in a straight line to a spot measured from where PX4 started."""
        self._position_target()
        self._fly_to((north, east, -up), self._speed(velocity))

    def turn_left(self, degrees: float, rate: float = 90.0) -> None:
        self._turn(-math.radians(degrees), rate)

    def turn_right(self, degrees: float, rate: float = 90.0) -> None:
        self._turn(math.radians(degrees), rate)

    def hover(self, seconds: float) -> None:
        """Stay where the drone is for this long."""
        self._position_target()
        for _ in self.every(seconds):
            pass

    # ── moving: start and stop, without waiting ──

    def start_forward(self, velocity: float | None = None) -> None:
        self._start_moving(self._speed(velocity), 0.0, 0.0)

    def start_back(self, velocity: float | None = None) -> None:
        self._start_moving(-self._speed(velocity), 0.0, 0.0)

    def start_left(self, velocity: float | None = None) -> None:
        self._start_moving(0.0, self._speed(velocity), 0.0)

    def start_right(self, velocity: float | None = None) -> None:
        self._start_moving(0.0, -self._speed(velocity), 0.0)

    def start_up(self, velocity: float | None = None) -> None:
        self._start_moving(0.0, 0.0, self._speed(velocity))

    def start_down(self, velocity: float | None = None) -> None:
        self._start_moving(0.0, 0.0, -self._speed(velocity))

    def stop(self) -> None:
        """Stop a start_* move and hold here."""
        self._hold_here()

    # ── moving: your own loop ──

    @property
    def position(self) -> Position:
        north, east, down = self._vehicle().position_ned
        return Position(north, east, -down)

    @property
    def heading(self) -> float:
        """Degrees clockwise from north."""
        return math.degrees(self._vehicle().yaw)

    def now(self) -> float:
        """Seconds, for `while drone.now() - start < 5:` loops."""
        return self.backend.now()

    def send_position(self, position: Position | Vector3, heading: float | None = None):
        """Command a position (north, east, up) for the next tick. Call tick() after.

        heading: degrees clockwise from north; omitted keeps the current heading.
        """
        north, east, up = position
        if heading is not None:
            self._yaw = math.radians(heading)
        self._target = (north, east, -up)
        self._set(Setpoint(position=self._target, yaw=_wrap(self._yaw)))

    def tick(self) -> None:
        """Send the current setpoint, then wait for the next 50 Hz step."""
        self._check_in_charge()
        self._publish()
        now = self.backend.now()
        step = self._next_tick if self._next_tick is not None else now
        self._next_tick = max(step + self._period, now)
        self.backend.sleep(self._next_tick - now)

    def every(self, duration: float) -> Iterator[float]:
        """`for t in drone.every(5):` runs the body at 50 Hz for 5 s; t is elapsed time."""
        start = self.backend.now()
        while self.backend.now() - start < duration:
            yield self.backend.now() - start
            self.tick()

    # ── inside ──

    @property
    def backend(self) -> Backend:
        if self._backend is None:
            raise FlightError("use the Drone inside `with Drone(...) as drone:`")
        return self._backend

    def _vehicle(self) -> VehicleState:
        vehicle = self.backend.vehicle()
        if vehicle is None:
            raise FlightError(
                "no recent position from PX4 (are the DDS agent and translation_node running?)"
            )
        return vehicle

    def _wait_for_vehicle(self) -> None:
        deadline = self.backend.now() + 15.0
        while self.backend.vehicle() is None:
            if self.backend.now() > deadline:
                raise FlightError(
                    "no position from PX4 after 15 s (are the DDS agent and "
                    "translation_node running?)"
                )
            self.backend.sleep(0.1)

    def _wait_until_selected(self) -> None:
        if self.backend.is_active():
            return
        self._log(
            f"Waiting for '{self.name}': select it in QGC (or px4-commander mode ext1)"
        )
        start = self.backend.now()
        while not self.backend.is_active():
            if (
                self._select_timeout is not None
                and self.backend.now() - start > self._select_timeout
            ):
                raise FlightError(f"'{self.name}' was not selected in time")
            self.backend.sleep(0.1)

    def _call(self, what: str, call: Callable[[], tuple[bool, str]]) -> None:
        self._check_in_charge()
        # Send nothing while the executor works: it ignores setpoints until it hands
        # over, and a stale one must not be the first thing the relay sees.
        self._set(None)
        ok, message = call()
        if not ok:
            raise FlightError(f"{what} failed: {message}")

    def _check_in_charge(self) -> None:
        if self._thread_error is not None:
            raise FlightError(self._thread_error)
        if self._in_charge and not self.backend.is_active():
            self._in_charge = False
            raise FlightError(
                f"'{self.name}' was deselected in QGC: the pilot has control, script stopped"
            )

    def _set(self, setpoint: Setpoint | None) -> None:
        with self._lock:
            self._setpoint = setpoint

    def _publish(self) -> None:
        with self._lock:
            if self._setpoint is not None:
                self._published_at = self.backend.now()
                self.backend.publish(self._setpoint)

    def _start_keepalive(self) -> None:
        if not self._keepalive:
            return

        def run() -> None:
            while not self._stop.wait(self._period / 2):
                if self.backend.now() - self._published_at >= 2 * self._period:
                    try:
                        self._publish()
                    except (
                        Exception
                    ) as err:  # the main thread raises it on its next tick
                        self._thread_error = f"setpoint publisher failed: {err}"
                        return

        self._thread = threading.Thread(
            target=run, daemon=True, name="jl_fly_keepalive"
        )
        self._thread.start()

    def _shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self._backend is not None:
            self._backend.close()

    def _hold_here(self) -> None:
        vehicle = self._vehicle()
        self._target = vehicle.position_ned
        self._set(Setpoint(position=self._target, yaw=_wrap(self._yaw)))

    def _position_target(self) -> Vector3:
        """The commanded position, starting from here if a start_* move was running."""
        if self._target is None:
            self._hold_here()
        assert self._target is not None
        return self._target

    def _speed(self, velocity: float | None) -> float:
        velocity = self._velocity if velocity is None else velocity
        if not 0 < velocity <= MAX_SPEED:
            raise ValueError(
                f"velocity must be in (0, {MAX_SPEED}] m/s, got {velocity}"
            )
        return velocity

    def _fly_to(self, end: Vector3, speed: float) -> None:
        """Slide the commanded position to `end` at `speed`, then wait for the drone."""
        start = self._position_target()
        distance = math.dist(start, end)
        began = self.backend.now()
        travelled = 0.0
        while travelled < distance:
            travelled = min(speed * (self.backend.now() - began), distance)
            f = travelled / distance
            self._target = (
                start[0] + (end[0] - start[0]) * f,
                start[1] + (end[1] - start[1]) * f,
                start[2] + (end[2] - start[2]) * f,
            )
            self._set(Setpoint(position=self._target, yaw=_wrap(self._yaw)))
            self.tick()
        self._target = end
        self._set(Setpoint(position=end, yaw=_wrap(self._yaw)))
        self._wait_until(
            lambda v: (
                math.dist(v.position_ned, end) < ARRIVE_M
                and math.hypot(*v.velocity_ned) < ARRIVE_SPEED
            ),
            allowed=distance / speed,
            what=f"reach {tuple(round(x, 2) for x in end)} (NED)",
        )

    def _turn(self, delta: float, rate: float) -> None:
        if not 0 < rate <= 360:
            raise ValueError(f"rate must be in (0, 360] degrees/s, got {rate}")
        self._position_target()
        start, end = self._yaw, self._yaw + delta
        began = self.backend.now()
        turned = 0.0
        while turned < abs(delta):
            turned = min(math.radians(rate) * (self.backend.now() - began), abs(delta))
            self._yaw = start + math.copysign(turned, delta)
            self._set_hold(_wrap(self._yaw))
            self.tick()
        self._yaw = end
        self._set_hold(_wrap(end))
        self._wait_until(
            lambda v: abs(_wrap(v.yaw - end)) < math.radians(ARRIVE_DEG),
            allowed=abs(delta) / math.radians(rate),
            what="finish the turn",
        )

    def _set_hold(self, yaw: float) -> None:
        assert self._target is not None
        self._set(Setpoint(position=self._target, yaw=yaw))

    def _start_moving(self, forward: float, left: float, up: float) -> None:
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        velocity = (forward * c + left * s, forward * s - left * c, -up)
        self._target = None
        self._set(Setpoint(velocity=velocity, yaw=_wrap(self._yaw)))

    def _wait_until(
        self, arrived: Callable[[VehicleState], bool], allowed: float, what: str
    ) -> None:
        """Tick until `arrived` stays true for SETTLE_S, or fail after the move's time + 10 s."""
        deadline = self.backend.now() + allowed + MOVE_TIMEOUT_S
        since: float | None = None
        while True:
            self.tick()
            now = self.backend.now()
            if arrived(self._vehicle()):
                since = now if since is None else since
                if now - since >= SETTLE_S:
                    return
            else:
                since = None
            if now > deadline:
                raise FlightError(f"the drone did not {what} in time")
