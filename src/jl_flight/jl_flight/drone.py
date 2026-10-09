"""Fly a drone from a plain Python function, like a Crazyflie MotionCommander.

    from jl_flight import AprilTag, mission

    TAG = AprilTag(id=0, size=0.30)

    @mission("LandBeforeAprilTag", needs=[TAG])
    def fly(drone):
        drone.take_off(1.5)
        drone.search(TAG, timeout=30)
        drone.approach(TAG, standoff=1.5)
        drone.land()

or, for a quick script, `with Drone("Name") as drone:`, which takes off on
entry and lands on exit, even if the script crashed.

Distances are metres and angles are degrees. "Up" is positive. forward / left /
turn_* are relative to where the drone is facing. north / east / up (Position)
are measured from the spot where PX4's position estimate started.

Everything goes through the same jl_mission mode, so its safety limits still
apply: no setpoint for 0.5 s holds, 5 s lands, and more than 1 m/s is clamped.
Switching modes in QGC stops the script: the pilot wins.

Every motion is a Skill (see jl_flight.skills) run by `run()`; the shortcuts
below (forward, turn_left, search, approach, ...) are thin wrappers, so there is
one implementation of each behaviour however it is called.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable, Iterator, Sequence
from typing import NamedTuple

from .airframes import Airframe, get_airframe
from .backend import Backend
from .controllers import Position as PositionController
from .core import (
    Controller,
    Setpoint,
    Skill,
    SkillContext,
    Status,
    Target,
    Vector3,
    VehicleState,
)
from .errors import FlightError, SkillFailed, TargetLost, Timeout
from .perception import Perception
from .report import describe
from .skills import GoTo, Hold, Search, Track, Turn
from .skills.turn import wrap

MAX_SPEED = 1.0  # m/s: jl_mission's default limit; it clamps anything faster
RATE_HZ = 50.0
MOVE_TIMEOUT_S = 10.0  # extra time a move is allowed beyond what it should take


class Position(NamedTuple):
    """North, east and up in metres, from where PX4's position estimate started."""

    north: float
    east: float
    up: float

    def with_up(self, up: float) -> Position:
        return Position(self.north, self.east, up)


class Drone:
    def __init__(
        self,
        name: str,
        *,
        backend: Backend | None = None,
        airframe: Airframe | str | None = None,
        needs: Sequence[Perception] = (),
        default_height: float = 1.5,
        velocity: float = 0.5,
        take_off_on_enter: bool = True,
        fresh_selection: bool = False,
        select_timeout: float | None = None,
        rate_hz: float = RATE_HZ,
        keepalive: bool = True,
        report_every: float | None = None,
        log: Callable[[str], None] = print,
    ) -> None:
        """name: the mode name shown in QGC; jl_mission must be running with it.

        needs: the perception this flight uses (AprilTag(...), ...). Using
        anything not listed here is an error, so nothing is silently unavailable.
        take_off_on_enter=False leaves the drone on the ground for a mission that
        calls take_off() itself. fresh_selection ignores a mode that was already
        selected when the mission started and waits for it to be selected again,
        so a mission that restarts after finishing never flies on its own.
        keepalive keeps re-sending the last setpoint while the mission is busy
        (say, in time.sleep), so the 0.5 s silence watchdog does not fire.
        report_every: print what is being commanded (setpoint, drone, targets;
        see jl_flight.report) at most this often, in seconds. None: never.
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
        self.airframe = (
            airframe if isinstance(airframe, Airframe) else get_airframe(airframe)
        )
        for need in needs:
            need.validate(self.airframe)
        self._needs = tuple(needs)
        self._targets: dict[Perception, Target] = {}
        self._default_height = default_height
        self._velocity = velocity
        self._take_off_on_enter = take_off_on_enter
        self._fresh_selection = fresh_selection
        self._select_timeout = select_timeout
        self._period = 1.0 / rate_hz
        self._keepalive = keepalive
        self._report_every = report_every
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
        self._activity = "start"  # what the mission is doing, for reports
        self._started_at = 0.0
        self._reported_at = -math.inf

    # ── the with block ──

    def __enter__(self) -> Drone:
        if self._backend is None:
            from .ros.vehicle_link import RosBackend  # needs ROS, so only now

            self._backend = RosBackend(self.name)
        self._backend.connect()
        try:
            for need in self._needs:
                self._bind(need)
            self._wait_for_vehicle()
            self._wait_until_selected()
            vehicle = self._vehicle()
            if not vehicle.landed:
                raise FlightError(
                    f"'{self.name}' was already selected while the drone is in the "
                    "air. Select another mode, then select it again."
                )
            self._in_charge = True
            self._started_at = self.backend.now()
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
        """Arm and stay on the ground, with the mission in control.

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
        self._go_to(
            (north + forward * c + left * s, east + forward * s - left * c, down - up),
            velocity,
        )

    def goto(
        self, north: float, east: float, up: float, velocity: float | None = None
    ) -> None:
        """Fly in a straight line to a spot measured from where PX4 started."""
        self._go_to((north, east, -up), velocity)

    def turn_left(self, degrees: float, rate: float = 90.0) -> None:
        self._turn(-math.radians(degrees), rate)

    def turn_right(self, degrees: float, rate: float = 90.0) -> None:
        self._turn(math.radians(degrees), rate)

    def hover(self, seconds: float) -> None:
        """Stay where the drone is for this long."""
        self.run(Hold(seconds, self._position_target(), wrap(self._yaw)))

    # ── seeing, and flying by what is seen ──

    def seen(self, perception: Perception) -> bool:
        """Is the target in view right now?"""
        return self.target(perception).estimate(self._vehicle()) is not None

    def search(
        self,
        perception: Perception,
        *,
        pattern: str = "hold",
        radius: float = 2.0,
        timeout: float | None = None,
    ) -> None:
        """Hold (or fly a `pattern`: "spiral" out to `radius` metres) until the
        target is seen. Raises Timeout after `timeout` seconds. For more control,
        run the Search skill yourself: drone.run(Search(...), target=..., until=...)."""
        self.run(
            Search(pattern=pattern, radius=radius),
            target=perception,
            timeout=timeout,
            until=lambda ctx: ctx.target_position is not None,
        )

    def approach(
        self,
        perception: Perception,
        *,
        standoff: float,
        tolerance: float = 0.15,
        max_speed: float = 0.5,
        timeout: float | None = None,
        controller: Controller | None = None,
    ) -> None:
        """Fly to `standoff` metres short of the target, facing it, and stop there.

        Raises TargetLost if it is not seen for too long, and Timeout if it
        takes longer than `timeout` seconds. `controller` (default: PX4's own
        position control) turns the setpoints into what is sent, e.g. PID().
        """
        self.run(
            Track(standoff=standoff, tolerance=tolerance, max_speed=max_speed),
            target=perception,
            timeout=timeout,
            controller=controller,
        )

    def follow(
        self,
        perception: Perception,
        *,
        standoff: float,
        duration: float | None = None,
        tolerance: float = 0.25,
        max_speed: float = 0.5,
        controller: Controller | None = None,
    ) -> None:
        """Hold station `standoff` metres short of the target, facing it, re-tracking
        it as it moves, for `duration` seconds (default: until the pilot takes over).

        Raises TargetLost if it is not seen for too long. `controller` as for approach().
        """
        self.run(
            Track(
                standoff=standoff,
                tolerance=tolerance,
                max_speed=max_speed,
                keep_station=True,
            ),
            target=perception,
            duration=duration,
            controller=controller,
        )

    def target(self, perception: Perception) -> Target:
        """The live target for something declared in `needs`."""
        if perception not in self._targets:
            raise FlightError(
                f"{perception} is not in this flight's needs; declare it, e.g. "
                f"@mission(..., needs=[{perception!r}]), so its tracker is started"
            )
        return self._targets[perception]

    # ── skills ──

    def run(
        self,
        skill: Skill,
        *,
        target: Perception | Target | None = None,
        controller: Controller | None = None,
        timeout: float | None = None,
        duration: float | None = None,
        until: Callable[[SkillContext], bool] | None = None,
    ) -> None:
        """Tick a skill about 50 times a second until it is done.

        It stops when the skill says it is done, when `until(ctx)` is true, or
        after `duration` seconds; or by raising: Timeout after `timeout` seconds,
        TargetLost or SkillFailed if the skill gives up. Afterwards the drone
        holds where it ended up.
        """
        name = type(skill).__name__
        tgt = self.target(target) if isinstance(target, Perception) else target
        if skill.needs_target and tgt is None:
            raise FlightError(f"{name} needs a target to follow: pass target=...")
        controller = controller if controller is not None else PositionController()
        skill.reset()
        controller.reset()
        self._activity = name
        self._event(f"{name}: started")
        began = last = self.backend.now()
        last_setpoint: Setpoint | None = None
        while True:
            self._check_in_charge()
            now = self.backend.now()
            dt, last = now - last, now
            vehicle = self._vehicle()
            ctx = SkillContext(
                vehicle=vehicle,
                target=tgt,
                target_position=tgt.estimate(vehicle) if tgt else None,
                elapsed=now - began,
                dt=dt,
            )
            setpoint = skill.step(ctx)
            status = skill.status(ctx)
            last_setpoint = setpoint
            self._set(controller.command(vehicle, setpoint, dt))
            if (
                status is Status.DONE
                or (until is not None and until(ctx))
                or (duration is not None and ctx.elapsed >= duration)
            ):
                break
            if status is Status.FAILED:
                self._event(f"{name}: failed")
                exc = TargetLost if skill.needs_target else SkillFailed
                raise exc(f"{name} gave up")
            if timeout is not None and ctx.elapsed >= timeout:
                self._event(f"{name}: timed out")
                raise Timeout(f"{name} did not finish in {timeout:g} s")
            self.tick()
        self._event(f"{name}: done")
        self._settle_on(last_setpoint)

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
        self._set(Setpoint(position=self._target, yaw=wrap(self._yaw)))

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

    def say(self, text: str) -> None:
        """Report what the mission is doing: logged, and published on /jl/NAME/events."""
        self._event(text)

    def _event(self, text: str) -> None:
        self.backend.event(text)

    def _bind(self, need: Perception) -> None:
        target = need.make_target(self.airframe)
        self._targets[need] = target

        def on_detection(position: Vector3) -> None:
            vehicle = self.backend.vehicle()
            if vehicle is not None:
                target.observe(position, vehicle)  # type: ignore[attr-defined]

        self.backend.subscribe_pose(need.topic, on_detection)

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
        already = self.backend.is_active()
        if already and not self._fresh_selection:
            return
        baseline = self.backend.selections()
        self._log(
            f"Waiting for '{self.name}': select "
            f"{'it again' if already else 'it'} in QGC"
        )
        start = self.backend.now()
        while not (
            self.backend.is_active()
            and (not already or self.backend.selections() > baseline)
        ):
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
        self._activity = what
        self._event(f"{what}: started")
        ok, message = call()
        if not ok:
            self._event(f"{what}: failed ({message})")
            raise FlightError(f"{what} failed: {message}")
        self._event(f"{what}: done")

    def _check_in_charge(self) -> None:
        if self._thread_error is not None:
            raise FlightError(self._thread_error)
        if self._in_charge and not self.backend.is_active():
            self._in_charge = False
            raise FlightError(
                f"'{self.name}' was deselected in QGC: the pilot has control, mission stopped"
            )

    def _set(self, setpoint: Setpoint | None) -> None:
        with self._lock:
            self._setpoint = setpoint

    def _publish(self) -> None:
        with self._lock:
            setpoint = self._setpoint
            if setpoint is not None:
                self._published_at = self.backend.now()
                self.backend.publish(setpoint)
        self._maybe_report(setpoint)

    def _maybe_report(self, setpoint: Setpoint | None) -> None:
        if self._report_every is None:
            return
        now = self.backend.now()
        if now - self._reported_at < self._report_every:
            return
        self._reported_at = now
        vehicle = self.backend.vehicle()
        seen = {
            need.key: (self._targets[need].estimate(vehicle) if vehicle else None)
            for need in self._needs
        }
        self._log(
            describe(now - self._started_at, self._activity, setpoint, vehicle, seen)
        )

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
            target=run, daemon=True, name="jl_flight_keepalive"
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
        self._set(Setpoint(position=self._target, yaw=wrap(self._yaw)))

    def _position_target(self) -> Vector3:
        """The commanded position, starting from here if a start_* move was running."""
        if self._target is None:
            self._hold_here()
        assert self._target is not None
        return self._target

    def _settle_on(self, setpoint: Setpoint | None) -> None:
        """Remember where a skill left the drone, so the next command starts there."""
        if setpoint is None:
            return
        if setpoint.yaw is not None:
            self._yaw = setpoint.yaw
        if setpoint.position is not None and setpoint.velocity is None:
            self._target = setpoint.position
        else:
            self._hold_here()
        self._set(Setpoint(position=self._target, yaw=wrap(self._yaw)))

    def _speed(self, velocity: float | None) -> float:
        velocity = self._velocity if velocity is None else velocity
        if not 0 < velocity <= MAX_SPEED:
            raise ValueError(
                f"velocity must be in (0, {MAX_SPEED}] m/s, got {velocity}"
            )
        return velocity

    def _go_to(self, end: Vector3, velocity: float | None) -> None:
        skill = GoTo(
            self._position_target(), end, wrap(self._yaw), speed=self._speed(velocity)
        )
        self.run(skill, timeout=skill.expected_s + MOVE_TIMEOUT_S)

    def _turn(self, delta: float, rate: float) -> None:
        if not 0 < rate <= 360:
            raise ValueError(f"rate must be in (0, 360] degrees/s, got {rate}")
        skill = Turn(self._position_target(), self._yaw, delta, math.radians(rate))
        self.run(skill, timeout=skill.expected_s + MOVE_TIMEOUT_S)
        self._yaw = skill.final_yaw

    def _start_moving(self, forward: float, left: float, up: float) -> None:
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        velocity = (forward * c + left * s, forward * s - left * c, -up)
        self._target = None
        self._set(Setpoint(velocity=velocity, yaw=wrap(self._yaw)))
