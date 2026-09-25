from __future__ import annotations

from jl_blocks.core import Engine, VehicleState, parse_mission

DT = 0.02


def engine_for(text, registry):
    engine = Engine(parse_mission(text, registry), registry)
    engine.start(now=0.0)
    return engine


def run(engine, vehicle, seconds, start=0.0):
    """Tick at 50 Hz from `start` for `seconds`; return the last setpoint."""
    setpoint = None
    t = start
    while t < start + seconds - 1e-9:
        t += DT
        setpoint = engine.tick(vehicle, t, DT)
    return setpoint


def test_nothing_happens_before_start(registry, vehicle):
    engine = Engine(
        parse_mission("name: M\nsteps:\n  - goto: {}\n", registry), registry
    )
    assert engine.tick(vehicle, 1.0, DT) is None
    assert engine.state == "idle"


def test_step_output_goes_through_the_controller(registry, vehicle):
    engine = engine_for(
        "name: M\ncontroller: {doubler: {}}\nsteps:\n  - goto: {x: 1.5}\n", registry
    )
    assert engine.tick(vehicle, DT, DT).position == (3.0, 0.0, -1.0)


def test_done_moves_to_the_next_step_and_resets_it(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {ticks: 3}\n  - goto: {x: 9}\n    name: second\n"
    engine = engine_for(text, registry)
    for i in range(3):
        engine.tick(vehicle, (i + 1) * DT, DT)
    assert engine.state == "second"
    assert engine.tick(vehicle, 4 * DT, DT).position == (9.0, 0.0, -1.0)


def test_last_step_done_finishes_and_holds_where_it_ended(registry):
    engine = engine_for("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    here = VehicleState(position_ned=(3.0, 4.0, -2.0))
    engine.tick(here, DT, DT)
    assert engine.finished and engine.state == "finished"
    assert engine.tick(
        VehicleState(position_ned=(0.0, 0.0, 0.0)), 2 * DT, DT
    ).position == (3.0, 4.0, -2.0)


def test_until_seconds(registry, vehicle):
    engine = engine_for("name: M\nsteps:\n  - goto: {}\n    until: 1\n", registry)
    run(engine, vehicle, 0.9)
    assert not engine.finished
    run(engine, vehicle, 0.2, start=0.9)
    assert engine.finished


def test_until_never_ignores_the_block_saying_done(registry, vehicle):
    engine = engine_for(
        "name: M\nsteps:\n  - goto: {ticks: 1}\n    until: never\n", registry
    )
    run(engine, vehicle, 5)
    assert engine.state == "goto"


def test_until_target_seen(registry, vehicle):
    beacon = registry.get("beacon")
    beacon.visible = False
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n    until: target_seen\n"
    )
    engine = engine_for(text, registry)
    run(engine, vehicle, 2)
    assert engine.state == "goto"
    beacon.visible = True
    engine.tick(vehicle, 2.1, DT)
    assert engine.finished


def test_until_target_lost_waits_lost_after_seconds(registry, vehicle):
    beacon = registry.get("beacon")
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n    until: target_lost\n"
    )
    engine = engine_for(text, registry)
    run(engine, vehicle, 1)
    beacon.visible = False
    run(engine, vehicle, 2.9, start=1)
    assert engine.state == "goto"
    run(engine, vehicle, 0.2, start=3.9)
    assert engine.finished


def test_timeout_jumps_to_on_fail(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {}\n    timeout: 2\n    on_fail: rescue\n  - goto: {x: 7}\n    name: rescue\n"
    engine = engine_for(text, registry)
    run(engine, vehicle, 2.1)
    assert engine.state == "rescue"
    assert "goto: failed (timeout after 2 s) -> rescue" in engine.events


def test_failure_without_on_fail_aborts_and_holds(registry):
    engine = engine_for("name: M\nsteps:\n  - quitter: {}\n", registry)
    here = VehicleState(position_ned=(1.0, 2.0, -3.0))
    engine.tick(here, DT, DT)
    assert engine.aborted and engine.state == "aborted"
    assert engine.tick(here, 2 * DT, DT).position == (1.0, 2.0, -3.0)
    assert (
        engine.events[-1]
        == "quitter: failed (block reported failure); no on_fail -> hold, then land"
    )


def test_block_exception_fails_the_step_instead_of_crashing(registry, vehicle):
    text = "name: M\nsteps:\n  - broken: {}\n    on_fail: goto\n  - goto: {}\n"
    engine = engine_for(text, registry)
    setpoint = engine.tick(vehicle, DT, DT)
    assert setpoint.position == vehicle.position_ned
    assert engine.state == "goto"
    assert "broken: failed (RuntimeError: boom) -> goto" in engine.events


def test_on_fail_can_loop_back_and_reset_the_step(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {}\n    name: search\n    until: 0.1\n  - quitter: {}\n    on_fail: search\n"
    engine = engine_for(text, registry)
    run(engine, vehicle, 0.2)
    assert engine.state == "search"
    search = engine._steps[engine._index["search"]].mission
    assert search.count < 5


def test_step_override_controller_applies_only_to_that_step(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {x: 1}\n    controller: {doubler: {}}\n    until: 0.05\n  - goto: {x: 1}\n    name: plain\n"
    engine = engine_for(text, registry)
    assert engine.tick(vehicle, DT, DT).position == (2.0, 0.0, -1.0)
    run(engine, vehicle, 0.1, start=DT)
    assert engine.state == "plain"
    assert engine.tick(vehicle, 1.0, DT).position == (1.0, 0.0, -1.0)


def test_tick_returns_none_when_the_mission_sends_nothing(registry, vehicle):
    engine = engine_for("name: M\nsteps:\n  - needy: {height: 1.0}\n", registry)
    assert engine.tick(vehicle, DT, DT) is None


# -- Fix 1: a reset() error must not crash the engine --------------------


def test_reset_failure_aborts_after_advancing_to_the_bad_step(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {ticks: 1}\n  - badreset: {}\n"
    engine = engine_for(text, registry)
    engine.tick(vehicle, DT, DT)
    assert engine.aborted and engine.state == "aborted"
    assert "badreset: failed (reset: RuntimeError: reset boom)" in engine.events[-1]
    setpoint = engine.tick(vehicle, 2 * DT, DT)
    assert setpoint is not None and setpoint.position == vehicle.position_ned


def test_reset_failure_during_on_fail_jump_aborts_without_raising(registry, vehicle):
    text = "name: M\nsteps:\n  - quitter: {}\n    name: quitter\n    on_fail: badreset\n  - badreset: {}\n    name: badreset\n"
    engine = engine_for(text, registry)
    engine.tick(vehicle, DT, DT)
    assert engine.aborted


def test_reset_failure_on_first_step_of_start_does_not_raise(registry):
    engine = Engine(
        parse_mission("name: M\nsteps:\n  - badreset: {}\n", registry), registry
    )
    engine.start(0.0)
    assert engine.aborted
    assert engine._hold_position is None


# -- Fix 3: keep the traceback for the runner -----------------------------


def test_last_error_holds_the_traceback_of_a_block_exception(registry, vehicle):
    text = "name: M\nsteps:\n  - broken: {}\n    on_fail: goto\n  - goto: {}\n"
    engine = engine_for(text, registry)
    engine.tick(vehicle, DT, DT)
    assert engine.last_error is not None
    assert "Traceback" in engine.last_error
    assert "RuntimeError: boom" in engine.last_error


# -- Fix 4: target reset contract -----------------------------------------


def test_shared_target_is_reset_once_in_start_not_between_steps(registry, vehicle):
    text = (
        "name: M\ntarget: {counter: {}}\nsteps:\n  - goto: {ticks: 1}\n"
        "  - goto: {x: 9}\n    name: second\n    until: 1\n"
    )
    engine = engine_for(text, registry)
    counter = registry.get("counter").instances[-1]
    assert counter.reset_count == 1
    run(engine, vehicle, 2)
    assert engine.finished
    assert counter.reset_count == 1


def test_step_override_target_is_reset_in_start_and_again_on_enter(registry, vehicle):
    text = "name: M\nsteps:\n  - goto: {ticks: 1}\n    target: {counter: {}}\n"
    engine_for(text, registry)  # start(): reset once, then step 0 entered: reset again
    counter = registry.get("counter").instances[-1]
    assert counter.reset_count == 2
