"""The action channel: a mission block asks the executor to take off or land."""

from __future__ import annotations

import pytest

from jl_blocks.core import Action, Engine, VehicleState, parse_mission

DT = 0.02
HERE = VehicleState(position_ned=(0.0, 0.0, -1.0))


def started(text, registry):
    engine = Engine(parse_mission(text, registry), registry)
    engine.start(0.0)
    return engine


def test_an_action_is_handed_out_once(registry):
    engine = started("name: M\nsteps:\n  - asker: {}\n", registry)
    assert engine.tick(HERE, DT, DT) is None
    taken = engine.take_action()
    assert taken is not None
    assert taken[1] == Action("takeoff", 1.5)
    engine.tick(HERE, 2 * DT, DT)
    assert engine.take_action() is None


def test_success_finishes_the_step(registry):
    engine = started("name: M\nsteps:\n  - asker: {}\n  - goto: {}\n", registry)
    engine.tick(HERE, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, True, "reached 1.50 m")
    engine.tick(HERE, 2 * DT, DT)
    assert engine.state == "goto"
    assert "asker: takeoff ok (reached 1.50 m)" in engine.events


def test_failure_fails_the_step_with_the_executor_message(registry):
    text = (
        "name: M\nsteps:\n  - asker: {}\n    on_fail: rescue\n"
        "  - goto: {}\n    name: rescue\n"
    )
    engine = started(text, registry)
    engine.tick(HERE, DT, DT)
    entry, _ = engine.take_action()
    engine.action_done(entry, False, "arming denied")
    engine.tick(HERE, 2 * DT, DT)
    assert engine.state == "rescue"
    assert "asker: failed (takeoff failed: arming denied) -> rescue" in engine.events


def test_a_reply_for_a_step_that_already_ended_is_ignored(registry):
    text = (
        "name: M\nsteps:\n  - asker: {}\n    timeout: 1\n    on_fail: again\n"
        "  - asker: {}\n    name: again\n"
    )
    engine = started(text, registry)
    engine.tick(HERE, DT, DT)
    old_entry, _ = engine.take_action()
    engine.tick(HERE, 1.0, DT)  # the timeout moves on to "again"
    assert engine.state == "again"
    engine.action_done(old_entry, True, "late")
    engine.tick(HERE, 1.02, DT)
    assert engine.state == "again"
    new_entry, _ = engine.take_action()
    assert new_entry != old_entry


def test_nothing_is_requested_after_the_mission_ends(registry):
    engine = started("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    engine.tick(HERE, DT, DT)
    assert engine.finished
    assert engine.take_action() is None


def test_each_engine_gets_its_own_copy_of_the_params(registry):
    spec = parse_mission("name: M\nsteps:\n  - mutator: {}\n", registry)
    first = Engine(spec, registry)
    first.start(0.0)
    first.tick(HERE, DT, DT)
    first.tick(HERE, 2 * DT, DT)
    second = Engine(spec, registry)
    second.start(0.0)
    assert second.tick(HERE, DT, DT).position == (2.0, 0.0, -1.0)  # ty: ignore[unresolved-attribute]


def test_drain_events_returns_each_event_once(registry):
    engine = started("name: M\nsteps:\n  - goto: {ticks: 1}\n", registry)
    engine.tick(HERE, DT, DT)
    assert engine.drain_events() == ["goto: started", "goto: done", "mission finished"]
    assert engine.drain_events() == []


def test_a_target_reset_failure_names_the_step_that_owns_the_target(registry):
    text = (
        "name: M\nsteps:\n  - goto: {}\n"
        "  - goto: {}\n    name: second\n    target: {badtarget: {}}\n"
    )
    engine = started(text, registry)
    assert engine.aborted
    assert engine.events[-1].startswith(
        "second: failed (reset: RuntimeError: target reset boom)"
    )


def test_targets_lists_each_distinct_target_once(registry):
    text = (
        "name: M\ntarget: {beacon: {}}\nsteps:\n  - goto: {}\n"
        "  - goto: {}\n    name: b\n"
        "  - goto: {}\n    name: c\n    target: {counter: {}}\n"
    )
    engine = Engine(parse_mission(text, registry), registry)
    assert len(engine.targets()) == 2


def test_vehicle_state_carries_a_timestamp():
    assert VehicleState(position_ned=(0.0, 0.0, 0.0), stamp=3.5).stamp == 3.5


def test_an_unknown_action_is_rejected():
    with pytest.raises(ValueError, match="takeoff or land"):
        Action("fly")
