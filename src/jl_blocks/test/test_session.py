from __future__ import annotations

import dataclasses
import math

from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import (
    Action,
    Output,
    Session,
    Setpoint,
    VehicleState,
    parse_mission,
    trajectory_fields,
)

DT = 0.02
MISSION = (
    "name: M\nsteps:\n  - takeoff: {height: 1.5}\n  - hold: {duration: 1}\n"
    "  - land: {}\n"
)
TAG_MISSION = (
    "name: M\ntarget: {aruco_tag: {camera: front}}\n"
    "steps:\n  - search: {}\n    until: target_seen\n  - hold: {}\n"
)
GROUND = VehicleState(position_ned=(0.0, 0.0, 0.0), landed=True)
AIR = VehicleState(position_ned=(0.0, 0.0, -1.5))


def session(text=MISSION, **kwargs):
    s = Session(parse_mission(text), **kwargs)
    # Consume the "first activation ever" slot with a no-op false, so tests
    # built around set_active(True, ...) being a normal start keep working
    # (Review Focus 1's ignore rule only applies to a session's very first
    # /active message).
    s.set_active(False, -1.0)
    return s


def restamp(vehicle, t):
    """A copy of `vehicle` with a fresh `stamp`, as if it had just arrived."""
    return dataclasses.replace(vehicle, stamp=t)


def fly_to_hold(s):
    """Activate on the ground and answer the takeoff; ends in the hold step."""
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, True, "reached 1.50 m")
    s.update_vehicle(AIR)
    s.tick(2 * DT, DT)


def test_nothing_happens_until_the_mission_is_active():
    s = session()
    s.update_vehicle(GROUND)
    assert s.tick(DT, DT) == Output()
    assert s.state == "inactive"


def test_nothing_is_sent_before_the_vehicle_state_is_known():
    s = session()
    s.set_active(True, 0.0)
    assert s.tick(DT, DT) == Output()


def test_activation_asks_for_takeoff_once():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    out = s.tick(DT, DT)
    assert out.setpoint is None
    assert [r.action for r in out.requests] == [Action("takeoff", 1.5)]
    assert s.tick(2 * DT, DT).requests == ()
    assert s.state == "takeoff"


def test_after_takeoff_the_next_step_sends_setpoints():
    s = session()
    fly_to_hold(s)
    assert s.state == "hold"
    assert s.tick(3 * DT, DT).setpoint == Setpoint(position=(0.0, 0.0, -1.5))


def test_every_activation_starts_again_from_step_one():
    s = session()
    fly_to_hold(s)
    s.set_active(True, 1.0)  # a second "true", with no "false" in between
    assert s.state == "takeoff"
    s.update_vehicle(restamp(AIR, 1.0))  # the vehicle topic itself keeps arriving
    assert [r.action.kind for r in s.tick(1.0 + DT, DT).requests] == ["takeoff"]


def test_a_first_true_ever_is_ignored():
    # /jl/NAME/active is latched: a runner started after the mission was
    # already flying (e.g. systemd restarted it mid-flight) must not resume
    # it (spec §10: no resume). The pilot has to re-select it in QGC.
    s = Session(parse_mission(MISSION))
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    assert s.state == "inactive"
    assert s.tick(DT, DT) == Output()
    assert (
        "ignored: the mission was already active when the runner started; "
        "select it again in QGC to fly it" in s.drain_events()
    )


def test_a_true_after_the_ignored_first_true_starts_from_step_one():
    s = Session(parse_mission(MISSION))
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)  # ignored
    s.set_active(True, 1.0)  # a real activation
    assert s.state == "takeoff"
    assert "activated: starting from step 1" in s.drain_events()


def test_false_then_true_starts_normally():
    s = Session(parse_mission(MISSION))
    s.update_vehicle(GROUND)
    s.set_active(False, 0.0)  # changes nothing
    assert s.state == "inactive"
    assert s.drain_events() == []
    s.set_active(True, 1.0)
    assert s.state == "takeoff"


def test_a_reply_from_before_a_reactivation_is_ignored():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    (old,) = s.tick(DT, DT).requests
    s.set_active(False, 0.5)
    s.set_active(True, 1.0)
    s.update_vehicle(restamp(GROUND, 1.0))  # the vehicle topic keeps arriving
    (new,) = s.tick(1.0 + DT, DT).requests
    s.action_done(old.token, True, "late")
    s.tick(1.0 + 2 * DT, DT)
    assert s.state == "takeoff"
    s.action_done(new.token, True, "reached 1.50 m")
    s.update_vehicle(restamp(AIR, 1.0 + 2 * DT))
    s.tick(1.0 + 3 * DT, DT)
    assert s.state == "hold"


def test_deactivation_stops_all_output():
    s = session()
    fly_to_hold(s)
    s.set_active(False, 0.1)
    assert s.tick(0.1 + DT, DT) == Output()
    assert s.state == "inactive"


def test_no_setpoints_while_landed():
    s = session("name: M\nsteps:\n  - hold: {}\n")
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    assert s.tick(DT, DT).setpoint is None
    s.update_vehicle(AIR)
    assert s.tick(2 * DT, DT).setpoint is not None


def test_a_failed_step_with_no_on_fail_holds_then_lands():
    s = session(abort_hold_s=2.0)
    s.update_vehicle(AIR)  # activated in the air
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)  # the step fails here, at t = 0.04
    assert s.state == "aborted"
    out = s.tick(3 * DT, DT)
    assert out.setpoint == Setpoint(position=(0.0, 0.0, -1.5))
    assert out.requests == ()
    s.update_vehicle(restamp(AIR, 2.0))  # the vehicle topic keeps arriving
    assert s.tick(2.03, DT).requests == ()  # held 1.99 s so far
    (land,) = s.tick(2.05, DT).requests
    assert land.action == Action("land")
    assert s.tick(2.07, DT).requests == ()


def test_abort_land_failure_goes_quiet():
    s = session(abort_hold_s=2.0)
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)  # fails -> aborted, at t = 0.04
    s.update_vehicle(restamp(AIR, 2.0))
    s.tick(2.03, DT)
    (land,) = s.tick(2.05, DT).requests  # the abort land is requested here
    s.update_vehicle(restamp(AIR, 2.06))
    s.action_done(land.token, False, "land rejected")
    out = s.tick(2.06, DT)
    assert out.setpoint is None
    assert out.requests == ()


def test_abort_land_no_reply_times_out():
    s = session(abort_hold_s=2.0)  # abort_land_timeout_s defaults to 5.0
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)  # fails -> aborted, at t = 0.04
    s.update_vehicle(restamp(AIR, 2.0))
    s.tick(2.03, DT)
    s.tick(2.05, DT)  # the abort land is requested here, never answered
    s.update_vehicle(restamp(AIR, 7.0))
    out = s.tick(7.03, DT)  # 4.98 s since the land request: still holding
    assert out.setpoint == Setpoint(position=(0.0, 0.0, -1.5))
    s.update_vehicle(restamp(AIR, 7.05))
    out = s.tick(7.05, DT)  # 5.0 s with no reply: go quiet
    assert out.setpoint is None
    assert out.requests == ()
    assert (
        "aborted: no land reply after 5 s; going quiet "
        "(jl_mission lands on silence, or is already landing)" in s.drain_events()
    )


def test_abort_land_going_quiet_is_logged_once():
    s = session(abort_hold_s=2.0)
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)
    s.update_vehicle(restamp(AIR, 2.0))
    s.tick(2.03, DT)
    (land,) = s.tick(2.05, DT).requests
    s.update_vehicle(restamp(AIR, 2.06))
    s.action_done(land.token, False, "land rejected")
    s.tick(2.06, DT)
    events = s.drain_events()
    assert (
        events.count("aborted: land failed, going quiet so jl_mission lands on silence")
        == 1
    )
    s.update_vehicle(restamp(AIR, 2.08))
    s.tick(2.08, DT)
    assert (
        "aborted: land failed, going quiet so jl_mission lands on silence"
        not in s.drain_events()
    )


def test_deactivating_during_the_abort_hold_resets_it():
    s = session(abort_hold_s=2.0)
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    (request,) = s.tick(DT, DT).requests
    s.action_done(request.token, False, "landing in progress")
    s.tick(2 * DT, DT)  # fails -> aborted, at t = 0.04
    assert s.state == "aborted"
    s.set_active(False, 1.0)
    assert s.state == "inactive"
    s.set_active(True, 2.0)  # re-activation starts from step 1
    assert s.state == "takeoff"
    s.update_vehicle(restamp(AIR, 2.0))
    (request2,) = s.tick(2 + DT, DT).requests
    assert request2.action.kind == "takeoff"
    s.action_done(request2.token, False, "landing in progress")
    s.tick(2 + 2 * DT, DT)  # fails -> aborted again, its own fresh hold
    assert s.state == "aborted"
    s.update_vehicle(restamp(AIR, 2 + 1.99))
    # Only 1.99 s into the *new* hold: no land request carried over from the
    # old abort cycle.
    assert s.tick(2 + 1.99 + DT, DT).requests == ()


def test_deactivate_and_reactivate_in_the_same_tick_sends_at_most_one_takeoff_request():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    s.set_active(False, 0.0)
    s.set_active(True, 0.0)
    out = s.tick(DT, DT)
    assert [r.action.kind for r in out.requests] == ["takeoff"]


def test_a_vehicle_state_that_stops_updating_stops_all_output():
    s = session()
    fly_to_hold(s)
    assert s.state == "hold"
    assert s.tick(0.6, DT) == Output()  # nothing arrived since the stamp-0.0 update


def test_a_fresh_vehicle_update_resumes_output():
    s = session()
    fly_to_hold(s)
    assert s.tick(0.6, DT) == Output()
    s.update_vehicle(restamp(AIR, 0.6))
    assert s.tick(0.62, DT).setpoint == Setpoint(position=(0.0, 0.0, -1.5))


def test_a_non_finite_attitude_counts_as_unknown():
    s = session()
    s.set_active(True, 0.0)
    s.update_vehicle(
        VehicleState(position_ned=(0.0, 0.0, -1.0), attitude=(math.nan, 0.0, 0.0, 1.0))
    )
    assert s.vehicle is None
    assert s.tick(DT, DT) == Output()


def test_an_empty_setpoint_is_never_sent(registry):
    # A target reset failure aborts before the vehicle is known: no hold position.
    text = "name: M\ntarget: {badtarget: {}}\nsteps:\n  - goto: {}\n"
    s = Session(parse_mission(text, registry), registry)
    s.set_active(False, -1.0)  # consume the "first activation ever" slot
    s.set_active(True, 0.0)
    s.update_vehicle(AIR)
    assert s.tick(DT, DT).setpoint is None
    assert s.drain_errors()  # the reset traceback is reported


def test_a_non_finite_vehicle_state_counts_as_unknown():
    s = session()
    s.set_active(True, 0.0)
    s.update_vehicle(VehicleState(position_ned=(math.nan, 0.0, 0.0)))
    assert s.vehicle is None
    assert s.tick(DT, DT) == Output()


def test_detections_reach_the_targets_listening_on_that_topic():
    s = session(TAG_MISSION)
    assert s.topics() == ["/front/target_pose"]
    s.update_vehicle(VehicleState(position_ned=(0.0, 0.0, -1.0)))
    s.set_active(True, 0.0)
    s.observe("/target_pose", (0.0, 0.0, 3.0))  # the down camera's topic
    s.tick(DT, DT)
    assert s.state == "search"
    s.observe("/front/target_pose", (0.0, 0.0, 3.0))
    s.tick(2 * DT, DT)
    assert s.state == "hold"


def test_a_detection_before_the_vehicle_state_is_known_is_dropped():
    s = session(TAG_MISSION)
    s.set_active(True, 0.0)
    s.observe("/front/target_pose", (0.0, 0.0, 3.0))  # no attitude to place it
    s.update_vehicle(VehicleState(position_ned=(0.0, 0.0, -1.0)))
    s.tick(DT, DT)
    assert s.state == "search"


def test_a_target_that_raises_in_observe_is_reported_not_raised(registry):
    text = "name: M\ntarget: {grumpy: {}}\nsteps:\n  - goto: {}\n"
    s = Session(parse_mission(text, registry), registry)
    s.set_active(False, -1.0)  # consume the "first activation ever" slot
    s.update_vehicle(AIR)
    s.set_active(True, 0.0)
    s.observe("/grumpy", (0.0, 0.0, 1.0))
    (error,) = s.drain_errors()
    assert "RuntimeError: grumpy" in error


def test_events_are_reported_once():
    s = session()
    s.update_vehicle(GROUND)
    s.set_active(True, 0.0)
    assert s.drain_events() == ["activated: starting from step 1", "takeoff: started"]
    s.tick(DT, DT)
    assert s.drain_events() == ["takeoff: requested takeoff"]
    assert s.drain_events() == []


def test_trajectory_fields_use_nan_for_uncontrolled_axes():
    position, velocity, yaw = trajectory_fields(Setpoint(velocity=(0.1, 0.2, 0.3)))
    assert all(math.isnan(v) for v in position)
    assert velocity == [0.1, 0.2, 0.3]
    assert math.isnan(yaw)
