from __future__ import annotations

from jl_blocks.testing.expect import BandHold, StepOrder, feed_gated, report


def fed(expected, events):
    order = StepOrder(expected)
    for event in events:
        order.feed(event)
    return order


def test_steps_started_in_order_pass():
    order = fed(
        ["takeoff", "search", "track"],
        [
            "activated: starting from step 1",
            "takeoff: started",
            "takeoff: requested takeoff",
            "takeoff: done",
            "search: started",
            "search: done",
            "track: started",
        ],
    )
    assert order.ok
    assert order.missing() == []


def test_out_of_order_steps_do_not_pass():
    order = fed(
        ["takeoff", "search", "track"],
        ["takeoff: started", "track: started", "search: started"],
    )
    assert not order.ok
    assert order.missing() == ["track"]


def test_only_started_events_count():
    order = fed(["takeoff"], ["takeoff: requested takeoff", "takeoff: done"])
    assert not order.ok


def test_repeated_steps_still_match_the_first_time_through():
    order = fed(
        ["takeoff", "search", "track"],
        [
            "takeoff: started",
            "search: started",
            "track: started",
            "track: failed (block reported failure) -> search",
            "search: started",
            "track: started",
        ],
    )
    assert order.ok


def test_finished_is_recorded():
    order = fed(["hold"], ["hold: started", "hold: done", "mission finished"])
    assert order.finished


def test_an_abort_is_a_failure_naming_the_missing_steps():
    order = fed(
        ["takeoff", "search", "track"],
        [
            "takeoff: started",
            "search: started",
            "search: failed (timeout after 30 s); no on_fail -> hold, then land",
            "aborted: held, now landing",
        ],
    )
    assert order.aborted
    lines, ok = report(order, need_finished=False, band=None)
    assert not ok
    assert "FAIL steps started in order: takeoff search track (missing: track)" in lines
    assert "FAIL mission did not abort" in lines


def test_mission_finished_ends_it_even_without_all_steps():
    order = fed(
        ["takeoff", "search", "track"],
        [
            "takeoff: started",
            "search: started",
            "search: failed (timeout after 30 s) -> land",
            "land: started",
            "mission finished",
        ],
    )
    assert order.over
    lines, ok = report(order, need_finished=True, band=None)
    assert not ok
    assert "FAIL steps started in order: takeoff search track (missing: track)" in lines


def test_deactivated_is_recorded_and_ends_it():
    order = fed(["takeoff"], ["takeoff: started", "deactivated"])
    assert order.deactivated
    assert order.over
    lines, ok = report(order, need_finished=False, band=None)
    assert not ok
    assert "FAIL mission was not deactivated" in lines


def test_not_over_while_mid_mission():
    order = fed(["takeoff", "search"], ["takeoff: started"])
    assert not order.over


def test_reactivation_starts_the_order_over():
    order = fed(
        ["takeoff", "search", "track"],
        [
            "activated: starting from step 1",
            "takeoff: started",
            "search: started",
            "deactivated",
            "activated: starting from step 1",
            "track: started",
        ],
    )
    assert not order.ok
    assert order.missing() == ["takeoff", "search", "track"]


def test_band_ignores_samples_before_every_step_has_started():
    order = fed(["takeoff", "track"], ["takeoff: started"])
    hold = band()
    t = 0.0
    while t <= 6.0 + 1e-9:
        feed_gated(order, hold, (0.0, 0.0, 3.0), t)
        t += 0.1
    assert not hold.held
    assert hold.last is None

    order.feed("track: started")
    t2 = t
    while t2 <= t + 6.0 + 1e-9:
        feed_gated(order, hold, (0.0, 0.0, 3.0), t2)
        t2 += 0.1
    assert hold.held


def band(**kwargs):
    return BandHold((0.0, 0.0, 3.0), 0.25, 5.0, **kwargs)


def feed_steady(hold, position, start, seconds, step=0.1):
    t = start
    while t <= start + seconds + 1e-9:
        hold.feed(position, t)
        t += step
    return t


def test_holding_inside_the_band_for_hold_s_passes():
    hold = band()
    feed_steady(hold, (0.1, -0.1, 3.2), 0.0, 4.9)
    assert not hold.held
    feed_steady(hold, (0.1, -0.1, 3.2), 5.0, 0.0)
    assert hold.held


def test_one_axis_outside_the_band_is_outside():
    hold = band()
    feed_steady(hold, (0.3, 0.0, 3.0), 0.0, 6.0)
    assert not hold.held
    assert hold.last == (0.3, 0.0, 3.0)


def test_leaving_the_band_restarts_the_hold():
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 3.0)
    hold.feed((0.0, 0.0, 3.5), 3.1)
    feed_steady(hold, (0.0, 0.0, 3.0), 3.2, 4.9)
    assert not hold.held
    feed_steady(hold, (0.0, 0.0, 3.0), 8.2, 0.0)
    assert hold.held


def test_a_gap_in_detections_restarts_the_hold():
    hold = band()
    hold.feed((0.0, 0.0, 3.0), 0.0)
    hold.feed((0.0, 0.0, 3.0), 5.0)  # nothing in between
    assert not hold.held
    feed_steady(hold, (0.0, 0.0, 3.0), 5.1, 5.0)
    assert hold.held


def test_held_stays_true_once_reached():
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 5.0)
    hold.feed((5.0, 5.0, 5.0), 5.2)
    assert hold.held


def test_report_all_passing():
    order = fed(["hold"], ["hold: started", "mission finished"])
    hold = band()
    feed_steady(hold, (0.0, 0.0, 3.0), 0.0, 5.0)
    lines, ok = report(order, need_finished=True, band=hold)
    assert ok
    assert lines == [
        "PASS steps started in order: hold",
        "PASS mission did not abort",
        "PASS mission was not deactivated",
        "PASS mission finished",
        "PASS camera saw the target within 0.25 m of (0.00, 0.00, 3.00) for 5 s",
    ]


def test_report_never_seen_and_not_finished():
    order = fed(["hold"], ["hold: started"])
    lines, ok = report(order, need_finished=True, band=band())
    assert not ok
    assert "FAIL mission finished" in lines
    assert (
        "FAIL camera saw the target within 0.25 m of (0.00, 0.00, 3.00) for 5 s "
        "(never saw it)"
    ) in lines


def test_report_last_seen_position():
    hold = band()
    hold.feed((0.5, 0.0, 2.0), 1.0)
    lines, _ = report(fed(["hold"], ["hold: started"]), False, hold)
    assert lines[-1].endswith("(last seen at (0.50, 0.00, 2.00))")
