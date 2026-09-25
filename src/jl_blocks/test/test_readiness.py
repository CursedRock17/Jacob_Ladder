from __future__ import annotations

from jl_blocks.readiness import Check, all_ok, format_report, rate_hz


def test_rate_from_stamps():
    assert rate_hz([0.0, 0.1, 0.2, 0.3, 0.4]) == 10.0
    assert rate_hz([1.0]) == 0.0
    assert rate_hz([]) == 0.0


def test_format_uses_marks_and_aligns():
    lines = format_report(
        [
            Check("FC data arriving", True, "/fmu/out/vehicle_status"),
            Check("VIO publishing", False, "/fmu/in/vehicle_visual_odometry  3 Hz"),
            Check("DDS agent session established", None, "journalctl unavailable"),
        ]
    )
    assert lines[0] == f"✓ {'FC data arriving':<34}/fmu/out/vehicle_status"
    assert lines[1].startswith("✗ VIO publishing")
    assert lines[2].startswith("? DDS agent session established")


def test_unknown_is_not_a_failure_but_false_is():
    assert all_ok([Check("a", True, ""), Check("b", None, "")])
    assert not all_ok([Check("a", True, ""), Check("b", False, "")])


def test_advisory_checks_never_fail_the_report():
    checks = [Check("FC data arriving", True, ""), Check("aruco_tracker", False, "")]
    assert all_ok(checks, advisory={"aruco_tracker"})
