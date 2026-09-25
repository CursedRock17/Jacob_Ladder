from __future__ import annotations

from jl_blocks import cli
from jl_blocks.core import (
    REGISTRY,
    Engine,
    Setpoint,
    VehicleState,
    load_mission,
    parse_mission,
)


def test_shipped_blocks_are_registered():
    assert REGISTRY.get("hold") is not None
    assert REGISTRY.get("position") is not None


def test_hold_keeps_the_position_from_when_the_step_started():
    engine = Engine(parse_mission("name: M\nsteps:\n  - hold: {duration: 1}\n"))
    engine.start(0.0)
    first = engine.tick(VehicleState(position_ned=(1.0, 1.0, -2.0)), 0.02, 0.02)
    drifted = engine.tick(VehicleState(position_ned=(1.3, 1.0, -2.0)), 0.04, 0.02)
    assert first == drifted == Setpoint(position=(1.0, 1.0, -2.0))


def test_hold_with_duration_finishes():
    engine = Engine(parse_mission("name: M\nsteps:\n  - hold: {duration: 0.5}\n"))
    engine.start(0.0)
    here = VehicleState(position_ned=(0.0, 0.0, -1.0))
    engine.tick(here, 0.4, 0.02)
    assert not engine.finished
    engine.tick(here, 0.6, 0.02)
    assert engine.finished


def test_check_passes_a_good_file(tmp_path, capsys):
    path = tmp_path / "hover.yaml"
    path.write_text("name: Hover\nsteps:\n  - hold: {duration: 10}\n")
    assert cli.main(["check", str(path)]) == 0
    assert f"ok   {path}  (Hover, 1 steps)" in capsys.readouterr().out


def test_check_fails_and_explains_a_bad_file(tmp_path, capsys):
    path = tmp_path / "bad.yaml"
    path.write_text("name: Bad\nsteps:\n  - hold: {duraton: 10}\n")
    assert cli.main(["check", str(path)]) == 1
    out = capsys.readouterr().out
    assert f"FAIL {path}" in out
    assert "unknown param 'duraton' for hold; did you mean 'duration'?" in out


def test_check_reports_a_missing_file(tmp_path, capsys):
    assert cli.main(["check", str(tmp_path / "nope.yaml")]) == 1
    assert "file not found" in capsys.readouterr().out


def test_check_reports_an_unreadable_directory(tmp_path, capsys):
    assert cli.main(["check", str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "FAIL" in out
    assert "cannot read file:" in out


def test_check_reports_a_file_that_cannot_be_decoded(tmp_path, capsys):
    path = tmp_path / "bad.yaml"
    path.write_bytes(b"\xff\xfe\x00bad")
    assert cli.main(["check", str(path)]) == 1
    out = capsys.readouterr().out
    assert "FAIL" in out
    assert "cannot read file:" in out


def test_load_mission_reads_a_real_file(tmp_path):
    path = tmp_path / "hover.yaml"
    path.write_text("name: Hover\nsteps:\n  - hold: {duration: 1}\n")
    spec = load_mission(path)
    assert spec.name == "Hover"
