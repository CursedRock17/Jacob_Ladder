"""Defining, loading, checking and running missions, and the command line."""

from __future__ import annotations

from pathlib import Path

import pytest
from fake_backend import FakeBackend

from jl_flight import AprilTag, YoloDrogue, mission
from jl_flight.airframes import X500_RGBD
from jl_flight.cli import main
from jl_flight.missions import MissionError, load, run

REPO = Path(__file__).resolve().parents[3]
MISSION_FILES = sorted((REPO / "missions").glob("*.py"))


def write(tmp_path: Path, body: str) -> Path:
    path = tmp_path / "m.py"
    path.write_text("from jl_flight import AprilTag, mission\n" + body)
    return path


@pytest.mark.parametrize("name", ["", "1Bad", "has space", "x" * 25, "dash-name"])
def test_names_follow_px4s_rule(name: str) -> None:
    with pytest.raises(MissionError, match="1-24 letters"):
        mission(name)


def test_unknown_options_are_rejected() -> None:
    with pytest.raises(MissionError, match="unknown mission option"):
        mission("M", take_off_on_enter=True)


def test_needs_takes_declarations() -> None:
    with pytest.raises(MissionError, match="perception declarations"):
        mission("M", needs=["/front/target_pose"])  # ty: ignore[invalid-argument-type]


def test_load_finds_the_one_mission(tmp_path: Path) -> None:
    path = write(
        tmp_path, '@mission("M", needs=[AprilTag(id=1, size=0.2)])\ndef m(d): pass\n'
    )
    spec = load(path)
    assert spec.name == "M"
    assert spec.path == str(path)
    assert spec.needs == (AprilTag(id=1, size=0.2),)


def test_a_file_needs_exactly_one_mission(tmp_path: Path) -> None:
    with pytest.raises(MissionError, match="defines no mission"):
        load(write(tmp_path, "x = 1\n"))
    two = '@mission("A")\ndef a(d): pass\n@mission("B")\ndef b(d): pass\n'
    with pytest.raises(MissionError, match="defines 2 missions"):
        load(write(tmp_path, two))


def test_an_import_error_names_the_file(tmp_path: Path) -> None:
    with pytest.raises(MissionError, match="m.py: NameError"):
        load(write(tmp_path, "nope\n"))


def test_helpers_are_not_started_twice() -> None:
    tag = AprilTag(id=0, size=0.3)

    @mission("M", needs=[tag, tag])
    def m(d) -> None:
        pass

    assert len(m.helpers(X500_RGBD)) == 1


def test_a_camera_the_airframe_lacks_is_caught_before_flying() -> None:
    @mission("M", needs=[YoloDrogue(camera="down")])
    def m(d) -> None:
        pass

    with pytest.raises(MissionError, match="has no camera 'down'"):
        m.check(X500_RGBD)


@pytest.mark.parametrize("path", MISSION_FILES, ids=lambda p: p.name)
def test_every_mission_in_the_repo_loads_and_checks(path: Path) -> None:
    load(path).check(X500_RGBD)


def test_run_flies_a_repo_mission_end_to_end() -> None:
    spec = load(REPO / "missions" / "land_before_apriltag.py")
    (tag,) = spec.needs
    m = X500_RGBD.camera("front").mount
    backend = FakeBackend(camera_offset=(m.forward, m.right, m.down))
    backend.put_tag(tag.topic, (6.0, 0.0, -1.2))
    run(spec, airframe=X500_RGBD, backend=backend, log=lambda m: None)
    assert backend.calls == ["connect", "takeoff 1.5", "land"]
    assert backend.position[0] == pytest.approx(4.5, abs=0.2)


def test_run_waits_for_a_fresh_selection_when_asked() -> None:
    spec = load(REPO / "missions" / "land_before_apriltag.py")

    class SelectedAgainLater(FakeBackend):
        def sleep(self, seconds: float) -> None:
            super().sleep(seconds)
            if self.t > 2.0 and self.selection_count == 1:
                self.select()

    backend = SelectedAgainLater(active=True)
    backend.put_tag(spec.needs[0].topic, (6.0, 0.0, -1.2))
    run(
        spec,
        airframe=X500_RGBD,
        backend=backend,
        fresh_selection=True,
        log=lambda m: None,
    )
    assert backend.published, "it never flew"
    first_setpoint_at = backend.published[0][0]
    assert (
        first_setpoint_at > 2.0
    )  # nothing was sent before the mode was selected again


def test_cli_check_name_and_helpers(capsys: pytest.CaptureFixture[str]) -> None:
    path = str(REPO / "missions" / "drogue_approach.py")
    assert main(["check", path, "--airframe", "x500_rgbd"]) == 0
    assert "ok" in capsys.readouterr().out
    assert main(["name", path]) == 0
    assert capsys.readouterr().out.strip() == "DrogueApproach"
    assert main(["helpers", path, "--airframe", "x500_rgbd"]) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert [line.split()[3] for line in lines] == [
        "drogue_detection_node",
        "pose_estimation_node",
    ]


def test_cli_reports_a_bad_file(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    assert main(["check", str(tmp_path / "missing.py")]) == 1
    assert "file not found" in capsys.readouterr().err


def test_cli_run_passes_report_through(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import jl_flight.missions as missions_module

    seen: dict[str, object] = {}

    def fake_run(spec, **kwargs) -> None:
        seen.update(kwargs)

    monkeypatch.setattr(missions_module, "run", fake_run)
    path = str(REPO / "missions" / "apriltag_approach.py")
    assert main(["run", path, "--report", "0.25", "--airframe", "x500_rgbd"]) == 0
    assert seen["report_every"] == 0.25
    assert seen["airframe"] == "x500_rgbd"
