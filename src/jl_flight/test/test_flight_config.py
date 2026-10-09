"""config/flight.yaml: what an aircraft registers at boot, as deploy.sh reads it."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from jl_flight.cli import main
from jl_flight.flight_config import (
    RESERVED_NAMES,
    load_flight,
    load_missions,
    packages,
    topics,
    units,
)
from jl_flight.missions import MissionError

REPO = Path(__file__).resolve().parents[3]
PLAIN = 'from jl_flight import mission\n\n@mission("{name}")\ndef m(drone):\n    drone.take_off(1.5)\n'
TAGGED = (
    "from jl_flight import AprilTag, mission\n\n"
    'TAG = AprilTag(id=0, size=0.3)\n\n@mission("{name}", needs=[TAG])\ndef m(drone):\n    pass\n'
)


def repo(
    tmp_path: Path, flight_text: str, missions: dict[str, str] | None = None
) -> Path:
    (tmp_path / "missions").mkdir()
    for stem, source in (missions or {"a": PLAIN.format(name="MissionA")}).items():
        (tmp_path / "missions" / f"{stem}.py").write_text(source)
    (tmp_path / "config").mkdir()
    path = tmp_path / "config" / "flight.yaml"
    path.write_text(flight_text)
    return path


def test_a_good_flight_file_loads(tmp_path: Path) -> None:
    path = repo(
        tmp_path, "airframe: x500_rgbd\nmissions:\n  - missions/a.py\nhelpers: [vio]\n"
    )
    flight = load_flight(path, tmp_path)
    assert flight.airframe == "x500_rgbd"
    assert flight.missions == ((tmp_path / "missions" / "a.py").resolve(),)
    assert flight.helpers == ("vio",)
    specs, errors = load_missions(flight)
    assert errors == [] and [s.name for s in specs] == ["MissionA"]


def test_the_airframe_defaults_to_generic(tmp_path: Path) -> None:
    path = repo(tmp_path, "missions:\n  - missions/a.py\n")
    assert load_flight(path, tmp_path).airframe == "generic"


def test_units_packages_and_topics(tmp_path: Path) -> None:
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.py\n  - missions/b.py\nhelpers: [vio, battery_monitor]\n",
        {"a": PLAIN.format(name="MissionA"), "b": TAGGED.format(name="MissionB")},
    )
    flight = load_flight(path, tmp_path)
    specs, _ = load_missions(flight)
    assert units(flight) == ["jl_mission@a", "jl_mission@b", "vio", "battery_monitor"]
    # The tracker the AprilTag mission declares is built too.
    assert packages(flight, specs) == [
        "jl_flight",
        "jl_mission",
        "oak_d_visual_odometry",
        "aruco_tracker",
    ]
    assert topics(specs) == ["/jl/perception/front/apriltag_36h11_0"]


@pytest.mark.parametrize(
    ("text", "match"),
    [
        (
            "missions:\n  - missions/a.py\nhelpers: [vi0]\n",
            "unknown helper 'vi0'; did you mean 'vio'",
        ),
        (
            "missions:\n  - elsewhere/a.py\n",
            "must be a file in missions/ ending in .py",
        ),
        (
            "missions:\n  - missions/a.yaml\n",
            "must be a file in missions/ ending in .py",
        ),
        ("missions:\n  - missions/nope.py\n", "missions/nope.py: file not found"),
        (
            "missions:\n  - missions/a.py\nhelper: [vio]\n",
            "unknown key 'helper'; did you mean 'helpers'",
        ),
        (
            "airframe: x500_rgdb\nmissions:\n  - missions/a.py\n",
            "did you mean 'x500_rgbd'",
        ),
        ("missions: []\n", "non-empty list"),
    ],
)
def test_bad_flight_files_are_rejected(tmp_path: Path, text: str, match: str) -> None:
    with pytest.raises(MissionError, match=match):
        load_flight(repo(tmp_path, text), tmp_path)


def test_duplicate_names_are_rejected(tmp_path: Path) -> None:
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.py\n  - missions/b.py\n",
        {"a": PLAIN.format(name="Same"), "b": PLAIN.format(name="Same")},
    )
    _, errors = load_missions(load_flight(path, tmp_path))
    assert any("two missions are named 'Same'" in e for e in errors)


def test_a_name_taken_by_an_existing_boot_mode_is_rejected(tmp_path: Path) -> None:
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.py\n",
        {"a": PLAIN.format(name="TakeoffHold")},
    )
    _, errors = load_missions(load_flight(path, tmp_path))
    assert any("already registered by another boot service" in e for e in errors)


def test_a_mission_the_airframe_cannot_fly_is_reported(tmp_path: Path) -> None:
    down = TAGGED.replace("size=0.3)", 'size=0.3, camera="down")').format(name="Down")
    path = repo(
        tmp_path, "airframe: x500_rgbd\nmissions:\n  - missions/a.py\n", {"a": down}
    )
    _, errors = load_missions(load_flight(path, tmp_path))
    assert any("has no camera 'down'" in e for e in errors)


def test_a_broken_mission_file_is_reported(tmp_path: Path) -> None:
    path = repo(tmp_path, "missions:\n  - missions/a.py\n", {"a": "nope\n"})
    _, errors = load_missions(load_flight(path, tmp_path))
    assert any("NameError" in e for e in errors)


def test_cli(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = str(repo(tmp_path, "missions:\n  - missions/a.py\nhelpers: [vio]\n"))
    root = ["--root", str(tmp_path)]
    assert main(["flight", "check", path, *root]) == 0
    assert "ok" in capsys.readouterr().out
    assert main(["flight", "units", path, *root]) == 0
    assert capsys.readouterr().out.split() == ["jl_mission@a", "vio"]
    assert main(["flight", "names", path, *root]) == 0
    assert capsys.readouterr().out.split() == ["MissionA"]
    assert main(["flight", "airframe", path, *root]) == 0
    assert capsys.readouterr().out.strip() == "generic"


def test_cli_check_fails_on_a_bad_file(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path = repo(tmp_path, "missions:\n  - missions/nope.py\n")
    assert main(["flight", "check", str(path), "--root", str(tmp_path)]) == 1
    assert "FAIL" in capsys.readouterr().out


def test_the_shipped_flight_file_checks_out() -> None:
    flight = load_flight(REPO / "config" / "flight.yaml", REPO)
    assert load_missions(flight)[1] == []


def test_reserved_names_cover_every_flight_code_mode() -> None:
    pattern = re.compile(r'k\w+ModeName\[\]\s*=\s*"([^"]+)"')
    found: set[str] = set()
    for hpp in (
        *REPO.glob("src/precision_land/*.hpp"),
        *REPO.glob("src/drogue_flight/*.hpp"),
    ):
        found.update(pattern.findall(hpp.read_text(encoding="utf-8")))
    assert found
    assert found <= RESERVED_NAMES
