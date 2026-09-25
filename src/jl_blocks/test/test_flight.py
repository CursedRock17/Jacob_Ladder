from __future__ import annotations

import pytest

from jl_blocks import cli
from jl_blocks.core import MissionError
from jl_blocks.flight import check_flight, load_flight, packages, units

GOOD = "name: {name}\nsteps:\n  - takeoff: {{height: 1.5}}\n  - land: {{}}\n"


def repo(tmp_path, flight_text, missions=None):
    (tmp_path / "missions").mkdir()
    for stem, name in (missions or {"a": "MissionA"}).items():
        (tmp_path / "missions" / f"{stem}.yaml").write_text(GOOD.format(name=name))
    (tmp_path / "config").mkdir()
    path = tmp_path / "config" / "flight.yaml"
    path.write_text(flight_text)
    return path


def test_a_good_flight_file_loads(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vio]\n")
    flight = load_flight(path, tmp_path)
    assert flight.missions == ((tmp_path / "missions" / "a.yaml").resolve(),)
    assert flight.helpers == ("vio",)
    assert check_flight(flight) == []


def test_units_and_packages(tmp_path):
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.yaml\n  - missions/b.yaml\nhelpers: [vio, battery_monitor]\n",
        {"a": "MissionA", "b": "MissionB"},
    )
    flight = load_flight(path, tmp_path)
    assert units(flight) == [
        "jl_mission@a",
        "jl_mission@b",
        "vio",
        "battery_monitor",
    ]
    assert packages(flight) == ["jl_blocks", "jl_mission", "oak_d_visual_odometry"]


def test_unknown_helper_is_rejected_with_the_choices(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vi0]\n")
    with pytest.raises(MissionError, match="unknown helper 'vi0'; did you mean 'vio'"):
        load_flight(path, tmp_path)


def test_a_mission_outside_missions_folder_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - elsewhere/a.yaml\n")
    with pytest.raises(MissionError, match="must be a file in missions/"):
        load_flight(path, tmp_path)


def test_a_missing_mission_file_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/nope.yaml\n")
    with pytest.raises(MissionError, match="missions/nope.yaml: file not found"):
        load_flight(path, tmp_path)


def test_unknown_top_level_key_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelper: [vio]\n")
    with pytest.raises(
        MissionError, match="unknown key 'helper'; did you mean 'helpers'"
    ):
        load_flight(path, tmp_path)


def test_duplicate_names_are_rejected(tmp_path):
    path = repo(
        tmp_path,
        "missions:\n  - missions/a.yaml\n  - missions/b.yaml\n",
        {"a": "Same", "b": "Same"},
    )
    errors = check_flight(load_flight(path, tmp_path))
    assert any("two missions are named 'Same'" in e for e in errors)


def test_a_name_taken_by_an_existing_boot_mode_is_rejected(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\n", {"a": "TakeoffHold"})
    errors = check_flight(load_flight(path, tmp_path))
    assert any(
        "'TakeoffHold' is already registered by another boot service" in e
        for e in errors
    )


def test_a_bad_mission_file_is_reported(tmp_path):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\n")
    (tmp_path / "missions" / "a.yaml").write_text("name: A\nsteps:\n  - holdd: {}\n")
    errors = check_flight(load_flight(path, tmp_path))
    assert any("did you mean 'hold'" in e for e in errors)


def test_cli_check_and_units(tmp_path, capsys):
    path = repo(tmp_path, "missions:\n  - missions/a.yaml\nhelpers: [vio]\n")
    assert cli.main(["flight", "check", str(path), "--root", str(tmp_path)]) == 0
    assert "ok" in capsys.readouterr().out
    assert cli.main(["flight", "units", str(path), "--root", str(tmp_path)]) == 0
    assert capsys.readouterr().out.split() == ["jl_mission@a", "vio"]


def test_cli_check_fails_on_a_bad_file(tmp_path, capsys):
    path = repo(tmp_path, "missions:\n  - missions/nope.yaml\n")
    assert cli.main(["flight", "check", str(path), "--root", str(tmp_path)]) == 1
    assert "FAIL" in capsys.readouterr().out


def test_the_shipped_flight_file_checks_out():
    from pathlib import Path

    root = Path(__file__).resolve().parents[3]
    assert check_flight(load_flight(root / "config" / "flight.yaml", root)) == []


def test_reserved_names_cover_every_flight_code_mode():
    import re
    from pathlib import Path

    from jl_blocks.flight import RESERVED_NAMES

    root = Path(__file__).resolve().parents[3]
    pattern = re.compile(r'k\w+ModeName\[\]\s*=\s*"([^"]+)"')
    found: set[str] = set()
    for hpp in (
        *root.glob("src/precision_land/*.hpp"),
        *root.glob("src/drogue_flight/*.hpp"),
    ):
        found.update(pattern.findall(hpp.read_text(encoding="utf-8")))

    assert found
    assert found <= RESERVED_NAMES
