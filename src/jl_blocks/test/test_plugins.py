from __future__ import annotations

from jl_blocks import cli
from jl_blocks import library  # noqa: F401  (registers the shipped blocks)
from jl_blocks.core import REGISTRY, load_block_files

BLOCK = """
from jl_blocks.core import Mission, Setpoint, StepContext, block


@block("{name}")
class Wiggle(Mission):
    def step(self, ctx: StepContext) -> Setpoint:
        return Setpoint(position=ctx.vehicle.position_ned)
"""


def test_blocks_in_a_folder_are_registered(tmp_path):
    (tmp_path / "wiggle.py").write_text(BLOCK.format(name="plugin_folder_wiggle"))
    assert load_block_files([str(tmp_path)]) == []
    assert REGISTRY.get("plugin_folder_wiggle") is not None


def test_a_single_file_works_too(tmp_path):
    path = tmp_path / "wiggle.py"
    path.write_text(BLOCK.format(name="plugin_file_wiggle"))
    assert load_block_files([str(path)]) == []
    assert REGISTRY.get("plugin_file_wiggle") is not None


def test_a_broken_file_is_reported_not_raised(tmp_path):
    (tmp_path / "broken.py").write_text("def oops(:\n")
    errors = load_block_files([str(tmp_path)])
    assert len(errors) == 1
    assert "broken.py" in errors[0]
    assert "SyntaxError" in errors[0]


def test_reusing_a_shipped_name_is_reported(tmp_path):
    (tmp_path / "hold2.py").write_text(BLOCK.format(name="hold"))
    (error,) = load_block_files([str(tmp_path)])
    assert "block 'hold' is already registered" in error


def test_a_missing_path_is_reported(tmp_path):
    (error,) = load_block_files([str(tmp_path / "nope")])
    assert "no such file or folder" in error


def test_check_uses_your_own_blocks(tmp_path, capsys):
    blocks = tmp_path / "my_blocks"
    blocks.mkdir()
    (blocks / "wiggle.py").write_text(BLOCK.format(name="plugin_check_wiggle"))
    mission = tmp_path / "m.yaml"
    mission.write_text("name: M\nsteps:\n  - plugin_check_wiggle: {}\n")
    assert cli.main(["check", "--blocks", str(blocks), str(mission)]) == 0


def test_check_fails_on_a_broken_block_file(tmp_path, capsys):
    (tmp_path / "broken.py").write_text("def oops(:\n")
    mission = tmp_path / "m.yaml"
    mission.write_text("name: M\nsteps:\n  - hold: {}\n")
    assert cli.main(["check", "--blocks", str(tmp_path), str(mission)]) == 1
    assert "FAIL blocks" in capsys.readouterr().out
