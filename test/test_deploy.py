"""deploy.sh against stub commands: nothing real is pulled, built or enabled."""

from __future__ import annotations

import os
import shlex
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
STUBS = ("git", "colcon", "systemctl", "sudo", "ros2", "journalctl")


@pytest.fixture
def env(tmp_path):
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir()
    log = tmp_path / "calls.log"
    for name in STUBS:
        stub = bin_dir / name
        stub.write_text(
            "#!/bin/bash\n"
            f'echo "{name} $*" >> {shlex.quote(str(log))}\n'
            + f'[[ -n "$STUB_FAIL_PREFIX" && "{name} $*" == "$STUB_FAIL_PREFIX"* ]] && exit "${{STUB_FAIL_RC:-7}}"\n'
            # sudo runs its command, so `sudo systemctl ...` is logged too
            + ('exec "$@"\n' if name == "sudo" else "")
            + (
                'if [[ "$*" == *" --armed "* && -n "$STUB_ARM_RC" ]]; then\n'
                '  echo "${STUB_ARM_OUT:-unknown}"; exit "$STUB_ARM_RC"\n'
                "fi\n"
                'if [[ "$STUB_REQUIRE_SOURCE" == 1 && "$STUB_SOURCED" != 1 ]]; then exit 4; fi\n'
                if name == "ros2"
                else ""
            )
            + (
                'if [[ "$1" == list-unit-files ]]; then echo "$STUB_UNIT_FILES_OUT"; exit 0; fi\n'
                if name == "systemctl"
                else ""
            )
            + f'var="STUB_{name.upper()}_OUT"; [ -n "${{!var}}" ] && echo "${{!var}}"\n'
            + f'var="STUB_{name.upper()}_RC"; exit "${{!var:-0}}"\n'
        )
        stub.chmod(0o755)
    installer = bin_dir / "install_services_stub"
    installer.write_text(
        f'#!/bin/bash\necho "install_services $*" >> {shlex.quote(str(log))}\n'
        'exit "${STUB_INSTALL_RC:-0}"\n'
    )
    installer.chmod(0o755)
    e = dict(os.environ)
    # The check venv's python (with PyYAML) must be the python3 deploy.sh finds.
    e["PATH"] = f"{bin_dir}:{Path(sys.executable).parent}:{e['PATH']}"
    e["JL_DEPLOY_ROOT"] = str(ROOT)
    e["JL_INSTALL_SERVICES"] = str(installer)
    e["JL_DEPLOY_NO_SOURCE"] = "1"
    e["JL_DEPLOY_SETTLE_S"] = "0"
    e["STUB_ROS2_OUT"] = "disarmed"
    e["STUB_SYSTEMCTL_OUT"] = ""
    return e, log


def deploy(env_log, *args, **overrides):
    e, log = env_log
    e = dict(e, **overrides)
    result = subprocess.run(
        ["bash", str(ROOT / "deploy.sh"), *args],
        env=e,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    calls = log.read_text().splitlines() if log.exists() else []
    return result, calls


def test_a_dirty_tree_refuses_before_anything(env):
    result, calls = deploy(env, STUB_GIT_OUT=" M src/x.py")
    assert result.returncode == 1
    assert "uncommitted local changes" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl", "git pull")) for c in calls)


def test_a_bad_mission_stops_before_build_and_touches_no_unit(env, tmp_path):
    bad = tmp_path / "flight.yaml"
    bad.write_text("missions:\n  - missions/nope.yaml\n")
    result, calls = deploy(env, JL_FLIGHT_FILE=str(bad))
    assert result.returncode == 1
    assert "FAIL" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl")) for c in calls)


def test_an_armed_vehicle_refuses(env):
    result, calls = deploy(env, STUB_ROS2_OUT="armed", STUB_ROS2_RC="3")
    assert result.returncode == 1
    assert "armed" in result.stdout
    assert not any(c.startswith(("colcon", "systemctl", "git pull")) for c in calls)


def test_unknown_arm_state_refuses_unless_forced(env):
    result, _ = deploy(env, STUB_ROS2_OUT="unknown", STUB_ROS2_RC="4")
    assert result.returncode == 1
    assert "--force-unknown-arm-state" in result.stdout


def test_a_full_deploy_runs_the_steps_in_order(env):
    result, calls = deploy(env)
    assert result.returncode == 0, result.stdout + result.stderr
    order = [
        next(i for i, c in enumerate(calls) if c.startswith(prefix))
        for prefix in (
            "git pull",
            "colcon build",
            "systemctl enable",
            "ros2 run jl_blocks readiness --names",
        )
    ]
    assert order == sorted(order)
    build = next(c for c in calls if c.startswith("colcon build"))
    assert "--packages-up-to jl_blocks jl_mission oak_d_visual_odometry" in build
    enable = next(c for c in calls if c.startswith("systemctl enable"))
    assert "jl_mission@takeoff_hold_land" in enable and "vio" in enable
    assert not any(
        "dds_agent" in c or "translation_node" in c
        for c in calls
        if c.startswith("systemctl")
    )


def test_a_removed_mission_is_disabled(env):
    running = "jl_mission@old_mission.service loaded active running Mission old_mission"
    result, calls = deploy(env, STUB_SYSTEMCTL_OUT=running)
    assert result.returncode == 0, result.stdout
    assert any(
        c.startswith("systemctl disable --now") and "jl_mission@old_mission" in c
        for c in calls
    )
    assert not any(
        c.startswith("systemctl disable") and "takeoff_hold_land" in c for c in calls
    )


def test_check_only_runs_readiness(env):
    result, calls = deploy(env, "--check")
    assert result.returncode == 0
    assert not any(c.startswith(("git", "colcon", "systemctl")) for c in calls)
    assert any(
        c.startswith("ros2 run jl_blocks readiness --names TakeoffHoldLand")
        for c in calls
    )


def test_dry_run_changes_nothing(env):
    result, calls = deploy(env, "--dry-run")
    assert result.returncode == 0
    assert "would run: colcon build" in result.stdout
    assert not any(
        c.startswith(("git pull", "colcon", "systemctl enable")) for c in calls
    )


def test_bad_option_is_a_usage_error(env):
    result, _ = deploy(env, "--nope")
    assert result.returncode == 2
    assert "usage" in result.stdout.lower()


@pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")
def test_script_is_executable():
    assert os.access(ROOT / "deploy.sh", os.X_OK)


def test_dirty_tree_is_reported_before_an_unavailable_arm_probe(env):
    result, calls = deploy(env, STUB_GIT_OUT=" M x", STUB_ROS2_RC="4")
    assert result.returncode == 1
    assert "uncommitted local changes" in result.stdout
    assert not any(c.startswith("ros2") for c in calls)


def test_dry_run_needs_no_live_ros_or_systemd_even_with_local_changes(env):
    result, calls = deploy(
        env,
        "--dry-run",
        STUB_GIT_OUT=" M x",
        STUB_ROS2_RC="127",
        STUB_SYSTEMCTL_RC="1",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "would run: colcon build" in result.stdout
    assert (
        "would run: ros2 run jl_blocks readiness --names TakeoffHoldLand"
        in result.stdout
    )
    assert calls == []


def test_force_only_overrides_unknown_arm_state(env):
    result, calls = deploy(env, "--force-unknown-arm-state", STUB_ARM_RC="4")
    assert result.returncode == 0, result.stdout + result.stderr
    assert any(c.startswith("git pull") for c in calls)


def test_force_cannot_override_armed(env):
    result, calls = deploy(
        env,
        "--force-unknown-arm-state",
        STUB_ARM_RC="3",
        STUB_ARM_OUT="armed",
    )
    assert result.returncode == 1
    assert not any(c.startswith(("git pull", "colcon", "systemctl")) for c in calls)


@pytest.mark.parametrize(
    ("command", "must_not_run"),
    [
        ("git status", ("git pull", "colcon", "systemctl")),
        ("git pull", ("colcon", "systemctl")),
        ("colcon build", ("install_services", "systemctl")),
        ("systemctl list-units", ("systemctl disable", "systemctl enable")),
        ("systemctl list-unit-files", ("systemctl disable", "systemctl enable")),
        ("systemctl disable", ("systemctl enable", "systemctl restart")),
        (
            "systemctl enable",
            ("systemctl restart", "ros2 run jl_blocks readiness --names"),
        ),
        ("systemctl restart", ("ros2 run jl_blocks readiness --names",)),
    ],
)
def test_a_failed_step_stops_deploy(env, command, must_not_run):
    result, calls = deploy(env, STUB_FAIL_PREFIX=command)
    assert result.returncode == 1, result.stdout + result.stderr
    assert not any(c.startswith(must_not_run) for c in calls)


def test_failed_installer_stops_before_units(env):
    result, calls = deploy(env, STUB_INSTALL_RC="7")
    assert result.returncode == 1
    assert not any(c.startswith("systemctl") for c in calls)


def test_removed_enabled_but_unloaded_mission_is_disabled(env):
    result, calls = deploy(
        env,
        STUB_UNIT_FILES_OUT="jl_mission@unloaded.service enabled enabled",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert "systemctl disable --now jl_mission@unloaded" in calls


def test_check_rejects_invalid_config_before_ros(env, tmp_path):
    bad = tmp_path / "flight.yaml"
    bad.write_text("missions:\n  - missions/nope.yaml\n")
    result, calls = deploy(env, "--check", JL_FLIGHT_FILE=str(bad))
    assert result.returncode == 1
    assert "FAIL" in result.stdout
    assert calls == []


def test_readiness_failure_is_a_deploy_failure(env):
    result, calls = deploy(env, "--check", STUB_ROS2_RC="7")
    assert result.returncode == 1
    assert any(c.startswith("ros2 run jl_blocks readiness --names") for c in calls)


def test_sources_workspace_before_arm_probe_with_nounset_relaxed(env, tmp_path):
    for name in ("src", "jl_env.sh"):
        (tmp_path / name).symlink_to(ROOT / name)
    for name in ("config", "missions"):
        shutil.copytree(ROOT / name, tmp_path / name)
    (tmp_path / "install").mkdir()
    (tmp_path / "install" / "setup.bash").write_text(
        ': "$UNSET_ROS_SETUP_VARIABLE"\nexport STUB_SOURCED=1\n'
    )
    result, calls = deploy(
        env,
        JL_DEPLOY_ROOT=str(tmp_path),
        JL_WS_ROOT=str(tmp_path),
        JL_ROS_DISTRO="deploy_test_missing",
        JL_DEPLOY_NO_SOURCE="0",
        STUB_REQUIRE_SOURCE="1",
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert any(c.startswith("git pull") for c in calls)
