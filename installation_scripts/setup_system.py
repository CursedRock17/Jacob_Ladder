#!/usr/bin/env python3
"""Reproducible, resumable system setup for a Jacob's Ladder Jetson.

=============================================================================
USE AT YOUR OWN RISK -- THIS SCRIPT HAS NEVER BEEN RUN END TO END
=============================================================================

It was reconstructed after the fact from the shell history of one machine that
was set up by hand, plus the steps taken in the session that wrote it. It has
only ever been validated with `--list` and `--dry-run` against a box that was
*already* fully configured. Every `check()` has been exercised; most `action()`
bodies have not.

That means, concretely:

  * Steps satisfied on the reference machine -- `ark-os`, `imu-driver`,
    `apt-base` and others -- have never actually executed from this script.
  * The ordering is inferred from an interactive history in which the operator
    backtracked, retried and rebooted. Real dependencies may differ.
  * `ark-os` runs a third-party installer that reconfigures networking, systemd
    units and the flight stack. This script does not constrain what it does.
  * Several steps are destructive or hard to undo (apt upgrades, a udev rule,
    systemd units, disabling ARK's DDS agent).

Run it on a board you are willing to reflash, not on a drone you are about to
fly. Start with `--list` and `--dry-run`, then work through it a step at a time
with `--only`. Read what a step will do before letting it run unattended.

Please fix this docstring as steps get proven on real hardware.

=============================================================================

Reconstructed from the shell history of the reference build (an ARK Electronics
Jetson Orin NX image), starting at the first command run on a freshly flashed
board -- `sudo apt update`. Everything before that point is what the
manufacturer ships in the image and is deliberately not reproduced here.

Design notes
------------
* **Stdlib only.** This has to run on a bare image before uv, pip packages or a
  venv exist. Do not add third-party imports.
* **Run as your normal user, not as root.** Individual steps escalate with sudo
  where they need to. Several steps (uv sync, colcon build) produce files that
  must be owned by you, and running the whole script as root silently breaks
  them.
* **Idempotent.** Every step has a `check` that reports whether it is already
  satisfied, so re-running is cheap and safe. `check` is the source of truth;
  the state file is only a fallback for steps nothing can probe.
* **Resumable across reboots.** Three steps require a reboot. Completed steps
  are recorded in the state file, so after rebooting you re-run the same command
  and it picks up where it left off.

Usage
-----
    ./setup_system.py --list                 # show all steps and their status
    ./setup_system.py --dry-run              # print what would run
    ./setup_system.py                        # run everything not yet satisfied
    ./setup_system.py --only opencv          # run one step (ignores its check)
    ./setup_system.py --from workspace       # run from this step onward
    ./setup_system.py --skip brave --skip lazygit
    ./setup_system.py --include-optional     # also run developer-convenience steps

Unattended operation
--------------------
Steps need sudo, and sudo cannot prompt when there is no terminal. Supply the
password out of band and the script will run without a human:

    ./setup_system.py --unattended --sudo-password-file ~/.jl-sudo    # file, mode 0600
    JL_SUDO_PASSWORD=... ./setup_system.py --unattended               # environment

`--unattended` also skips the steps that need a human at the keyboard (the
jetson-io TUI, `gh auth login`) instead of blocking on them forever; it reports
which ones it skipped so you can do them afterwards.

There is deliberately no `--sudo-password=` flag. Command-line arguments are
visible to every user on the box via `ps` and land in your shell history. A file
is the best of the options here: create it with `install -m 600 /dev/null
~/.jl-sudo`, write the password into it, and delete it when you are done. The
environment variable is convenient but is readable via /proc for your own
processes and tends to leak into logs and CI transcripts.

The password is never passed on a command line internally either. It is written
to a mode-0600 file inside a mode-0700 private directory, exposed to sudo
through SUDO_ASKPASS, and both are overwritten and removed when the script
exits.
"""

from __future__ import annotations

import argparse
import atexit
import getpass
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

# --------------------------------------------------------------------------- #
# Paths and constants
# --------------------------------------------------------------------------- #

HOME = Path.home()
# This file lives in <workspace>/installation_scripts/, so the workspace root is
# its parent. Mirrors how jl_env.sh resolves paths -- no hardcoded home dirs.
WS_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = WS_ROOT / "installation_scripts"
STATE_FILE = HOME / ".jacob_ladder_setup_state.json"

ARK_OS_DIR = HOME / "ARK-OS"
ROS_DISTRO = "humble"

# NOTE: PX4-Autopilot is deliberately NOT cloned here.
#
# The reference build never had it. The history contains exactly two clones --
# ARK-OS and Jacob_Ladder -- and no PX4 clone at all. The `cd PX4-Autopilot`
# at history line 108 failed (that directory has never existed on the box), so
# the `git status` / `git checkout v1.16.0` that follow it actually ran against
# the Jacob_Ladder repo and failed too.
#
# That matches how the hardware works: the flight controller runs the PX4
# firmware, and the companion computer talks to it over XRCE-DDS. You only need
# a PX4-Autopilot checkout for the Gazebo simulation workflow on a dev machine,
# which is not what this script sets up. jl_env.sh still resolves JL_PX4_DIR for
# the scripts that do care; it is simply not part of drone setup.
OPENCV_VERSION = "4.10.0"
OPENCV_CMAKE_DIR = "/usr/local/lib/cmake/opencv4"

# Leave a core free so the box stays usable, and cap it: the CUDA modules are
# memory-hungry and an 8-core Orin NX with 16 GB gets OOM-killed at -j8.
OPENCV_JOBS = max(1, min(os.cpu_count() or 4, 6))

BOLD, GREEN, YELLOW, RED, DIM, RESET = (
    "\033[1m", "\033[32m", "\033[33m", "\033[31m", "\033[2m", "\033[0m"
)
if not sys.stdout.isatty():
    BOLD = GREEN = YELLOW = RED = DIM = RESET = ""


# --------------------------------------------------------------------------- #
# Unattended sudo
#
# sudo cannot prompt without a terminal, so an unattended run needs the password
# supplied out of band. SUDO_ASKPASS is the mechanism designed for this: sudo -A
# executes a helper program and reads the password from its stdout. That keeps
# the secret off every command line, including our own.
#
# Note this only covers `sudo` invocations *this script* issues, which is why
# they are rewritten to `sudo -A`. Steps that hand a whole shell script to sudo
# (`sudo bash base_tools.sh`) are already covered: the nested sudo calls inside
# those scripts run as root, where sudo needs no password at all.
# --------------------------------------------------------------------------- #

_ASKPASS_DIR: Optional[Path] = None


def _shred(path: Path) -> None:
    """Overwrite before unlinking so the secret does not linger in free blocks."""
    try:
        size = path.stat().st_size
        with open(path, "r+b", buffering=0) as fh:
            fh.write(b"\0" * size)
            fh.flush()
            os.fsync(fh.fileno())
    except OSError:
        pass
    path.unlink(missing_ok=True)


def _cleanup_askpass() -> None:
    global _ASKPASS_DIR
    if _ASKPASS_DIR is None or not _ASKPASS_DIR.exists():
        return
    for child in _ASKPASS_DIR.iterdir():
        _shred(child)
    try:
        _ASKPASS_DIR.rmdir()
    except OSError:
        pass
    _ASKPASS_DIR = None


def enable_unattended_sudo(password: str) -> None:
    """Install a SUDO_ASKPASS helper so sudo never needs a terminal."""
    global _ASKPASS_DIR
    _ASKPASS_DIR = Path(tempfile.mkdtemp(prefix="jl-setup-"))
    os.chmod(_ASKPASS_DIR, 0o700)
    atexit.register(_cleanup_askpass)

    # The password lives in its own file rather than inside the helper script,
    # so no quoting or delimiter in the password can break the helper.
    pw_file = _ASKPASS_DIR / "pw"
    pw_file.touch(mode=0o600)
    pw_file.write_text(password + "\n")
    os.chmod(pw_file, 0o600)

    helper = _ASKPASS_DIR / "askpass.sh"
    helper.write_text(f'#!/bin/sh\nexec cat {pw_file}\n')
    os.chmod(helper, 0o700)

    os.environ["SUDO_ASKPASS"] = str(helper)

    # Fail fast on a wrong password rather than 40 minutes into an OpenCV build.
    if not ok("sudo -A -n true 2>/dev/null || sudo -A true"):
        _cleanup_askpass()
        raise SystemExit(
            f"{RED}error{RESET}: the supplied sudo password was rejected."
        )


def resolve_sudo_password(args) -> Optional[str]:
    """Password from --sudo-password-file, then $JL_SUDO_PASSWORD, then a prompt."""
    if args.sudo_password_file:
        path = Path(args.sudo_password_file).expanduser()
        if not path.exists():
            raise SystemExit(f"{RED}error{RESET}: {path} does not exist")
        mode = path.stat().st_mode & 0o077
        if mode:
            print(f"{YELLOW}warning{RESET}: {path} is readable by others "
                  f"(mode {oct(path.stat().st_mode & 0o777)}); "
                  f"consider chmod 600")
        return path.read_text().splitlines()[0]

    env_pw = os.environ.get("JL_SUDO_PASSWORD")
    if env_pw:
        return env_pw

    if args.unattended:
        raise SystemExit(
            f"{RED}error{RESET}: --unattended needs a sudo password. Use "
            f"--sudo-password-file or $JL_SUDO_PASSWORD (see --help)."
        )
    return None


# --------------------------------------------------------------------------- #
# Shell helpers
# --------------------------------------------------------------------------- #

# Matches a bare `sudo` token so it can become `sudo -A`. Avoids touching an
# already-rewritten `sudo -A`, and the lookbehind keeps it from firing on
# things like `no-sudo` or a path ending in "sudo".
_SUDO_RE = re.compile(r"(?<![\w./-])sudo (?!-A\b)")


def run(cmd: str, *, check: bool = True, cwd: Optional[Path] = None,
        env: Optional[dict] = None) -> subprocess.CompletedProcess:
    """Run a shell command, streaming its output."""
    if _ASKPASS_DIR is not None:
        cmd = _SUDO_RE.sub("sudo -A ", cmd)
    print(f"{DIM}$ {cmd}{RESET}")
    merged = {**os.environ, **(env or {})}
    return subprocess.run(cmd, shell=True, cwd=cwd, env=merged, check=check)


def quiet(cmd: str, *, cwd: Optional[Path] = None) -> tuple[int, str]:
    """Run a command capturing output. Returns (returncode, stdout+stderr)."""
    p = subprocess.run(cmd, shell=True, cwd=cwd, capture_output=True, text=True)
    return p.returncode, (p.stdout + p.stderr).strip()


def ok(cmd: str, *, cwd: Optional[Path] = None) -> bool:
    """True if the command exits 0."""
    return quiet(cmd, cwd=cwd)[0] == 0


def have(binary: str) -> bool:
    return shutil.which(binary) is not None


def apt_installed(pkg: str) -> bool:
    rc, out = quiet(f"dpkg-query -W -f='${{Status}}' {pkg}")
    return rc == 0 and "install ok installed" in out


# --------------------------------------------------------------------------- #
# Step model
# --------------------------------------------------------------------------- #

@dataclass
class Step:
    name: str
    description: str
    action: Callable[[], None]
    # Returns True when the step's effect is already present on the system.
    check: Optional[Callable[[], bool]] = None
    # Developer convenience -- skipped unless --include-optional.
    optional: bool = False
    # Needs a human at the keyboard (TUI, browser login, prompts).
    interactive: bool = False
    # The system must be rebooted before later steps are meaningful.
    reboot_after: bool = False
    notes: str = ""
    tags: list = field(default_factory=list)

    def satisfied(self, state: dict) -> bool:
        if self.check is not None:
            return bool(self.check())
        return self.name in state.get("completed", [])


STEPS: list[Step] = []


def step(**kwargs):
    def register(fn):
        STEPS.append(Step(action=fn, **kwargs))
        return fn
    return register


# --------------------------------------------------------------------------- #
# Phase 1 -- base OS
#
# History lines 1-41. This is everything from the first `sudo apt update` on a
# freshly flashed image up to a working ARK-OS install.
# --------------------------------------------------------------------------- #

@step(
    name="apt-base",
    description="apt update && apt upgrade",
    notes="History lines 1-2. Always re-run; there is nothing meaningful to probe.",
)
def _apt_base():
    run("sudo apt-get update")
    run("sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y")


@step(
    name="jetpack",
    description="Install the nvidia-jetpack metapackage",
    check=lambda: apt_installed("nvidia-jetpack"),
    notes="History line 3. Pulls CUDA, cuDNN, TensorRT, VPI.",
)
def _jetpack():
    run("sudo DEBIAN_FRONTEND=noninteractive apt-get install -y nvidia-jetpack")


@step(
    name="ark-os",
    description="Clone and install ARK-OS",
    check=lambda: ARK_OS_DIR.is_dir() and (ARK_OS_DIR / "install.sh").exists(),
    reboot_after=True,
    notes=(
        "History lines 4-30. ARK's installer brings up the flight stack, the "
        "mavlink-router/autopilot-manager user services, and its own "
        "dds-agent.service -- which this workspace later disables (see "
        "DRONE_SETUP.md section 4)."
    ),
)
def _ark_os():
    if not ARK_OS_DIR.is_dir():
        run(f"git clone https://github.com/ARK-Electronics/ARK-OS.git {ARK_OS_DIR}")
    else:
        run("git pull", cwd=ARK_OS_DIR)
    run("./install.sh", cwd=ARK_OS_DIR)


@step(
    name="jetson-io",
    description="Enable UART1 on the 40-pin header via jetson-io",
    interactive=True,
    reboot_after=True,
    notes=(
        "History line 28. `sudo /opt/nvidia/jetson-io/jetson-io.py` is a TUI and "
        "cannot be scripted. Choose: Configure Jetson 40pin Header -> Configure "
        "header pins manually -> enable uart1 -> Save and reboot. This is what "
        "gives you /dev/ttyTHS1 for the FC link. Watch out for the custom-DTB "
        "gotcha in DRONE_SETUP.md: with an Arducam overlay, jetson-io may patch "
        "the default DTB rather than the one actually booted."
    ),
)
def _jetson_io():
    print(f"{YELLOW}MANUAL STEP{RESET} -- run this yourself, it is an interactive TUI:")
    print("    sudo /opt/nvidia/jetson-io/jetson-io.py")
    print("  Configure Jetson 40pin Header -> Configure header pins manually")
    print("  -> enable 'uart1' -> Save pin changes -> Save and reboot to reconfigure")
    input("  Press Enter once you have done this (or Ctrl-C to abort)... ")


@step(
    name="spidev",
    description="Install spidev for the ARK IMU driver",
    check=lambda: ok("python3 -c 'import spidev'"),
    notes="History line 31.",
)
def _spidev():
    run("sudo pip3 install spidev")


@step(
    name="imu-driver",
    description="Run the ARK ICM42688-P IMU driver setup",
    check=lambda: False,  # no probe; state file decides
    notes="History lines 33-35.",
)
def _imu_driver():
    script = ARK_OS_DIR / "platform" / "jetson" / "scripts" / "icm42688p_driver.py"
    if not script.exists():
        print(f"{YELLOW}skip{RESET}: {script} not found (is ARK-OS installed?)")
        return
    run(f"sudo python3 {script.name}", cwd=script.parent)


# --------------------------------------------------------------------------- #
# Phase 2 -- developer tooling
#
# History lines 42-86 and 161-194. uv and gh are required; the rest are
# convenience and gated behind --include-optional.
# --------------------------------------------------------------------------- #

@step(
    name="uv",
    description="Install uv (Python environment manager)",
    check=lambda: have("uv") or (HOME / ".local/bin/uv").exists(),
    notes="History line 69. Required -- the workspace venv is uv-managed.",
)
def _uv():
    run("curl -LsSf https://astral.sh/uv/install.sh | sh")


@step(
    name="gh",
    description="Install the GitHub CLI from its apt repo",
    check=lambda: have("gh"),
    notes="History line 76.",
)
def _gh():
    run(
        "(type -p wget >/dev/null || (sudo apt-get update && sudo apt-get install wget -y)) "
        "&& sudo mkdir -p -m 755 /etc/apt/keyrings "
        "&& out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg "
        "&& cat $out | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null "
        "&& sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg "
        "&& sudo mkdir -p -m 755 /etc/apt/sources.list.d "
        '&& echo "deb [arch=$(dpkg --print-architecture) '
        'signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] '
        'https://cli.github.com/packages stable main" '
        "| sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null "
        "&& sudo apt-get update && sudo apt-get install gh -y"
    )


@step(
    name="gh-auth",
    description="Authenticate the GitHub CLI",
    check=lambda: ok("gh auth status"),
    interactive=True,
    notes="History line 85. Needed to push branches and open PRs from the drone.",
)
def _gh_auth():
    run("gh auth login")


@step(
    name="claude-code",
    description="Install Claude Code",
    check=lambda: have("claude") or (HOME / ".local/bin/claude").exists(),
    optional=True,
    notes="History line 42.",
)
def _claude_code():
    run("curl -fsSL https://claude.ai/install.sh | bash")


@step(
    name="lazygit",
    description="Install lazygit from its GitHub release",
    check=lambda: have("lazygit"),
    optional=True,
    notes=(
        "History lines 162-188. Note `apt install lazygit` does NOT work on "
        "jammy (no such package) -- that is why the history shows three failed "
        "attempts before the release tarball."
    ),
)
def _lazygit():
    run(
        'LAZYGIT_VERSION=$(curl -s "https://api.github.com/repos/jesseduffield/lazygit/releases/latest" '
        "| grep -Po '\"tag_name\": *\"v\\K[^\"]*') "
        '&& LAZYGIT_ARCH=$(uname -m | sed -e "s/aarch64/arm64/") '
        '&& cd "$(mktemp -d)" '
        '&& curl -Lo lazygit.tar.gz "https://github.com/jesseduffield/lazygit/releases/download/'
        'v${LAZYGIT_VERSION}/lazygit_${LAZYGIT_VERSION}_Linux_${LAZYGIT_ARCH}.tar.gz" '
        "&& tar xf lazygit.tar.gz lazygit "
        "&& sudo install lazygit -D -t /usr/local/bin/"
    )


@step(
    name="herdr",
    description="Install herdr",
    check=lambda: have("herdr"),
    optional=True,
    notes="History line 125.",
)
def _herdr():
    run("curl -fsSL https://herdr.dev/install.sh | sh")


@step(
    name="brave",
    description="Install the Brave browser",
    check=lambda: have("brave-browser"),
    optional=True,
    notes="History lines 51-52.",
)
def _brave():
    run("curl -fsS https://dl.brave.com/install.sh | FLAVOR=origin sh")


@step(
    name="tegrastats",
    description="Fetch the tegrastats helper script",
    check=lambda: (HOME / "tegrastats.py").exists(),
    optional=True,
    notes="History lines 54-56, 127-130.",
)
def _tegrastats():
    run(
        "curl -fsSLo tegrastats.py https://raw.githubusercontent.com/cdenihan/"
        "Pathfinder/refs/heads/main/src/pathfinder/tegrastats.py "
        "&& chmod +x tegrastats.py",
        cwd=HOME,
    )


# --------------------------------------------------------------------------- #
# Phase 3 -- the workspace itself
# --------------------------------------------------------------------------- #

@step(
    name="workspace",
    description="Initialise Jacob_Ladder submodules",
    check=lambda: (WS_ROOT / "src" / "px4-ros2-interface-lib"
                   / "px4_ros2_cpp" / "CMakeLists.txt").exists(),
    notes=(
        "History lines 68, 119. There is no patch step any more: "
        "px4-ros2-interface-lib points at a fork carrying the ModeExecutorBase "
        "fix, so a plain submodule init yields a buildable tree. This replaced "
        "the old manual `git apply patch.diff`."
    ),
)
def _workspace():
    run("git submodule update --init --recursive", cwd=WS_ROOT)


@step(
    name="base-tools",
    description="installation_scripts/base_tools.sh",
    check=lambda: apt_installed("ccache") and apt_installed("ninja-build"),
    notes="History lines 121-124. Runs as root.",
)
def _base_tools():
    run(f"sudo bash {SCRIPTS / 'base_tools.sh'}")


@step(
    name="ros2",
    description="installation_scripts/ros2_humble.sh",
    check=lambda: Path(f"/opt/ros/{ROS_DISTRO}/setup.bash").exists(),
    notes=(
        "Not in the reference history -- ROS arrived via ARK-OS on that box. "
        "Included so a clean image without ARK-OS still ends up correct."
    ),
)
def _ros2():
    run(f"sudo bash {SCRIPTS / 'ros2_humble.sh'}")


@step(
    name="opencv",
    description="Build CUDA-enabled OpenCV (long: 60-120 min)",
    check=lambda: _opencv_ok(),
    notes=(
        "History lines 135, 196, 198. REMOVE_DEFAULT_OPENCV=no is important: "
        "purging the stock OpenCV takes ros-humble-cv-bridge and image_pipeline "
        "with it. See the README 'OpenCV on the Jetson' section."
    ),
)
def _opencv():
    run(
        f"bash {SCRIPTS / 'install_opencv.sh'} 2>&1 | tee {HOME}/opencv_build.log",
        env={
            "REMOVE_DEFAULT_OPENCV": "no",
            "OPENCV_BUILD_JOBS": str(OPENCV_JOBS),
            "OPENCV_PYTHON_VENV": str(WS_ROOT / ".venv"),
        },
    )


def _opencv_ok() -> bool:
    """CUDA OpenCV is only 'done' if the venv imports it AND reports a GPU."""
    rc, out = quiet(f"pkg-config --modversion opencv4")
    if rc != 0 or out.strip() != OPENCV_VERSION:
        return False
    py = WS_ROOT / ".venv" / "bin" / "python"
    if not py.exists():
        return False
    return ok(
        f"{py} -c \"import cv2,sys; "
        f"sys.exit(0 if cv2.__version__=='{OPENCV_VERSION}' "
        f"and cv2.cuda.getCudaEnabledDeviceCount()>0 else 1)\""
    )


@step(
    name="drone-packages",
    description="installation_scripts/drone_packages.sh",
    check=lambda: apt_installed(f"ros-{ROS_DISTRO}-cv-bridge"),
    notes="History lines 136, 144.",
)
def _drone_packages():
    run(f"sudo bash {SCRIPTS / 'drone_packages.sh'}")


@step(
    name="venv",
    description="Create the uv venv and sync Python dependencies",
    check=lambda: (WS_ROOT / ".venv" / "bin" / "python").exists()
    and ok(f"{WS_ROOT}/.venv/bin/python -c 'import ultralytics'"),
    notes=(
        "History lines 90-91. --system-site-packages is required so ROS 2 and "
        "JetPack Python modules stay visible; do not drop it."
    ),
)
def _venv():
    if not (WS_ROOT / ".venv").exists():
        run("uv venv --python 3.10 --system-site-packages .venv", cwd=WS_ROOT)
    run("uv sync", cwd=WS_ROOT)


@step(
    name="build",
    description="colcon build the workspace against the CUDA OpenCV",
    check=lambda: (WS_ROOT / "install" / "aruco_tracker").is_dir(),
    notes=(
        "-DOpenCV_DIR pins the build to /usr/local so cmake cannot pick up the "
        "JetPack 4.8.0 or the Ubuntu 4.5.4 debs instead."
    ),
)
def _build():
    run(
        f"bash -lc 'source /opt/ros/{ROS_DISTRO}/setup.bash && "
        f"cd {WS_ROOT} && colcon build --cmake-args -DOpenCV_DIR={OPENCV_CMAKE_DIR}'"
    )


# --------------------------------------------------------------------------- #
# Phase 4 -- services
# --------------------------------------------------------------------------- #

@step(
    name="services",
    description="Install systemd units, the OAK udev rule and /var/log/ros2",
    check=lambda: Path("/etc/systemd/system/dds_agent.service").exists()
    and Path("/etc/udev/rules.d/99-oak-usb-power.rules").exists()
    and Path("/var/log/ros2").is_dir(),
)
def _services():
    run(f"sudo bash {WS_ROOT / 'services' / 'install_services.sh'}")


@step(
    name="disable-ark-agent",
    description="Disable ARK's conflicting user-level dds-agent.service",
    check=lambda: not (
        HOME / ".config/systemd/user/default.target.wants/dds-agent.service"
    ).exists(),
    notes=(
        "ARK's agent drives the same /dev/ttyTHS1 at 3000000 baud while ours "
        "uses 921600. Nothing in systemd stops both running -- one is user-level "
        "and one is system-level, with different unit names. Exactly one must "
        "run. See DRONE_SETUP.md section 4."
    ),
)
def _disable_ark_agent():
    run("systemctl --user disable --now dds-agent.service", check=False)


@step(
    name="enable-services",
    description="Enable dds_agent and translation_node at boot",
    check=lambda: ok("systemctl is-enabled dds_agent.service")
    and ok("systemctl is-enabled translation_node.service"),
    notes=(
        "aruco_tracker and usb_cam are deliberately left disabled: they grab "
        "cameras at boot and would fight launch_scripts/super_real.sh."
    ),
)
def _enable_services():
    run("sudo systemctl enable --now dds_agent.service translation_node.service")


# --------------------------------------------------------------------------- #
# State
# --------------------------------------------------------------------------- #

def load_state() -> dict:
    if STATE_FILE.exists():
        try:
            return json.loads(STATE_FILE.read_text())
        except json.JSONDecodeError:
            print(f"{YELLOW}warning{RESET}: {STATE_FILE} is corrupt, ignoring")
    return {"completed": []}


def save_state(state: dict) -> None:
    STATE_FILE.write_text(json.dumps(state, indent=2) + "\n")


# --------------------------------------------------------------------------- #
# Driver
# --------------------------------------------------------------------------- #

def print_listing(state: dict) -> None:
    print(f"\n{BOLD}Setup steps{RESET}  ({WS_ROOT})\n")
    for s in STEPS:
        done = s.satisfied(state)
        mark = f"{GREEN}done{RESET}" if done else f"{YELLOW}todo{RESET}"
        flags = []
        if s.optional:
            flags.append("optional")
        if s.interactive:
            flags.append("interactive")
        if s.reboot_after:
            flags.append("reboot after")
        suffix = f" {DIM}[{', '.join(flags)}]{RESET}" if flags else ""
        print(f"  [{mark}] {BOLD}{s.name:<18}{RESET} {s.description}{suffix}")
    print()


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--list", action="store_true", help="show steps and exit")
    ap.add_argument("--dry-run", action="store_true", help="print without running")
    ap.add_argument("--only", action="append", default=[], metavar="STEP")
    ap.add_argument("--skip", action="append", default=[], metavar="STEP")
    ap.add_argument("--from", dest="start_at", metavar="STEP")
    ap.add_argument("--include-optional", action="store_true")
    ap.add_argument("--force", action="store_true",
                    help="run steps even if their check reports satisfied")
    ap.add_argument("--unattended", action="store_true",
                    help="never prompt: requires a sudo password, and skips "
                         "the steps that need a human at the keyboard")
    ap.add_argument("--sudo-password-file", metavar="PATH",
                    help="file whose first line is the sudo password "
                         "(create with `install -m 600 /dev/null PATH`). "
                         "There is no --sudo-password flag on purpose: "
                         "arguments are visible in ps and shell history.")
    ap.add_argument("--accept-risk", action="store_true",
                    help="suppress the untested-script banner")
    args = ap.parse_args()

    if os.geteuid() == 0:
        print(f"{RED}error{RESET}: do not run this as root. Run as your normal "
              f"user; steps escalate with sudo where needed.")
        return 1

    known = {s.name for s in STEPS}
    for name in [*args.only, *args.skip, *([args.start_at] if args.start_at else [])]:
        if name not in known:
            print(f"{RED}error{RESET}: unknown step '{name}'. "
                  f"Try --list.")
            return 1

    state = load_state()

    if args.list:
        print_listing(state)
        return 0

    if not args.accept_risk:
        print(f"\n{RED}{BOLD}  USE AT YOUR OWN RISK{RESET}")
        print(f"{YELLOW}  This script has never been run end to end. It was "
              f"reconstructed from{RESET}")
        print(f"{YELLOW}  one machine's shell history and validated only with "
              f"--list and --dry-run{RESET}")
        print(f"{YELLOW}  against an already-configured box. Some steps are "
              f"destructive and hard{RESET}")
        print(f"{YELLOW}  to undo. Use a board you can reflash, not a drone "
              f"you are about to fly.{RESET}")
        print(f"{DIM}  See the module docstring for detail; --accept-risk "
              f"hides this.{RESET}\n")

    # Unattended sudo must be armed before any step runs.
    password = resolve_sudo_password(args)
    if password:
        enable_unattended_sudo(password)
        print(f"{GREEN}ok{RESET}   sudo password accepted; running unattended")
    del password

    selected = STEPS
    if args.start_at:
        idx = next(i for i, s in enumerate(STEPS) if s.name == args.start_at)
        selected = STEPS[idx:]
    if args.only:
        selected = [s for s in STEPS if s.name in args.only]

    deferred_interactive: list[str] = []

    for s in selected:
        if s.name in args.skip:
            print(f"{DIM}skip {s.name} (--skip){RESET}")
            continue
        if s.optional and not args.include_optional and not args.only:
            print(f"{DIM}skip {s.name} (optional; --include-optional to run){RESET}")
            continue
        # An interactive step would block forever with nobody at the keyboard.
        if args.unattended and s.interactive and not s.satisfied(state):
            print(f"{YELLOW}defer{RESET} {s.name} "
                  f"{DIM}(interactive; needs a human){RESET}")
            deferred_interactive.append(s.name)
            continue
        # --only implies you want it run regardless of its check.
        if not args.force and not args.only and s.satisfied(state):
            print(f"{GREEN}ok{RESET}   {s.name} {DIM}already satisfied{RESET}")
            continue

        print(f"\n{BOLD}==> {s.name}{RESET}: {s.description}")
        if s.notes:
            for line in s.notes.split(". "):
                if line.strip():
                    print(f"    {DIM}{line.strip().rstrip('.')}.{RESET}")
        if args.dry_run:
            print(f"    {YELLOW}(dry run -- not executed){RESET}")
            continue

        try:
            s.action()
        except subprocess.CalledProcessError as exc:
            print(f"\n{RED}FAILED{RESET} at step '{s.name}' (exit {exc.returncode}).")
            print(f"Fix the problem, then resume with:\n"
                  f"    {sys.argv[0]} --from {s.name}")
            return exc.returncode
        except KeyboardInterrupt:
            print(f"\n{YELLOW}interrupted{RESET} during '{s.name}'.")
            return 130

        if s.name not in state["completed"]:
            state["completed"].append(s.name)
            save_state(state)

        if s.reboot_after:
            print(f"\n{YELLOW}This step requires a reboot.{RESET}")
            print(f"Reboot, then resume with:\n    {sys.argv[0]}")
            return 0

    if deferred_interactive:
        print(f"\n{YELLOW}{BOLD}Deferred -- these need a human at the keyboard:"
              f"{RESET}")
        for name in deferred_interactive:
            print(f"    {sys.argv[0]} --only {name}")
        print(f"{DIM}Setup is not finished until those are done.{RESET}")

    print(f"\n{GREEN}{BOLD}Setup complete.{RESET}")
    print("Verify with:")
    print(f"    {WS_ROOT}/.venv/bin/python -c "
          f"'import cv2; print(cv2.__version__, cv2.cuda.getCudaEnabledDeviceCount())'")
    print("    systemctl status dds_agent translation_node")
    print("    journalctl -u dds_agent | grep -i 'session established'")
    return 0


if __name__ == "__main__":
    sys.exit(main())
