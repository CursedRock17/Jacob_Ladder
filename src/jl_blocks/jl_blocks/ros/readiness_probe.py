"""readiness: the deploy.sh readiness report (spec section 7), or --armed.

Samples the topics for a few seconds, reads the journal where there is one,
and prints one line per check. Exit 0 = ready (advisory checks aside).
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from ..readiness import Check, all_ok, format_report, rate_hz

# VehicleStatus uses this workspace's v1 layout; translation_node bridges
# older firmware to the suffixed topic, just as it does for local position.
STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
# VehicleOdometry is still version 0, so its topic has no version suffix.
VIO_TOPIC = "/fmu/in/vehicle_visual_odometry"
TAG_TOPIC = "/front/target_pose"


def journal_has(unit: str, text: str) -> bool | None:
    # In a container without systemd, journalctl can return 0 with no entries.
    # That is unavailable evidence, not proof that the service failed.
    if not Path("/run/systemd/system").is_dir() or shutil.which("journalctl") is None:
        return None
    result = subprocess.run(
        ["journalctl", "-b", "-u", unit, "--no-pager", "-o", "cat"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0 and not result.stdout:
        return None
    return text in result.stdout


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("readiness_probe")
        self.status: VehicleStatus | None = None
        self.vio: list[float] = []
        self.tags = 0
        self.create_subscription(
            VehicleStatus,
            STATUS_TOPIC,
            self.on_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            VehicleOdometry, VIO_TOPIC, self.on_vio, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, TAG_TOPIC, self.on_tag, qos_profile_sensor_data
        )

    def on_status(self, msg: VehicleStatus) -> None:
        self.status = msg

    def on_vio(self, msg: VehicleOdometry) -> None:
        self.vio.append(time.monotonic())

    def on_tag(self, msg: PoseStamped) -> None:
        self.tags += 1

    def sample(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def report(probe: Probe, names: list[str], helpers: list[str]) -> list[Check]:
    checks = []
    dds = journal_has("dds_agent", "session established")
    checks.append(
        Check(
            "DDS agent session established",
            dds,
            "dds_agent journal" if dds is not None else "journalctl unavailable",
        )
    )
    checks.append(Check("FC data arriving", probe.status is not None, STATUS_TOPIC))
    if "vio" in helpers:
        hz = rate_hz(probe.vio)
        checks.append(Check("VIO publishing", hz >= 20.0, f"{VIO_TOPIC}  {hz:g} Hz"))
    missing = []
    for name in names:
        has_pub = probe.count_publishers(f"/jl/{name}/active") > 0
        logged = journal_has("jl_mission@*", f"Registered '{name}'")
        if not has_pub or logged is False:
            missing.append(name)
    checks.append(
        Check(
            "Registered in PX4",
            not missing,
            ", ".join(names) if not missing else f"missing: {', '.join(missing)}",
        )
    )
    if "aruco_tracker" in helpers:
        checks.append(
            Check(
                "aruco_tracker",
                probe.tags > 0,
                f"{TAG_TOPIC}  {probe.tags} detections"
                + ("" if probe.tags else " (fine if no tag in view)"),
            )
        )
    return checks


def main() -> None:
    parser = argparse.ArgumentParser(prog="readiness")
    parser.add_argument("--names", nargs="*", default=[])
    parser.add_argument("--helpers", nargs="*", default=[])
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--armed", action="store_true", help="only print the arm state")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    probe = Probe()
    try:
        probe.sample(args.seconds)
        if args.armed:
            if probe.status is None:
                print("unknown")
                code = 4
            elif probe.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                print("armed")
                code = 3
            else:
                print("disarmed")
                code = 0
        else:
            checks = report(probe, args.names, args.helpers)
            for line in format_report(checks):
                print(line)
            code = 0 if all_ok(checks, advisory={"aruco_tracker"}) else 1
    finally:
        probe.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == "__main__":
    main()
