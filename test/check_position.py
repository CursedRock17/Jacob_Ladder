"""Where is the drone? Reads PX4's local position once and checks it is within
TOL metres of NORTH, EAST (metres from where it spawned), and either on the
ground (--landed) or in the air (--airborne).

    check_position.py NORTH EAST [TOL] (--landed | --airborne) [--ring R]

With --ring R, NORTH EAST is a target instead, and the drone must be R metres
from it (within TOL) in any direction: how standoff skills are specified, since
they back off along the drone's own line to the target.

Run by the SITL harnesses after (or during) a flight. Exit 0 = pass.
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import qos_profile_sensor_data

parser = argparse.ArgumentParser()
parser.add_argument("north", type=float)
parser.add_argument("east", type=float)
parser.add_argument("tolerance", type=float, nargs="?", default=0.35)
state = parser.add_mutually_exclusive_group(required=True)
state.add_argument("--landed", action="store_true")
state.add_argument("--airborne", action="store_true")
parser.add_argument("--ring", type=float, default=None, metavar="R")
args = parser.parse_args()

rclpy.init()
node = rclpy.create_node("check_position")
got: list[VehicleLocalPosition] = []
node.create_subscription(
    VehicleLocalPosition,
    "/fmu/out/vehicle_local_position_v1",
    got.append,
    qos_profile_sensor_data,
)
deadline = time.monotonic() + 15
while not got and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.5)
if not got:
    print("FAIL no position from PX4", flush=True)
    sys.exit(1)

p = got[-1]
distance = ((p.x - args.north) ** 2 + (p.y - args.east) ** 2) ** 0.5
on_ground = abs(p.z) < 0.3
wanted = "on the ground" if args.landed else "in the air"
print(f"at north {p.x:.2f}, east {p.y:.2f}, up {-p.z:.2f}")
if args.ring is None:
    error = distance
    where = f"within {args.tolerance} m of ({args.north}, {args.east})"
else:
    error = abs(distance - args.ring)
    where = f"{args.ring} m (+/- {args.tolerance}) from ({args.north}, {args.east})"
ok = error <= args.tolerance and on_ground == args.landed
print(
    f"{'PASS' if ok else 'FAIL'} {wanted}, {where} (off by {error:.2f} m)", flush=True
)
rclpy.shutdown()
sys.exit(0 if ok else 1)
