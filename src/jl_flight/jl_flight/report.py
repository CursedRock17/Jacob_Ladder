"""One line saying what the mission is commanding, for checking setpoints by eye.

    12.4 s Track     setpoint N +2.91 E -0.12 U 1.20 hdg  88 vel N +0.50 E +0.00 U +0.00
                     drone    N +0.02 E +0.01 U 0.05 hdg  96
                     apriltag_36h11_0  N +5.90 E +0.10 U 1.20, 5.9 m away

North, east and up are metres from where PX4's position estimate started;
headings are degrees clockwise from north. No ROS here.
"""

from __future__ import annotations

import math

from .core import Setpoint, Vector3, VehicleState

INDENT = " " * 17


def _ned(p: Vector3) -> str:
    return f"N {p[0] + 0.0:+6.2f} E {p[1] + 0.0:+6.2f} U {0.0 - p[2]:5.2f}"


def describe(
    t: float,
    activity: str,
    setpoint: Setpoint | None,
    vehicle: VehicleState | None,
    targets: dict[str, Vector3 | None],
) -> str:
    """t: seconds since the mission started; activity: what it is doing (a skill
    or a call like takeoff); targets: each declared target's estimate (None: not seen)."""
    lines = []
    head = f"{t:6.1f} s {activity:<9}"
    if setpoint is None:
        lines.append(f"{head}setpoint none (PX4 is flying its own {activity})")
    else:
        parts = ["setpoint"]
        if setpoint.position is not None:
            parts.append(_ned(setpoint.position))
        if setpoint.yaw is not None:
            parts.append(f"hdg {math.degrees(setpoint.yaw) % 360:4.0f}")
        if setpoint.velocity is not None:
            v = setpoint.velocity
            parts.append(f"vel N {v[0]:+5.2f} E {v[1]:+5.2f} U {0.0 - v[2]:+5.2f}")
        lines.append(head + " ".join(parts))
    if vehicle is not None:
        heading = math.degrees(vehicle.yaw) % 360
        lines.append(
            f"{INDENT}drone    {_ned(vehicle.position_ned)} hdg {heading:4.0f}"
        )
    for name, where in targets.items():
        if where is None:
            lines.append(f"{INDENT}{name}  not seen")
        else:
            away = (
                math.dist(where, vehicle.position_ned)
                if vehicle is not None
                else math.nan
            )
            lines.append(f"{INDENT}{name}  {_ned(where)}, {away:.1f} m away")
    return "\n".join(lines)
