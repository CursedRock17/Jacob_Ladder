"""The readiness report (spec section 7): one line per check. No ROS here; the
probe node in jl_blocks.ros.readiness_probe gathers the evidence."""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass

MARKS = {True: "✓", False: "✗", None: "?"}


@dataclass(frozen=True)
class Check:
    name: str
    ok: bool | None  # None = could not tell (e.g. no journal in a container)
    detail: str


def rate_hz(stamps: list[float]) -> float:
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return 0.0
    return round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 1)


def format_report(checks: Iterable[Check]) -> list[str]:
    return [f"{MARKS[c.ok]} {c.name:<34}{c.detail}" for c in checks]


def all_ok(checks: Iterable[Check], advisory: set[str] | None = None) -> bool:
    skip = advisory or set()
    return all(c.ok is not False for c in checks if c.name not in skip)
