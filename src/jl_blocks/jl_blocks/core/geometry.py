"""Small vector helpers for blocks: plain tuples, no numpy, so core stays light."""

from __future__ import annotations

import math
from collections.abc import Iterable

from .types import Vector3

# w, x, y, z, as PX4 reports attitude (body FRD -> NED)
Quaternion = tuple[float, float, float, float]


def add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def scale(a: Vector3, k: float) -> Vector3:
    return (a[0] * k, a[1] * k, a[2] * k)


def norm(a: Vector3) -> float:
    return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])


def is_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(v) for v in values)


def valid_quaternion(q: Quaternion) -> bool:
    """False for NaN or near-zero quaternions (e.g. attitude not received yet)."""
    return is_finite(q) and math.sqrt(sum(c * c for c in q)) >= 0.1


def rotate(q: Quaternion, v: Vector3) -> Vector3:
    """Rotate v by the (normalised) quaternion q."""
    length = math.sqrt(sum(c * c for c in q))
    w, x, y, z = (c / length for c in q)
    # v' = v + w*t + u x t, where u = (x, y, z) and t = 2 * (u x v)
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )
