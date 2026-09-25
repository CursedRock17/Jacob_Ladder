"""Setpoint inter-arrival statistics, as in spec section 6's spike table."""

from __future__ import annotations


def _percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return 0.0
    index = min(len(sorted_values) - 1, round(q * (len(sorted_values) - 1)))
    return sorted_values[index]


def summary(stamps: list[float]) -> dict[str, float]:
    gaps = sorted((b - a) * 1000.0 for a, b in zip(stamps, stamps[1:]))
    return {
        "count": float(len(stamps)),
        "p50_ms": round(_percentile(gaps, 0.50), 1),
        "p99_ms": round(_percentile(gaps, 0.99), 1),
        "max_ms": round(gaps[-1], 1) if gaps else 0.0,
    }
