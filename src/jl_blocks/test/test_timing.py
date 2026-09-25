from __future__ import annotations

import pytest

from jl_blocks.timing import summary


def test_regular_20_ms_stream():
    stamps = [i * 0.02 for i in range(101)]
    s = summary(stamps)
    assert s["count"] == 101
    assert s["p50_ms"] == pytest.approx(20.0)
    assert s["max_ms"] == pytest.approx(20.0)


def test_one_late_message_shows_in_max_and_p99():
    stamps = [i * 0.02 for i in range(100)] + [1.98 + 0.05]
    s = summary(stamps)
    assert s["max_ms"] == pytest.approx(50.0)
    assert s["p99_ms"] >= 20.0


def test_too_few_messages():
    assert summary([0.0])["count"] == 1
    assert summary([0.0])["max_ms"] == 0.0


def test_percentiles_distinguish_regular_gaps_from_tail_delays():
    stamps = [i * 0.02 for i in range(99)] + [2.01, 2.06]
    s = summary(stamps)
    assert s["p50_ms"] == pytest.approx(20.0)
    assert s["p99_ms"] == pytest.approx(50.0)
    assert summary([]) == {"count": 0, "p50_ms": 0, "p99_ms": 0, "max_ms": 0}
