from __future__ import annotations

import math
from typing import cast

import pytest

from jl_blocks.core.geometry import (
    Quaternion,
    add,
    is_finite,
    norm,
    rotate,
    scale,
    sub,
    valid_quaternion,
)

LEVEL = (1.0, 0.0, 0.0, 0.0)
YAW_90 = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))


def test_vector_arithmetic():
    assert add((1.0, 2.0, 3.0), (1.0, 1.0, 1.0)) == (2.0, 3.0, 4.0)
    assert sub((1.0, 2.0, 3.0), (1.0, 1.0, 1.0)) == (0.0, 1.0, 2.0)
    assert scale((1.0, 2.0, 3.0), 2.0) == (2.0, 4.0, 6.0)
    assert norm((3.0, 4.0, 0.0)) == 5.0


def test_level_attitude_leaves_a_vector_alone():
    assert rotate(LEVEL, (1.0, 2.0, 3.0)) == pytest.approx((1.0, 2.0, 3.0))


def test_facing_east_turns_forward_into_east():
    assert rotate(YAW_90, (3.0, 0.0, 0.0)) == pytest.approx((0.0, 3.0, 0.0))


def test_an_unnormalised_quaternion_still_rotates_correctly():
    doubled = cast(Quaternion, tuple(2.0 * c for c in YAW_90))
    assert rotate(doubled, (3.0, 0.0, 0.0)) == pytest.approx((0.0, 3.0, 0.0))


def test_finite_and_quaternion_checks():
    assert is_finite((1.0, 2.0, 3.0))
    assert not is_finite((1.0, math.nan, 3.0))
    assert not is_finite((math.inf, 0.0, 0.0))
    assert valid_quaternion(LEVEL)
    assert not valid_quaternion((0.0, 0.0, 0.0, 0.0))
    assert not valid_quaternion((math.nan, 0.0, 0.0, 1.0))
