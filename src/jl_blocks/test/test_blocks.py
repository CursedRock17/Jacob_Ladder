from __future__ import annotations

import pytest

from jl_blocks.core import Controller, Mission, Registry, Target, block


def test_decorator_registers_block_under_its_name():
    reg = Registry()

    @block("spin", registry=reg)
    class Spin(Mission):
        pass

    assert reg.get("spin") is Spin
    assert Spin.name == "spin"


def test_registering_the_same_name_twice_is_an_error():
    reg = Registry()

    class First(Mission):
        pass

    class Second(Controller):
        pass

    reg.register("a", First)
    with pytest.raises(ValueError, match="already registered"):
        reg.register("a", Second)


def test_names_can_be_filtered_by_kind(registry):
    assert registry.names("target") == ["badtarget", "beacon", "counter", "grumpy"]
    assert "goto" in registry.names("mission")
    assert "goto" not in registry.names("controller")


def test_block_uses_default_params_when_none_given(registry):
    goto = registry.get("goto")
    assert goto is not None
    assert goto().params.x == 1.0


def test_target_default_lost_after_is_three_seconds():
    assert Target.lost_after == 3.0
