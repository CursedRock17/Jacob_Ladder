from __future__ import annotations

import pytest

from jl_blocks.core import MissionError, parse_mission

GOOD = """
name: TrackBeacon
target:
  beacon: {id: 3}
controller:
  doubler: {}
steps:
  - goto: {x: 2.0}
    until: target_seen
    timeout: 30
    on_fail: goto_again
  - goto: {x: 4}
    name: goto_again
    until: never
"""


def errors(text, registry):
    with pytest.raises(MissionError) as caught:
        parse_mission(text, registry, source="m.yaml")
    return caught.value.errors


def test_good_mission_parses_into_a_spec(registry):
    spec = parse_mission(GOOD, registry)
    assert spec.name == "TrackBeacon"
    assert spec.target is not None and spec.target.params.id == 3
    assert spec.controller.name == "doubler"
    assert [s.name for s in spec.steps] == ["goto", "goto_again"]
    first = spec.steps[0]
    assert (first.until, first.timeout, first.on_fail) == (
        "target_seen",
        30,
        "goto_again",
    )
    assert spec.steps[1].block.params.x == 4


def test_controller_defaults_to_position(registry):
    spec = parse_mission("name: M\nsteps:\n  - goto: {}\n", registry)
    assert spec.controller.name == "position"


def test_unknown_param_suggests_the_close_one(registry):
    errs = errors("name: M\nsteps:\n  - goto: {xx: 1}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (goto): unknown param 'xx' for goto; did you mean 'x'?"
    ]


def test_unknown_block_suggests_a_block_of_the_same_kind(registry):
    errs = errors("name: M\nsteps:\n  - gotoo: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (gotoo): unknown mission 'gotoo'; did you mean 'goto'?"
    ]


def test_block_of_the_wrong_kind_is_named(registry):
    errs = errors("name: M\ncontroller:\n  goto: {}\nsteps:\n  - goto: {}\n", registry)
    assert errs == ["m.yaml: controller: 'goto' is a mission, not a controller"]


def test_param_type_is_checked_and_ints_count_as_floats(registry):
    errs = errors("name: M\nsteps:\n  - goto: {x: fast, ticks: 2.5}\n", registry)
    assert "param 'x' for goto should be float, got 'fast'" in errs[0]
    assert "param 'ticks' for goto should be int, got 2.5" in errs[1]


def test_missing_required_param_is_reported(registry):
    errs = errors("name: M\nsteps:\n  - needy: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (needy): missing required param 'height' for needy"
    ]


def test_all_problems_are_reported_together(registry):
    text = "name: 9bad\nsteps:\n  - gotoo: {}\n  - goto: {xx: 1}\n"
    assert len(errors(text, registry)) == 3


def test_step_must_have_exactly_one_block(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    quitter: {}\n", registry)
    assert errs == [
        "m.yaml: steps[0]: each step needs exactly one mission block (found: goto, quitter)"
    ]


def test_until_accepts_words_and_seconds_only(registry):
    parse_mission("name: M\nsteps:\n  - goto: {}\n    until: 5\n", registry)
    errs = errors("name: M\nsteps:\n  - goto: {}\n    until: soon\n", registry)
    assert "until must be one of done, target_seen, target_lost, never" in errs[0]


def test_on_fail_must_name_an_existing_step(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    on_fail: gotoo\n", registry)
    assert errs == [
        "m.yaml: step 'goto': on_fail 'gotoo' is not a step; did you mean 'goto'?"
    ]


def test_duplicate_step_names_need_a_name_field(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n  - goto: {}\n", registry)
    assert errs == [
        "m.yaml: step 'goto': duplicate step name; add 'name:' to one of them"
    ]


def test_target_conditions_need_a_target(registry):
    errs = errors("name: M\nsteps:\n  - goto: {}\n    until: target_seen\n", registry)
    assert errs == [
        "m.yaml: step 'goto': until: target_seen needs a target (at the top or on this step)"
    ]


def test_step_can_override_target_and_controller(registry):
    text = "name: M\nsteps:\n  - goto: {}\n    target: {beacon: {id: 7}}\n    controller: {doubler: {}}\n    until: target_seen\n"
    step = parse_mission(text, registry).steps[0]
    assert step.target is not None and step.target.params.id == 7
    assert step.controller is not None and step.controller.name == "doubler"


def test_unknown_top_level_key_is_reported(registry):
    errs = errors("name: M\nstep:\n  - goto: {}\n", registry)
    assert "m.yaml: top level: unknown key 'step'; did you mean 'steps'?" in errs


def test_invalid_yaml_is_reported_not_raised_raw(registry):
    errs = errors("name: [unclosed\n", registry)
    assert errs[0].startswith("m.yaml: invalid YAML:")


def test_mission_name_must_be_usable_in_qgc_and_ros(registry):
    errs = errors("name: my mission\nsteps:\n  - goto: {}\n", registry)
    assert "name must start with a letter" in errs[0]


def test_a_blocks_own_param_validation_is_reported_not_raised_raw(registry):
    errs = errors("name: M\nsteps:\n  - ranged: {height: -1}\n", registry)
    assert errs == [
        "m.yaml: steps[0] (ranged): bad params for ranged: height must be > 0"
    ]


def errors_of(text, registry):
    with pytest.raises(MissionError) as caught:
        parse_mission(text, registry)
    return caught.value.errors


def test_a_name_longer_than_24_characters_is_rejected(registry):
    errs = errors_of("name: " + "A" * 25 + "\nsteps:\n  - goto: {}\n", registry)
    assert "at most 24 characters" in errs[0]


def test_a_name_of_exactly_24_characters_is_fine(registry):
    spec = parse_mission("name: " + "A" * 24 + "\nsteps:\n  - goto: {}\n", registry)
    assert len(spec.name) == 24


def test_nan_is_not_a_number(registry):
    errs = errors_of("name: M\nsteps:\n  - goto: {x: .nan}\n", registry)
    assert "param 'x' for goto should be float, got nan" in errs[0]


def test_infinity_is_not_a_timeout(registry):
    errs = errors_of("name: M\nsteps:\n  - goto: {}\n    timeout: .inf\n", registry)
    assert "timeout must be a positive number" in errs[0]


def test_a_block_that_needs_a_target_says_so(registry):
    errs = errors_of("name: M\nsteps:\n  - seeker: {}\n", registry)
    assert "seeker needs a target (at the top or on this step)" in errs[0]


def test_a_top_level_target_satisfies_needs_target(registry):
    parse_mission("name: M\ntarget: {beacon: {}}\nsteps:\n  - seeker: {}\n", registry)
