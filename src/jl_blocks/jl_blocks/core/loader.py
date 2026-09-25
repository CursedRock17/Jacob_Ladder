"""Read a mission YAML file into a checked MissionSpec.

Every problem found is collected and reported together, each with where it is
and, for typos, a suggestion, so a beginner fixes a file in one pass instead
of one error at a time.
"""

from __future__ import annotations

import dataclasses
import difflib
import math
import re
import typing
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .blocks import REGISTRY, Registry

TOP_KEYS = ("name", "target", "controller", "steps")
STEP_KEYS = ("until", "timeout", "on_fail", "name", "target", "controller")
UNTIL_WORDS = ("done", "target_seen", "target_lost", "never")
DEFAULT_CONTROLLER = "position"
NAME_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_]{0,23}$")


@dataclass(frozen=True)
class BlockSpec:
    name: str
    params: Any


@dataclass(frozen=True)
class StepSpec:
    name: str
    block: BlockSpec
    until: str | float = "done"
    timeout: float | None = None
    on_fail: str | None = None
    target: BlockSpec | None = None
    controller: BlockSpec | None = None


@dataclass(frozen=True)
class MissionSpec:
    name: str
    target: BlockSpec | None
    controller: BlockSpec
    steps: tuple[StepSpec, ...]


class MissionError(Exception):
    def __init__(self, errors: list[str]) -> None:
        self.errors = errors
        super().__init__("\n".join(errors))


def _suggest(word: str, choices: list[str]) -> str:
    close = difflib.get_close_matches(word, choices, n=1, cutoff=0.6)
    return f"; did you mean '{close[0]}'?" if close else ""


def _is_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


class _Parser:
    def __init__(self, registry: Registry, source: str) -> None:
        self.registry = registry
        self.source = source
        self.errors: list[str] = []
        self.step_names: list[str] = []

    def error(self, where: str, message: str) -> None:
        self.errors.append(f"{self.source}: {where}: {message}")

    def block_ref(self, node: Any, kind: str, where: str) -> BlockSpec | None:
        """Parse ``{name: {params}}`` into a BlockSpec of the given kind."""
        if not isinstance(node, dict) or len(node) != 1:
            self.error(
                where, f"expected one {kind} block, like 'name: {{param: value}}'"
            )
            return None
        ((name, params),) = node.items()
        return self.block(str(name), params, kind, where)

    def block(self, name: str, params: Any, kind: str, where: str) -> BlockSpec | None:
        cls = self.registry.get(name)
        if cls is None:
            self.error(
                where,
                f"unknown {kind} '{name}'{_suggest(name, self.registry.names(kind))}",
            )
            return None
        if cls.kind != kind:
            self.error(where, f"'{name}' is a {cls.kind}, not a {kind}")
            return None
        if params is None:
            params = {}
        if not isinstance(params, dict):
            self.error(
                where, f"params for {name} must be a mapping like {{param: value}}"
            )
            return None
        fields = {f.name: f for f in dataclasses.fields(cls.Params)}
        hints = typing.get_type_hints(cls.Params)
        ok = True
        for key, value in params.items():
            if key not in fields:
                self.error(
                    where,
                    f"unknown param '{key}' for {name}{_suggest(str(key), list(fields))}",
                )
                ok = False
            elif not _type_ok(hints.get(key), value):
                self.error(
                    where,
                    f"param '{key}' for {name} should be {_type_name(hints.get(key))}, got {value!r}",
                )
                ok = False
        for field in fields.values():
            no_default = (
                field.default is dataclasses.MISSING
                and field.default_factory is dataclasses.MISSING
            )
            if no_default and field.name not in params:
                self.error(where, f"missing required param '{field.name}' for {name}")
                ok = False
        if not ok:
            return None
        try:
            return BlockSpec(name, cls.Params(**params))
        except Exception as err:
            self.error(where, f"bad params for {name}: {err}")
            return None

    def step(self, index: int, node: Any) -> StepSpec | None:
        where = f"steps[{index}]"
        if not isinstance(node, dict):
            self.error(where, "each step must be a mapping like '- hold: {}'")
            return None
        block_keys = [k for k in node if k not in STEP_KEYS]
        if len(block_keys) != 1:
            found = ", ".join(map(str, block_keys)) or "none"
            self.error(
                where, f"each step needs exactly one mission block (found: {found})"
            )
            return None
        block_name = str(block_keys[0])
        where = f"steps[{index}] ({block_name})"
        spec = self.block(block_name, node[block_name], "mission", where)

        until = node.get("until", "done")
        if not (until in UNTIL_WORDS or (_is_number(until) and until > 0)):
            self.error(
                where,
                f"until must be one of {', '.join(UNTIL_WORDS)} or a number of seconds, got {until!r}",
            )
        timeout = node.get("timeout")
        if timeout is not None and not (_is_number(timeout) and timeout > 0):
            self.error(
                where, f"timeout must be a positive number of seconds, got {timeout!r}"
            )
        on_fail = node.get("on_fail")
        if on_fail is not None and not isinstance(on_fail, str):
            self.error(where, f"on_fail must be a step name, got {on_fail!r}")
        name = node.get("name", block_name)
        if not isinstance(name, str):
            self.error(where, f"name must be text, got {name!r}")
        self.step_names.append(str(name))
        target = (
            self.block_ref(node["target"], "target", f"{where} target")
            if "target" in node
            else None
        )
        controller = (
            self.block_ref(node["controller"], "controller", f"{where} controller")
            if "controller" in node
            else None
        )
        if spec is None:
            return None
        return StepSpec(
            name=str(name),
            block=spec,
            until=until,
            timeout=timeout,
            on_fail=on_fail,
            target=target,
            controller=controller,
        )

    def mission(self, doc: Any) -> MissionSpec | None:
        if not isinstance(doc, dict):
            self.error(
                "file",
                "a mission file must be a mapping with name, steps and optionally target and controller",
            )
            return None
        for key in doc:
            if key not in TOP_KEYS:
                self.error(
                    "top level",
                    f"unknown key '{key}'{_suggest(str(key), list(TOP_KEYS))}",
                )

        name = doc.get("name")
        if not isinstance(name, str) or not NAME_PATTERN.match(name):
            self.error(
                "name",
                "name must start with a letter, use only letters, digits and _, "
                "and be at most 24 characters (PX4's limit for a mode name) "
                f"(got {name!r})",
            )

        target = (
            self.block_ref(doc["target"], "target", "target")
            if "target" in doc
            else None
        )
        if "controller" in doc:
            controller = self.block_ref(doc["controller"], "controller", "controller")
        else:
            controller = self.block(
                DEFAULT_CONTROLLER, {}, "controller", "controller (default)"
            )

        raw_steps = doc.get("steps")
        if not isinstance(raw_steps, list) or not raw_steps:
            self.error("steps", "a mission needs a non-empty list of steps")
            return None
        steps = [
            s
            for s in (self.step(i, node) for i, node in enumerate(raw_steps))
            if s is not None
        ]

        seen: set[str] = set()
        for step_name in self.step_names:
            if step_name in seen:
                self.error(
                    f"step '{step_name}'",
                    "duplicate step name; add 'name:' to one of them",
                )
            seen.add(step_name)
        for step in steps:
            if step.on_fail is not None and step.on_fail not in seen:
                self.error(
                    f"step '{step.name}'",
                    f"on_fail '{step.on_fail}' is not a step{_suggest(step.on_fail, sorted(seen))}",
                )
            if (
                step.until in ("target_seen", "target_lost")
                and step.target is None
                and target is None
            ):
                self.error(
                    f"step '{step.name}'",
                    f"until: {step.until} needs a target (at the top or on this step)",
                )
            block_cls = self.registry.get(step.block.name)
            if (
                getattr(block_cls, "needs_target", False)
                and step.target is None
                and target is None
            ):
                self.error(
                    f"step '{step.name}'",
                    f"{step.block.name} needs a target (at the top or on this step)",
                )

        if self.errors or controller is None or not isinstance(name, str):
            return None
        return MissionSpec(
            name=name, target=target, controller=controller, steps=tuple(steps)
        )


def _type_ok(expected: object, value: object) -> bool:
    if expected is float:
        return _is_number(value)
    if expected is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if expected in (bool, str):
        return isinstance(value, expected)  # ty: ignore[invalid-argument-type]
    return True


def _type_name(expected: object) -> str:
    return getattr(expected, "__name__", str(expected))


def parse_mission(
    text: str, registry: Registry = REGISTRY, source: str = "<mission>"
) -> MissionSpec:
    try:
        doc = yaml.safe_load(text)
    except yaml.YAMLError as err:
        raise MissionError([f"{source}: invalid YAML: {err}"]) from err
    parser = _Parser(registry, source)
    spec = parser.mission(doc)
    if spec is None:
        raise MissionError(parser.errors)
    return spec


def load_mission(path: str | Path, registry: Registry = REGISTRY) -> MissionSpec:
    return parse_mission(
        Path(path).read_text(encoding="utf-8"), registry, source=str(path)
    )
