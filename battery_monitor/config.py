"""Configuration loading and validation for the battery monitor.

Everything the pilot is expected to touch -- land voltage, min voltage, cell
count -- lives in `config/battery_monitor.yaml`. This module supplies the
defaults, merges the file over them, and rejects a file that would make the
alerts meaningless (thresholds out of order, impossible cell count).
"""

from __future__ import annotations

import copy
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

# Search order for the config file when --config is not given.
CONFIG_ENV_VAR = "JL_BATTERY_CONFIG"
CONFIG_BASENAME = "battery_monitor.yaml"

DEFAULTS: dict[str, Any] = {
    "sensor": {
        "bus": 7,
        "address": 0x45,
        "shunt_resistance": 0.001,
        "shunt_power_rating": 5.0,
    },
    "battery": {
        "cells": 6,
        "chemistry": "lipo",
        "cell_full": 4.20,
        "cell_nominal": 3.70,
        "internal_resistance": 0.0,
        "detect_cells": True,
        "present_min_voltage": 5.0,
    },
    # Per-cell trip points, most severe last. Pack volts = cell volts x cells.
    "thresholds": {
        "warn": 3.70,
        "land": 3.50,
        "critical": 3.40,
        "min": 3.30,
    },
    "sampling": {
        "interval": 2.0,
        "smoothing": 0.25,
        "confirm_samples": 3,
        "hysteresis": 0.04,
        "sag_compensate": False,
        "trend_window": 60.0,
    },
    "alerts": {
        "repeat_seconds": {
            "warn": 120,
            "land": 45,
            "critical": 20,
            "min": 10,
        },
        "notify_on_recover": True,
        "notify_on_start": True,
        # At boot this service is running seconds before the desktop session
        # exists, so the startup notice would be fired into a session bus that
        # is not there yet and lost. Hold it until every enabled channel is
        # reachable, or until this many seconds have passed -- whichever comes
        # first, so a machine that never logs in still gets it on the channels
        # that do work. Threshold alerts are never delayed by this.
        "start_notice_timeout": 120.0,
    },
    "notifications": {
        "desktop": {"enabled": True, "user": None},
        "wall": {"enabled": True, "min_level": "warn"},
    },
    "logging": {
        "csv_path": None,
        "csv_interval": 10.0,
        # Periodic status line to the journal. Confirms the monitor is alive
        # and sampling during a flight where nothing crosses a threshold.
        "heartbeat_seconds": 300.0,
    },
}

# Ordered least- to most-severe. OK is implicit (index 0).
LEVELS = ("ok", "warn", "land", "critical", "min")

# Manufacturer discharge floor per cell. Below these you are damaging cells,
# so a threshold underneath one is treated as a chemistry mix-up rather than a
# deliberate choice: 2.55 V/cell is routine for Li-ion and ruinous for LiPo,
# and the two are trivially confused when thresholds are quoted as pack volts.
CHEMISTRY_FLOOR = {
    "lipo": 3.00,
    "li-ion": 2.50,
}


class ConfigError(ValueError):
    """The config file is present but unusable."""


@dataclass
class Config:
    sensor: dict[str, Any] = field(default_factory=dict)
    battery: dict[str, Any] = field(default_factory=dict)
    thresholds: dict[str, float] = field(default_factory=dict)
    sampling: dict[str, Any] = field(default_factory=dict)
    alerts: dict[str, Any] = field(default_factory=dict)
    notifications: dict[str, Any] = field(default_factory=dict)
    logging: dict[str, Any] = field(default_factory=dict)
    source: Path | None = None

    # --- Derived helpers ---------------------------------------------------
    @property
    def cells(self) -> int:
        return int(self.battery["cells"])

    @property
    def chemistry(self) -> str:
        return str(self.battery.get("chemistry", "lipo")).lower()

    def pack_threshold(self, level: str) -> float:
        """Trip point for `level` as a whole-pack voltage."""
        return self.thresholds[level] * self.cells

    def pack_thresholds(self) -> dict[str, float]:
        return {lvl: self.pack_threshold(lvl) for lvl in LEVELS if lvl != "ok"}

    @property
    def pack_full(self) -> float:
        return self.battery["cell_full"] * self.cells


def _deep_merge(base: dict, over: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in (over or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = value
    return out


def default_config_path() -> Path:
    """Resolve the config path from $JL_BATTERY_CONFIG, else the workspace."""
    env = os.environ.get(CONFIG_ENV_VAR)
    if env:
        return Path(env).expanduser()
    ws_root = os.environ.get("JL_WS_ROOT")
    root = Path(ws_root) if ws_root else Path(__file__).resolve().parent.parent
    return root / "config" / CONFIG_BASENAME


def load(path: str | os.PathLike | None = None) -> Config:
    """Load config, falling back to built-in defaults if the file is absent.

    A missing file is fine -- the defaults are a safe 6S LiPo profile. A file
    that exists but does not parse is an error: silently flying on defaults
    when the pilot believes they edited a threshold is the failure mode this
    whole tool exists to prevent.
    """
    cfg_path = Path(path) if path else default_config_path()

    raw: dict[str, Any] = {}
    source: Path | None = None
    if cfg_path.is_file():
        try:
            loaded = yaml.safe_load(cfg_path.read_text())
        except yaml.YAMLError as exc:
            raise ConfigError(f"{cfg_path}: not valid YAML: {exc}") from exc
        if loaded is not None and not isinstance(loaded, dict):
            raise ConfigError(f"{cfg_path}: top level must be a mapping")
        raw = loaded or {}
        source = cfg_path
    elif path is not None:
        raise ConfigError(f"{cfg_path}: no such config file")

    merged = _deep_merge(DEFAULTS, raw)
    cfg = Config(source=source, **{k: merged[k] for k in DEFAULTS})
    validate(cfg)
    return cfg


def validate(cfg: Config) -> None:
    where = cfg.source or "<defaults>"

    cells = cfg.battery.get("cells")
    if not isinstance(cells, int) or not 1 <= cells <= 24:
        raise ConfigError(f"{where}: battery.cells must be an integer 1-24, got {cells!r}")

    chemistry = cfg.chemistry
    if chemistry not in CHEMISTRY_FLOOR:
        raise ConfigError(
            f"{where}: battery.chemistry must be one of "
            f"{', '.join(sorted(CHEMISTRY_FLOOR))}, got {chemistry!r}")

    for level in LEVELS[1:]:
        value = cfg.thresholds.get(level)
        if not isinstance(value, (int, float)):
            raise ConfigError(f"{where}: thresholds.{level} missing or not a number")
        if not 0.5 <= value <= 5.0:
            raise ConfigError(
                f"{where}: thresholds.{level} = {value} is not a per-cell voltage. "
                f"These are volts PER CELL (e.g. 3.5), not pack volts."
            )
        floor = CHEMISTRY_FLOOR[chemistry]
        if value < floor:
            other = next((c for c, f in CHEMISTRY_FLOOR.items()
                          if c != chemistry and value >= f), None)
            hint = (f" If this is actually a {other} pack, set battery.chemistry: {other}."
                    if other else "")
            raise ConfigError(
                f"{where}: thresholds.{level} = {value:.2f} V/cell is below the "
                f"{floor:.2f} V/cell discharge floor for {chemistry} -- that damages cells."
                f"{hint}"
            )

    # Severity must strictly increase as voltage falls, or the state machine
    # can never reach the more severe levels.
    ordered = [(lvl, float(cfg.thresholds[lvl])) for lvl in LEVELS[1:]]
    for (lo_name, lo), (hi_name, hi) in zip(ordered, ordered[1:]):
        if hi >= lo:
            raise ConfigError(
                f"{where}: thresholds.{hi_name} ({hi}) must be below "
                f"thresholds.{lo_name} ({lo}) -- each level is more severe than the last"
            )

    for level in LEVELS[1:]:
        if not isinstance(cfg.alerts.get("repeat_seconds", {}).get(level), (int, float)):
            raise ConfigError(f"{where}: alerts.repeat_seconds.{level} missing or not a number")

    interval = cfg.sampling.get("interval")
    if not isinstance(interval, (int, float)) or interval <= 0:
        raise ConfigError(f"{where}: sampling.interval must be > 0")

    smoothing = cfg.sampling.get("smoothing")
    if not isinstance(smoothing, (int, float)) or not 0 < smoothing <= 1:
        raise ConfigError(f"{where}: sampling.smoothing must be in (0, 1]")

    confirm = cfg.sampling.get("confirm_samples")
    if not isinstance(confirm, int) or confirm < 1:
        raise ConfigError(f"{where}: sampling.confirm_samples must be an integer >= 1")

    for channel, settings in cfg.notifications.items():
        min_level = settings.get("min_level", "warn")
        if min_level not in LEVELS[1:]:
            raise ConfigError(
                f"{where}: notifications.{channel}.min_level must be one of "
                f"{', '.join(LEVELS[1:])}, got {min_level!r}"
            )

    unknown = set(cfg.notifications) - set(DEFAULTS["notifications"])
    if unknown:
        raise ConfigError(
            f"{where}: unknown notification channel(s) "
            f"{', '.join(sorted(unknown))} -- available: "
            f"{', '.join(sorted(DEFAULTS['notifications']))}")
