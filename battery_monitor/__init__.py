"""Drone pack voltage monitoring and alerting over the INA238.

See README.md in this directory. Thresholds live in
`config/battery_monitor.yaml`; the systemd unit is `services/battery_monitor.service.in`.
"""

from .config import LEVELS, Config, ConfigError, load
from .ina238 import INA238, INA238Error
from .monitor import BatteryMonitor, Reading, estimate_soc
from .notify import Notifier

__all__ = [
    "LEVELS",
    "BatteryMonitor",
    "Config",
    "ConfigError",
    "INA238",
    "INA238Error",
    "Notifier",
    "Reading",
    "estimate_soc",
    "load",
]
