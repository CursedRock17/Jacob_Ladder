"""Command-line entry point for the Jacob's Ladder battery monitor.

    python3 -m battery_monitor                 # daemon (what the service runs)
    python3 -m battery_monitor --once          # print one reading and exit
    python3 -m battery_monitor --watch         # live single-line display
    python3 -m battery_monitor --show-config   # resolved thresholds
    python3 -m battery_monitor --test-alerts   # fire every channel at each level
"""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import time

from .config import LEVELS, ConfigError, default_config_path, load
from .ina238 import INA238Error
from .monitor import BatteryMonitor
from .notify import Notifier


def _setup_logging(verbose: bool, journal: bool) -> None:
    # systemd already stamps the journal with a timestamp and unit name, so
    # under the service the bare message is enough.
    fmt = "%(message)s" if journal else "%(asctime)s %(levelname)-7s %(message)s"
    logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO,
                        format=fmt, datefmt="%H:%M:%S", stream=sys.stdout)


def _show_config(cfg) -> int:
    packs = cfg.pack_thresholds()
    print(f"config      : {cfg.source or 'built-in defaults'}")
    print(f"sensor      : i2c-{cfg.sensor['bus']} @ 0x{int(cfg.sensor['address']):02X}, "
          f"{cfg.sensor['shunt_resistance'] * 1000:.1f} mOhm shunt")
    print(f"battery     : {cfg.cells}S, full {cfg.pack_full:.2f} V")
    print(f"sampling    : every {cfg.sampling['interval']}s, "
          f"confirm {cfg.sampling['confirm_samples']} samples, "
          f"hysteresis {cfg.sampling['hysteresis']:.2f} V/cell")
    print()
    print(f"{'LEVEL':<9} {'PACK':>8}  {'PER CELL':>9}   REPEAT")
    for level in LEVELS[1:]:
        print(f"{level.upper():<9} {packs[level]:>7.2f} V  {cfg.thresholds[level]:>7.2f} V   "
              f"every {cfg.alerts['repeat_seconds'][level]:.0f}s")
    print()
    enabled = [n for n, s in cfg.notifications.items() if s.get("enabled")]
    print(f"channels    : {', '.join(enabled) if enabled else 'none (journal only)'}")
    for problem in Notifier(cfg.notifications).check_availability():
        print(f"  UNAVAILABLE {problem}")
    return 0


def _once(monitor: BatteryMonitor) -> int:
    reading = monitor.read()
    packs = monitor.cfg.pack_thresholds()
    if not reading.present:
        print(f"No pack detected -- {reading.bus_voltage:.3f} V on the bus.")
        return 1
    level = monitor.classify(reading)
    print(f"Pack        : {reading.bus_voltage:.3f} V  "
          f"({reading.cell_voltage:.3f} V/cell, {reading.cells}S)")
    print(f"Current     : {reading.current:.3f} A   {reading.power:.1f} W")
    print(f"Temperature : {reading.temperature:.1f} C")
    print(f"Charge      : ~{reading.soc * 100:.0f}% (estimate from resting voltage)")
    print(f"Status      : {level.upper()}")
    print()
    for name in LEVELS[1:]:
        margin = reading.voltage - packs[name]
        mark = "<-- you are here" if name == level else ""
        print(f"  {name.upper():<9} {packs[name]:6.2f} V   {margin:+6.2f} V {mark}")
    return 0 if level == "ok" else 2


def _watch(monitor: BatteryMonitor) -> int:
    print(" ".join(f"{lvl.upper()}={monitor.cfg.pack_threshold(lvl):.2f}V"
                   for lvl in LEVELS[1:]))
    interval = float(monitor.cfg.sampling["interval"])
    while True:
        try:
            reading = monitor.read()
        except INA238Error as exc:
            print(f"\rread failed: {exc}", end="", flush=True)
            time.sleep(interval)
            continue
        monitor.update(reading)
        print(f"\r{monitor.status_line(reading)}   ", end="", flush=True)
        time.sleep(interval)


def _test_alerts(monitor: BatteryMonitor) -> int:
    problems = monitor.notifier.check_availability()
    for problem in problems:
        print(f"UNAVAILABLE {problem}")
    try:
        reading = monitor.read()
    except INA238Error as exc:
        print(f"sensor read failed ({exc}); using the configured pack voltage")
        reading = None
    for level in LEVELS[1:] + ("ok",):
        print(f"firing {level.upper()} ...")
        if reading is not None:
            title, body = monitor.describe(reading, level)
        else:
            title, body = f"TEST: {level.upper()}", "Battery monitor channel test."
        monitor.notifier.send(level, f"[TEST] {title}", body)
        # Delivery is asynchronous; wait for each one so the alerts arrive in
        # level order rather than all at once at the end.
        monitor.notifier.flush()
        time.sleep(2.0)
    print("done -- if you saw each one as a popup and in this terminal, "
          "the channels work.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="battery_monitor",
        description="Monitor the drone pack voltage on the INA238 and alert at "
                    "the configured land / min thresholds.")
    parser.add_argument("-c", "--config", default=None,
                        help=f"config file (default: {default_config_path()})")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--once", action="store_true",
                      help="print one reading with threshold margins, then exit")
    mode.add_argument("--watch", action="store_true",
                      help="live single-line display in this terminal")
    mode.add_argument("--show-config", action="store_true",
                      help="print the resolved thresholds and channels, then exit")
    mode.add_argument("--test-alerts", action="store_true",
                      help="fire every notification channel at every level")
    parser.add_argument("--journal", action="store_true",
                        help="log without timestamps (systemd adds its own)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    _setup_logging(args.verbose, args.journal)

    try:
        cfg = load(args.config)
    except ConfigError as exc:
        print(f"battery_monitor: {exc}", file=sys.stderr)
        return 2

    if args.show_config:
        return _show_config(cfg)

    try:
        monitor = BatteryMonitor(cfg)
    except INA238Error as exc:
        print(f"battery_monitor: {exc}", file=sys.stderr)
        return 3

    # SIGTERM is how systemd stops us; turn it into the same clean unwind as
    # Ctrl-C so the CSV log is flushed and the I2C handle closed.
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))

    try:
        if args.once:
            return _once(monitor)
        if args.watch:
            return _watch(monitor)
        if args.test_alerts:
            return _test_alerts(monitor)
        monitor.run()
    except KeyboardInterrupt:
        print()
        return 0
    except INA238Error as exc:
        logging.error("%s", exc)
        return 3
    finally:
        monitor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
