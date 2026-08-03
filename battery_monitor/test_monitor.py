"""Tests for the threshold state machine, driven by a fake INA238.

The alerting logic is the part that has to be right when nobody is watching,
and it is the part that cannot be exercised by draining a real pack. Run with:

    python3 -m pytest battery_monitor/test_monitor.py
    python3 battery_monitor/test_monitor.py        # no pytest needed
"""

from __future__ import annotations

import copy
from pathlib import Path

from .config import DEFAULTS, Config, ConfigError, validate
from .monitor import BatteryMonitor, estimate_soc
from .notify import Notifier, level_at_least


class FakeSensor:
    """Stands in for an INA238; `volts` is set by the test."""

    def __init__(self, volts: float = 25.0, current: float = 0.0) -> None:
        self.volts = volts
        self.current = current
        self.closed = False

    def read_bus_voltage(self) -> float:
        return self.volts

    def read_current(self) -> float:
        return self.current

    def read_power(self) -> float:
        return self.volts * self.current

    def read_temperature(self) -> float:
        return 30.0

    def probe(self) -> bool:
        return True

    def close(self) -> None:
        self.closed = True


class RecordingNotifier(Notifier):
    def __init__(self) -> None:
        # Synchronous: tests assert on `sent` immediately after send(), and a
        # worker thread would make that a race. Going through the real
        # __init__ keeps the lifecycle attributes (queue, worker) present so
        # close() behaves as it does in production.
        super().__init__({}, dispatch_async=False)
        self.sent: list[tuple[str, str, str]] = []

    def send(self, level: str, title: str, body: str) -> None:
        self.sent.append((level, title, body))

    def check_availability(self) -> list[str]:
        return []


def make_config(**overrides) -> Config:
    raw = copy.deepcopy(DEFAULTS)
    for section, values in overrides.items():
        raw[section].update(values)
    cfg = Config(**raw)
    validate(cfg)
    return cfg


def make_monitor(cfg=None, volts: float = 25.0):
    cfg = cfg or make_config()
    sensor = FakeSensor(volts)
    monitor = BatteryMonitor(cfg, notifier=RecordingNotifier(), sensor=sensor)
    return monitor, sensor


def feed(monitor, sensor, volts: float, samples: int) -> list[str | None]:
    """Push `samples` readings at `volts` and collect the fired alerts."""
    sensor.volts = volts
    return [monitor.update(monitor.read()) for _ in range(samples)]


# --- Threshold crossing ----------------------------------------------------

def test_ok_above_all_thresholds():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 24.0, 10)
    assert monitor.level == "ok", monitor.level


def test_each_level_fires_in_order():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 24.0, 5)
    for volts, expected in [(22.0, "warn"), (20.8, "land"),
                            (20.2, "critical"), (19.5, "min")]:
        fired = feed(monitor, sensor, volts, 20)
        assert monitor.level == expected, f"{volts} V -> {monitor.level}, want {expected}"
        assert expected in fired, f"{volts} V never fired {expected}: {fired}"


def test_exact_threshold_counts_as_tripped():
    # A pack sitting exactly on 21.00 V is at the land threshold, not above
    # it. Smoothing is disabled here: an EMA falling toward 21.0 approaches it
    # asymptotically from above and never actually reaches equality, so with
    # it on this would test the filter rather than the comparison.
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 3})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 24.0, 5)
    feed(monitor, sensor, 21.00, 10)
    assert monitor.level == "land", monitor.level


def test_deep_drop_skips_straight_to_min():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 24.0, 5)
    fired = feed(monitor, sensor, 18.0, 30)
    assert monitor.level == "min", monitor.level
    assert "min" in fired


# --- Sag rejection ---------------------------------------------------------

def test_momentary_sag_does_not_trip():
    """A single punch-out below LAND must not fire: that is the false alarm
    that trains a pilot to ignore the alerts."""
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 3})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 24.0, 10)
    fired = feed(monitor, sensor, 20.0, 1)      # one sagging sample
    feed(monitor, sensor, 24.0, 5)              # throttle released
    assert monitor.level == "ok", monitor.level
    assert all(f is None or f == "ok" for f in fired), fired


def test_sustained_sag_does_trip():
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 3})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 24.0, 10)
    feed(monitor, sensor, 20.8, 5)
    assert monitor.level == "land", monitor.level


def test_smoothing_damps_a_spike():
    cfg = make_config(sampling={"smoothing": 0.25, "confirm_samples": 1})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 24.0, 20)
    reading = monitor.read()
    sensor.volts = 15.0
    spiked = monitor.read()
    assert spiked.voltage > 20.0, f"EMA barely damped the spike: {spiked.voltage}"
    assert spiked.voltage < reading.voltage


# --- Hysteresis and latching ----------------------------------------------

def test_hysteresis_prevents_chatter():
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 1,
                                "hysteresis": 0.04})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 20.9, 5)
    assert monitor.level == "land"
    # 21.10 V is above the 21.00 trip point but inside the 0.24 V band.
    feed(monitor, sensor, 21.10, 5)
    assert monitor.level == "land", "recovered without clearing hysteresis"
    feed(monitor, sensor, 21.50, 5)
    assert monitor.level == "warn", monitor.level


def test_worst_level_latches_after_recovery():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 20.2, 10)
    assert monitor.worst == "critical"
    feed(monitor, sensor, 24.0, 20)
    assert monitor.level == "ok"
    assert monitor.worst == "critical", "worst level must survive recovery"


def test_recovery_notice_fires_once():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 22.0, 10)
    fired = feed(monitor, sensor, 24.0, 20)
    assert fired.count("ok") == 1, fired


# --- Repeat cadence --------------------------------------------------------

def test_alert_repeats_on_cadence():
    cfg = make_config(sampling={"interval": 1.0, "confirm_samples": 1, "smoothing": 1.0})
    monitor, sensor = make_monitor(cfg)
    sensor.volts = 20.8
    reading = monitor.read()
    assert monitor.update(reading) == "land"

    # Same level well inside the repeat window: silent.
    reading.timestamp += 10
    assert monitor.update(reading) is None
    # Past the 45 s land cadence: fires again.
    reading.timestamp += 40
    assert monitor.update(reading) == "land"


def test_more_severe_repeats_faster():
    repeats = DEFAULTS["alerts"]["repeat_seconds"]
    assert repeats["warn"] > repeats["land"] > repeats["critical"] > repeats["min"]


# --- Pack presence ---------------------------------------------------------

def test_absent_pack_stays_quiet():
    monitor, sensor = make_monitor()
    fired = feed(monitor, sensor, 0.0, 20)
    assert monitor.level == "ok"
    assert all(f is None for f in fired), fired


def test_reconnect_does_not_fire_phantom_alert():
    """Unplug at 24 V, reconnect at 24 V: the EMA must not carry the 0 V
    samples across and briefly report CRITICAL."""
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 24.0, 10)
    feed(monitor, sensor, 0.0, 10)
    fired = feed(monitor, sensor, 24.0, 10)
    assert monitor.level == "ok", monitor.level
    assert all(f is None or f == "ok" for f in fired), fired


# --- Trend / ETA -----------------------------------------------------------

def test_trend_slope_is_negative_while_discharging():
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 1})
    monitor, sensor = make_monitor(cfg)
    volts = 24.0
    for i in range(40):
        sensor.volts = volts - i * 0.01
        reading = monitor.read()
        monitor._history[-1] = (monitor._history[-1][0] + i * 2.0, reading.voltage)
    slope = monitor.volts_per_minute()
    assert slope is not None and slope < 0, slope


def test_eta_to_land_is_finite_while_discharging():
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 1})
    monitor, sensor = make_monitor(cfg)
    for i in range(40):
        sensor.volts = 24.0 - i * 0.01
        reading = monitor.read()
        monitor._history[-1] = (monitor._history[-1][0] + i * 2.0, reading.voltage)
    eta = monitor.minutes_to("land", reading)
    assert eta is not None and eta > 0, eta


def test_no_eta_while_idle():
    cfg = make_config(sampling={"smoothing": 1.0, "confirm_samples": 1})
    monitor, sensor = make_monitor(cfg)
    feed(monitor, sensor, 24.0, 40)
    assert monitor.minutes_to("land", monitor.read()) is None


# --- Sag compensation ------------------------------------------------------

def test_sag_compensation_raises_effective_voltage():
    cfg = make_config(
        battery={"internal_resistance": 0.02},
        sampling={"sag_compensate": True, "smoothing": 1.0})
    monitor, sensor = make_monitor(cfg)
    sensor.volts, sensor.current = 21.0, 30.0
    reading = monitor.read()
    assert abs(reading.voltage - (21.0 + 30.0 * 0.02)) < 1e-6, reading.voltage
    assert reading.bus_voltage == 21.0, "raw reading must be preserved"


# --- Cell-count detection --------------------------------------------------

def test_detects_cells_on_an_unambiguous_voltage():
    # 14.8 V only divides into the LiPo 3.00-4.25 band at 4S (3.70 V/cell).
    monitor, sensor = make_monitor(volts=14.8)
    assert monitor.detect_cells(monitor.read()) == 4


def test_configured_count_is_plausible_when_correct():
    monitor, sensor = make_monitor(volts=24.3)   # 6S LiPo at 4.05 V/cell
    assert monitor.configured_cells_plausible(monitor.read())


def test_configured_count_is_implausible_when_wrong():
    # 6S configured, but 12.4 V is 2.07 V/cell -- no LiPo reads that low.
    monitor, sensor = make_monitor(volts=12.4)
    assert not monitor.configured_cells_plausible(monitor.read())


def test_low_liion_pack_does_not_false_alarm_on_cell_count():
    """Regression: a 6S2P Li-ion pack down at 16.2 V divides just as neatly
    by 4 (4.05 V/cell). Best-fit detection alone would report a bogus cell
    mismatch precisely when the pack is nearly flat and the real alerts
    matter most -- so plausibility of the *configured* count is what gates
    the warning."""
    cfg = make_config(battery={"chemistry": "li-ion", "cells": 6, "cell_nominal": 3.60},
                      thresholds={"warn": 3.00, "land": 2.70,
                                  "critical": 2.60, "min": 2.55})
    monitor, sensor = make_monitor(cfg, volts=16.2)
    assert monitor.configured_cells_plausible(monitor.read()), \
        "2.70 V/cell is normal for Li-ion and must not read as a wrong cell count"


def test_absent_pack_is_never_a_cell_mismatch():
    monitor, sensor = make_monitor(volts=0.0)
    assert monitor.configured_cells_plausible(monitor.read())
    assert monitor.detect_cells(monitor.read()) is None


# --- Config validation -----------------------------------------------------

def _expect_config_error(**overrides) -> str:
    try:
        make_config(**overrides)
    except ConfigError as exc:
        return str(exc)
    raise AssertionError(f"expected ConfigError for {overrides}")


def test_rejects_out_of_order_thresholds():
    msg = _expect_config_error(thresholds={"land": 3.20})   # below critical
    assert "critical" in msg and "land" in msg, msg


def test_rejects_pack_volts_where_cell_volts_belong():
    """The likeliest edit mistake: typing 21.0 into thresholds.land."""
    msg = _expect_config_error(thresholds={"land": 21.0})
    assert "PER CELL" in msg, msg


def test_rejects_absurd_cell_count():
    _expect_config_error(battery={"cells": 0})
    _expect_config_error(battery={"cells": 6.5})


def test_rejects_bad_min_level():
    _expect_config_error(notifications={"wall": {"enabled": True, "min_level": "urgent"}})



def test_rejects_liion_thresholds_on_a_lipo_pack():
    """The mix-up that motivates the floor check: 2.55 V/cell is routine for
    Li-ion and destroys a LiPo. Caught, with the fix named in the message."""
    msg = _expect_config_error(thresholds={"warn": 3.00, "land": 2.70,
                                           "critical": 2.60, "min": 2.55})
    assert "discharge floor" in msg
    assert "chemistry: li-ion" in msg, msg


def test_accepts_liion_thresholds_when_chemistry_says_so():
    cfg = make_config(battery={"chemistry": "li-ion"},
                      thresholds={"warn": 3.00, "land": 2.70,
                                  "critical": 2.60, "min": 2.55})
    assert abs(cfg.pack_threshold("land") - 16.2) < 1e-9
    assert abs(cfg.pack_threshold("min") - 15.3) < 1e-9


def test_rejects_below_the_liion_floor_too():
    msg = _expect_config_error(battery={"chemistry": "li-ion"},
                               thresholds={"warn": 3.00, "land": 2.70,
                                           "critical": 2.40, "min": 2.30})
    assert "discharge floor" in msg


def test_rejects_unknown_chemistry():
    msg = _expect_config_error(battery={"chemistry": "lifepo4"})
    assert "chemistry" in msg


def test_bench_config_on_disk_is_valid():
    """The real config file must load and match the pilot's stated numbers."""
    from .config import load
    cfg = load()
    assert cfg.cells == 6
    assert abs(cfg.pack_threshold("land") - 16.2) < 1e-6, cfg.pack_threshold("land")
    assert abs(cfg.pack_threshold("min") - 15.3) < 1e-6, cfg.pack_threshold("min")


def test_defaults_are_valid():
    # Compared with a tolerance: 3.30 * 6 is 19.799999999999997 in binary
    # floating point. The 3e-15 V error is far below the sensor's 3.125 mV
    # LSB, so it is not worth distorting the arithmetic to hide.
    cfg = make_config()
    assert abs(cfg.pack_threshold("land") - 21.0) < 1e-9
    assert abs(cfg.pack_threshold("min") - 19.8) < 1e-9


# --- Helpers ---------------------------------------------------------------

def test_soc_curve_is_monotonic():
    for chemistry in ("lipo", "li-ion"):
        previous = -1.0
        v = 2.4
        while v <= 4.25:
            soc = estimate_soc(v, chemistry)
            assert soc >= previous, f"{chemistry}: SoC dropped at {v:.2f} V"
            assert 0.0 <= soc <= 1.0
            previous = soc
            v += 0.01


def test_liion_curve_reports_capacity_below_the_lipo_floor():
    """The bug this curve split fixes: a Li-ion pack in its normal 2.5-3.3 V
    working range scored 0% on the LiPo curve."""
    for v in (2.80, 3.00, 3.20):
        assert estimate_soc(v, "lipo") == 0.0, f"LiPo really is empty at {v} V"
        assert estimate_soc(v, "li-ion") > 0.0, f"Li-ion has capacity left at {v} V"
    assert estimate_soc(3.20, "li-ion") > estimate_soc(2.80, "li-ion")


def test_both_chemistries_agree_at_the_extremes():
    for chemistry in ("lipo", "li-ion"):
        assert estimate_soc(4.20, chemistry) == 1.0
        assert estimate_soc(1.00, chemistry) == 0.0


def test_unknown_chemistry_falls_back_to_lipo():
    assert estimate_soc(3.80, "nickel-unobtainium") == estimate_soc(3.80, "lipo")


def test_level_at_least_ordering():
    assert level_at_least("critical", "warn")
    assert level_at_least("warn", "warn")
    assert not level_at_least("warn", "land")


def test_desktop_does_not_cache_a_missing_session_bus():
    """Regression: this service starts at boot before the desktop session
    exists. Memoizing that miss left the popup channel dead for the whole
    uptime even after login -- silently, since wall still worked."""
    import os as _os

    from .notify import DesktopChannel

    channel = DesktopChannel({"enabled": True})
    saved = _os.environ.pop("DBUS_SESSION_BUS_ADDRESS", None)
    try:
        # Point at a uid whose runtime dir cannot exist, simulating pre-login.
        channel.settings["user"] = None
        original_getuid = _os.getuid
        _os.getuid = lambda: 999999
        try:
            env = channel._session_env()
            assert "DBUS_SESSION_BUS_ADDRESS" not in env
            assert channel._env is None, "a failed bus lookup must not be cached"
        finally:
            _os.getuid = original_getuid

        # Once the bus appears, the next call must pick it up and cache it.
        env = channel._session_env()
        if Path(f"/run/user/{_os.getuid()}/bus").exists():
            assert "DBUS_SESSION_BUS_ADDRESS" in env
            assert channel._env is not None, "a successful lookup should be cached"
    finally:
        if saved is not None:
            _os.environ["DBUS_SESSION_BUS_ADDRESS"] = saved


def test_slow_channel_does_not_block_the_caller():
    """Regression: dispatch used to run inline, so a channel blocking on its
    timeout stalled the sampling loop -- worst at MIN, where alerts repeat
    every 10 s and the readings matter most."""
    import time as _time

    from .notify import Channel, Notifier

    class SlowChannel(Channel):
        name = "slow"

        def __init__(self):
            super().__init__({"enabled": True, "min_level": "warn"})
            self.delivered = 0

        def _send(self, level, title, body):
            _time.sleep(0.4)
            self.delivered += 1

    notifier = Notifier({})
    slow = SlowChannel()
    notifier.channels = [slow]

    started = _time.monotonic()
    for _ in range(3):
        notifier.send("land", "t", "b")
    enqueue_time = _time.monotonic() - started
    assert enqueue_time < 0.1, f"send() blocked for {enqueue_time:.2f}s"

    assert notifier.flush(timeout=10.0), "queue never drained"
    assert slow.delivered == 3, slow.delivered
    notifier.close()


def test_notification_backlog_is_bounded():
    """A wedged channel must not grow memory without limit on a flight
    computer -- alerts are dropped with a log line instead."""
    import time as _time

    from .notify import Channel, Notifier

    class WedgedChannel(Channel):
        name = "wedged"

        def __init__(self):
            super().__init__({"enabled": True, "min_level": "warn"})
            self.release = False

        def _send(self, level, title, body):
            while not self.release:
                _time.sleep(0.01)

    notifier = Notifier({})
    wedged = WedgedChannel()
    notifier.channels = [wedged]
    try:
        for _ in range(200):
            notifier.send("land", "t", "b")
        assert notifier._dropped > 0, "unbounded queue: nothing was dropped"
        assert notifier._queue.qsize() <= 16, notifier._queue.qsize()
    finally:
        wedged.release = True
        notifier.close(timeout=2.0)


def test_synchronous_mode_still_delivers():
    from .notify import Channel, Notifier

    class CountingChannel(Channel):
        name = "counting"

        def __init__(self):
            super().__init__({"enabled": True, "min_level": "warn"})
            self.delivered = 0

        def _send(self, level, title, body):
            self.delivered += 1

    notifier = Notifier({}, dispatch_async=False)
    counting = CountingChannel()
    notifier.channels = [counting]
    notifier.send("land", "t", "b")
    assert counting.delivered == 1
    assert notifier.flush()


def test_startup_notice_waits_for_channels_then_fires_once():
    """Regression: at boot the desktop session does not exist yet, so firing
    the startup notice immediately lost it on every cold start."""
    cfg = make_config(alerts={"start_notice_timeout": 120.0})
    monitor, sensor = make_monitor(cfg)

    unavailable = ["desktop: no session bus yet"]
    monitor.notifier.check_availability = lambda: list(unavailable)

    monitor.announce_startup(monitor.read())
    assert monitor._pending_start_notice is not None, "notice should be deferred"

    monitor._flush_start_notice()
    assert not monitor.notifier.sent, "must not fire while a channel is down"

    unavailable.clear()                       # the desktop logs in
    monitor._last_channel_poll = 0.0          # bypass the 5 s poll throttle
    monitor._flush_start_notice()
    assert len(monitor.notifier.sent) == 1, monitor.notifier.sent
    assert monitor.notifier.sent[0][1] == "Battery monitor started"

    monitor._last_channel_poll = 0.0
    monitor._flush_start_notice()
    assert len(monitor.notifier.sent) == 1, "startup notice must fire only once"


def test_startup_notice_fires_anyway_after_the_timeout():
    """A headless machine that never logs in must still get the notice on the
    channels that do work, rather than holding it forever."""
    cfg = make_config(alerts={"start_notice_timeout": 0.0})
    monitor, sensor = make_monitor(cfg)
    monitor.notifier.check_availability = lambda: ["desktop: never coming up"]

    monitor.announce_startup(monitor.read())
    monitor._flush_start_notice()
    assert len(monitor.notifier.sent) == 1, "timeout should release the notice"





def test_desktop_tracks_a_replaces_id():
    """Critical popups persist until dismissed, so repeats must replace the
    previous popup rather than stack dozens of them on screen."""
    from .notify import DesktopChannel

    channel = DesktopChannel({"enabled": True})
    assert channel._notify_id == 0, "first notify must ask the server for a new id"
    # Urgency must map to the byte the D-Bus spec expects, not the string form.
    assert channel._URGENCY_BYTE["land"] == 2
    assert channel._URGENCY_BYTE["min"] == 2
    assert channel._URGENCY_BYTE["ok"] == 0


def test_missing_pygobject_falls_back_permanently():
    from .notify import DesktopChannel

    channel = DesktopChannel({"enabled": True})
    channel._dbus_broken = True
    assert channel._send_via_dbus("land", "t", "b") is False



def test_describe_includes_pack_and_cell_voltage():
    monitor, sensor = make_monitor()
    feed(monitor, sensor, 20.8, 10)
    title, body = monitor.describe(monitor.read(), "land")
    assert "LAND NOW" in title
    assert "20.80 V pack" in body, body
    assert "3.47 V/cell" in body, body


def test_close_releases_the_sensor():
    monitor, sensor = make_monitor()
    monitor.close()
    assert sensor.closed


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items())
             if n.startswith("test_") and callable(f)]
    failures = []
    for name, fn in tests:
        try:
            fn()
            print(f"  PASS  {name}")
        except Exception as exc:  # noqa: BLE001
            failures.append((name, exc))
            print(f"  FAIL  {name}: {exc}")
    print(f"\n{len(tests) - len(failures)}/{len(tests)} passed")
    raise SystemExit(1 if failures else 0)
