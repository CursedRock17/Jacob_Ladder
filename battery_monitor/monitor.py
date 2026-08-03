"""Battery voltage state machine and sampling loop.

Design notes -- the reason this is more than a threshold comparison:

* **Sag.** Pack voltage drops under load. A throttle punch can push a healthy
  6S pack momentarily below the LAND trip point. Raw comparison would cry
  wolf every aggressive manoeuvre and get ignored -- so voltage is smoothed
  with an EMA and a level must hold for `confirm_samples` consecutive samples
  before it becomes active. Optionally the pack's internal resistance is used
  to reconstruct open-circuit voltage.
* **Hysteresis.** Recovering out of a level needs the voltage back above the
  trip point plus a margin, so a pack hovering on a boundary does not chatter.
* **Latching.** The worst level reached is remembered for the session even
  after recovery. A pack that touched CRITICAL under load has been stressed;
  that fact must survive the load going away.
"""

from __future__ import annotations

import csv
import logging
import time
from dataclasses import dataclass
from pathlib import Path

from .config import LEVELS, Config
from .ina238 import INA238, INA238Error
from .notify import Notifier

log = logging.getLogger("battery_monitor")

# Per-cell resting voltage -> state of charge, interpolated linearly between
# points. Only meaningful at low current; labelled an estimate everywhere it
# is surfaced.
#
# The two chemistries need separate curves. A LiPo is empty at 3.30 V/cell,
# where an 18650/21700 Li-ion still has roughly a fifth of its capacity left
# and keeps going to 2.50 V. Scoring a Li-ion pack against the LiPo curve
# reports 0% for the whole bottom of its usable range.
_SOC_CURVES = {
    "lipo": [
        (3.30, 0.00), (3.50, 0.05), (3.60, 0.10), (3.70, 0.20), (3.75, 0.30),
        (3.80, 0.40), (3.85, 0.50), (3.90, 0.60), (3.95, 0.70), (4.00, 0.80),
        (4.08, 0.90), (4.15, 0.95), (4.20, 1.00),
    ],
    "li-ion": [
        (2.50, 0.00), (2.60, 0.01), (2.70, 0.02), (2.80, 0.03), (2.90, 0.05),
        (3.00, 0.08), (3.10, 0.12), (3.20, 0.16), (3.30, 0.22), (3.40, 0.29),
        (3.50, 0.38), (3.60, 0.48), (3.70, 0.58), (3.80, 0.68), (3.90, 0.77),
        (4.00, 0.85), (4.10, 0.92), (4.20, 1.00),
    ],
}

# Plausible resting per-cell voltage, used to sanity-check the configured
# series count. Li-ion runs lower, so the band has to follow the chemistry or
# a legitimately low Li-ion pack reads as a smaller LiPo pack.
_PLAUSIBLE_CELL_RANGE = {
    "lipo": (3.00, 4.25),
    "li-ion": (2.45, 4.25),
}

_HEADLINE = {
    "ok": "Battery OK",
    "warn": "BATTERY LOW",
    "land": "LAND NOW",
    "critical": "BATTERY CRITICAL",
    "min": "BATTERY BELOW MINIMUM - CELL DAMAGE",
}

_ADVICE = {
    "warn": "Begin wrapping up the flight.",
    "land": "Bring it down now -- this is the land threshold.",
    "critical": "Land immediately. Pack is below the critical threshold.",
    "min": "Disconnect and charge NOW. Every second below this damages cells.",
}


def estimate_soc(cell_voltage: float, chemistry: str = "lipo") -> float:
    """State of charge 0.0-1.0 from resting per-cell voltage."""
    curve = _SOC_CURVES.get(chemistry, _SOC_CURVES["lipo"])
    if cell_voltage <= curve[0][0]:
        return 0.0
    if cell_voltage >= curve[-1][0]:
        return 1.0
    for (v_lo, s_lo), (v_hi, s_hi) in zip(curve, curve[1:]):
        if v_lo <= cell_voltage <= v_hi:
            span = v_hi - v_lo
            return s_lo + (s_hi - s_lo) * ((cell_voltage - v_lo) / span if span else 0.0)
    return 0.0


@dataclass
class Reading:
    timestamp: float
    bus_voltage: float       # as measured
    voltage: float           # smoothed, sag-compensated -- what levels use
    current: float
    power: float
    temperature: float
    cells: int
    present: bool
    chemistry: str = "lipo"

    @property
    def cell_voltage(self) -> float:
        return self.voltage / self.cells if self.cells else 0.0

    @property
    def soc(self) -> float:
        return estimate_soc(self.cell_voltage, self.chemistry)


class BatteryMonitor:
    """Samples the INA238 and raises alerts as the pack crosses trip points."""

    def __init__(self, cfg: Config, notifier: Notifier | None = None,
                 sensor: INA238 | None = None) -> None:
        self.cfg = cfg
        self.notifier = notifier or Notifier(cfg.notifications)
        self.sensor = sensor or self._open_sensor()

        self.level = "ok"            # confirmed, currently-active level
        self.worst = "ok"            # latched worst level this session
        self._candidate = "ok"       # level seen but not yet confirmed
        self._candidate_count = 0
        self._last_alert: dict[str, float] = {}
        self._smoothed: float | None = None
        self._history: list[tuple[float, float]] = []   # (t, volts) for trend
        self._csv_file = None
        self._csv_writer = None
        self._last_csv = 0.0
        self._last_heartbeat = 0.0
        self._read_errors = 0
        self._pending_start_notice: tuple[str, str] | None = None
        self._start_notice_deadline = 0.0
        self._last_channel_poll = 0.0

    def _open_sensor(self) -> INA238:
        s = self.cfg.sensor
        sensor = INA238.from_shunt_rating(
            bus_num=int(s["bus"]),
            address=int(s["address"]),
            r_shunt=float(s["shunt_resistance"]),
            power_rating=float(s["shunt_power_rating"]),
        )
        if not sensor.probe():
            log.warning(
                "device at i2c-%s 0x%02X did not identify as a TI part -- "
                "check sensor.bus / sensor.address", s["bus"], int(s["address"]))
        return sensor

    # --- Sampling ----------------------------------------------------------
    def read(self) -> Reading:
        """One sample: raw read, sag compensation, EMA smoothing."""
        now = time.time()
        bus_v = self.sensor.read_bus_voltage()
        current = self.sensor.read_current()
        power = self.sensor.read_power()
        temperature = self.sensor.read_temperature()

        present = bus_v >= float(self.cfg.battery["present_min_voltage"])

        # Reconstruct open-circuit voltage from the measured sag, when the
        # pack's internal resistance is known. Off by default: a wrong
        # r_internal makes the estimate worse than the raw reading.
        effective = bus_v
        if self.cfg.sampling.get("sag_compensate"):
            r_int = float(self.cfg.battery.get("internal_resistance") or 0.0)
            if r_int > 0 and current > 0:
                effective = bus_v + current * r_int

        if not present:
            # A disconnected pack must not drag the average down and then
            # trigger a phantom CRITICAL when it is plugged back in.
            self._smoothed = None
            self._history.clear()
            smoothed = bus_v
        else:
            alpha = float(self.cfg.sampling["smoothing"])
            self._smoothed = (effective if self._smoothed is None
                              else alpha * effective + (1 - alpha) * self._smoothed)
            smoothed = self._smoothed
            self._history.append((now, smoothed))
            window = float(self.cfg.sampling.get("trend_window") or 60.0)
            cutoff = now - window
            self._history = [p for p in self._history if p[0] >= cutoff]

        return Reading(now, bus_v, smoothed, current, power, temperature,
                       self.cfg.cells, present, self.cfg.chemistry)

    def cell_range(self) -> tuple[float, float]:
        """Plausible resting per-cell voltage band for the configured chemistry."""
        return _PLAUSIBLE_CELL_RANGE.get(self.cfg.chemistry,
                                         _PLAUSIBLE_CELL_RANGE["lipo"])

    def configured_cells_plausible(self, reading: Reading) -> bool:
        """True if the configured series count explains the measured voltage."""
        if not reading.present:
            return True
        low, high = self.cell_range()
        return low <= reading.bus_voltage / self.cfg.cells <= high

    def detect_cells(self, reading: Reading) -> int | None:
        """Infer series count from a resting pack, to catch a mis-set config.

        Picks the count whose implied per-cell voltage lands in the plausible
        band for the chemistry and sits closest to nominal. Returns None when
        no count fits.

        Voltage alone cannot uniquely identify a series count -- a 6S Li-ion
        at 16.2 V divides just as neatly by 4. So this is only ever consulted
        after `configured_cells_plausible` has already ruled the configured
        count out; see `announce_startup`.
        """
        if not reading.present:
            return None
        low, high = self.cell_range()
        nominal = self.cfg.battery.get("cell_nominal") or 3.7
        best, best_err = None, None
        for n in range(1, 25):
            per_cell = reading.bus_voltage / n
            if not low <= per_cell <= high:
                continue
            err = abs(per_cell - nominal)
            if best_err is None or err < best_err:
                best, best_err = n, err
        return best

    # --- State machine -----------------------------------------------------
    def classify(self, reading: Reading) -> str:
        """Most severe level whose trip point the reading is at or below.

        Hysteresis applies only to leaving the *current* level upward, so
        crossing further down always registers immediately.
        """
        if not reading.present:
            return "ok"

        hysteresis = float(self.cfg.sampling.get("hysteresis") or 0.0) * reading.cells
        thresholds = self.cfg.pack_thresholds()

        level = "ok"
        for candidate in LEVELS[1:]:
            if reading.voltage <= thresholds[candidate]:
                level = candidate

        # Recovering upward out of the active level needs the extra margin.
        if LEVELS.index(level) < LEVELS.index(self.level) and self.level != "ok":
            if reading.voltage < thresholds[self.level] + hysteresis:
                return self.level
        return level

    def update(self, reading: Reading) -> str | None:
        """Fold a reading into the state machine.

        Returns the level whose alert should fire now, or None.
        """
        observed = self.classify(reading)

        if observed == self._candidate:
            self._candidate_count += 1
        else:
            self._candidate = observed
            self._candidate_count = 1

        confirmed = self.level
        if self._candidate_count >= int(self.cfg.sampling["confirm_samples"]):
            confirmed = self._candidate

        previous = self.level
        self.level = confirmed
        if LEVELS.index(confirmed) > LEVELS.index(self.worst):
            self.worst = confirmed

        if confirmed != previous:
            if confirmed == "ok":
                return "ok" if self.cfg.alerts.get("notify_on_recover", True) else None
            self._last_alert[confirmed] = reading.timestamp
            return confirmed

        # Same level: re-alert on the level's repeat cadence.
        if confirmed != "ok":
            repeat = float(self.cfg.alerts["repeat_seconds"][confirmed])
            last = self._last_alert.get(confirmed, 0.0)
            if repeat > 0 and reading.timestamp - last >= repeat:
                self._last_alert[confirmed] = reading.timestamp
                return confirmed
        return None

    # --- Trend -------------------------------------------------------------
    def volts_per_minute(self) -> float | None:
        """Least-squares slope of pack voltage over the trend window, V/min.

        Negative means discharging. None until there is enough spread to fit.
        """
        pts = self._history
        if len(pts) < 5:
            return None
        t0 = pts[0][0]
        xs = [p[0] - t0 for p in pts]
        ys = [p[1] for p in pts]
        if xs[-1] - xs[0] < 10.0:
            return None
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom == 0:
            return None
        slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
        return slope * 60.0

    def minutes_to(self, level: str, reading: Reading) -> float | None:
        """Extrapolated minutes until `level` is reached at the current rate."""
        slope = self.volts_per_minute()
        if slope is None or slope >= -1e-4:
            return None
        target = self.cfg.pack_threshold(level)
        remaining = reading.voltage - target
        if remaining <= 0:
            return 0.0
        return remaining / -slope

    # --- Presentation ------------------------------------------------------
    def describe(self, reading: Reading, level: str) -> tuple[str, str]:
        """(title, body) for an alert at `level`."""
        if level == "ok":
            title = "Battery recovered"
            body = (f"{reading.voltage:.2f} V ({reading.cell_voltage:.2f} V/cell) "
                    f"is back above the {self.cfg.thresholds['warn']:.2f} V/cell warn point.")
            if self.worst != "ok":
                body += f"\nWorst level reached this session: {self.worst.upper()}."
            return title, body

        title = _HEADLINE[level]
        lines = [
            f"{reading.voltage:.2f} V pack  |  {reading.cell_voltage:.2f} V/cell "
            f"({reading.cells}S)  |  ~{reading.soc * 100:.0f}% remaining",
            f"{reading.current:.1f} A  {reading.power:.0f} W  {reading.temperature:.0f} C",
        ]

        if level != "min":
            nxt = LEVELS[LEVELS.index(level) + 1]
            eta = self.minutes_to(nxt, reading)
            headroom = reading.voltage - self.cfg.pack_threshold(nxt)
            note = (f"{headroom:.2f} V above {nxt.upper()} "
                    f"({self.cfg.pack_threshold(nxt):.2f} V)")
            if eta is not None:
                note += f" -- about {eta:.1f} min at the current rate"
            lines.append(note)

        lines.append(_ADVICE[level])
        return title, "\n".join(lines)

    def status_line(self, reading: Reading) -> str:
        if not reading.present:
            return f"no pack detected ({reading.bus_voltage:.2f} V on the bus)"
        slope = self.volts_per_minute()
        trend = f"{slope:+.3f} V/min" if slope is not None else "trend n/a"
        return (f"{reading.voltage:5.2f} V  {reading.cell_voltage:4.2f} V/cell  "
                f"{reading.current:6.2f} A  {reading.power:6.1f} W  "
                f"{reading.temperature:5.1f} C  soc~{reading.soc * 100:3.0f}%  "
                f"{trend}  [{self.level.upper()}]")

    # --- CSV logging -------------------------------------------------------
    def _log_csv(self, reading: Reading) -> None:
        path = self.cfg.logging.get("csv_path")
        if not path:
            return
        if reading.timestamp - self._last_csv < float(self.cfg.logging.get("csv_interval") or 0):
            return
        self._last_csv = reading.timestamp
        if self._csv_writer is None:
            target = Path(path).expanduser()
            target.parent.mkdir(parents=True, exist_ok=True)
            new = not target.exists() or target.stat().st_size == 0
            self._csv_file = target.open("a", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            if new:
                self._csv_writer.writerow(
                    ["timestamp", "bus_voltage", "voltage", "cell_voltage",
                     "current_a", "power_w", "temp_c", "soc", "level"])
        self._csv_writer.writerow([
            f"{reading.timestamp:.3f}", f"{reading.bus_voltage:.4f}",
            f"{reading.voltage:.4f}", f"{reading.cell_voltage:.4f}",
            f"{reading.current:.4f}", f"{reading.power:.3f}",
            f"{reading.temperature:.2f}", f"{reading.soc:.3f}", self.level,
        ])
        self._csv_file.flush()

    # --- Main loop ---------------------------------------------------------
    def announce_startup(self, reading: Reading) -> None:
        thresholds = self.cfg.pack_thresholds()
        log.info("Configured for a %dS pack (%s)", reading.cells,
                 self.cfg.source or "built-in defaults")
        for level in LEVELS[1:]:
            log.info("  %-8s %6.2f V pack  (%.2f V/cell)",
                     level.upper(), thresholds[level], self.cfg.thresholds[level])

        # Only complain when the configured count cannot explain the measured
        # voltage. Checking "did the best-fit guess differ" instead would fire
        # every time a Li-ion pack sat low enough to also divide neatly by a
        # smaller count -- a false alarm exactly when the pack is nearly flat
        # and the real alerts matter most.
        if self.cfg.battery.get("detect_cells") and not self.configured_cells_plausible(reading):
            detected = self.detect_cells(reading)
            if detected and detected != reading.cells:
                # Wrong cell count silently scales every trip point -- exactly
                # the failure this tool is meant to catch, so it is loud.
                message = (
                    f"Config says {reading.cells}S but {reading.bus_voltage:.2f} V "
                    f"looks like {detected}S ({reading.bus_voltage / detected:.2f} V/cell). "
                    f"Every threshold is scaled by cell count -- fix battery.cells in "
                    f"{self.cfg.source or 'the config'}.")
                log.warning(message)
                self.notifier.send("warn", "Battery monitor: cell count mismatch", message)

        if self.cfg.alerts.get("notify_on_start", True) and reading.present:
            # Queued rather than sent: at boot the desktop session does not
            # exist yet, and firing now would lose the popup every cold start.
            # See _flush_start_notice.
            self._pending_start_notice = (
                "Battery monitor started",
                f"{reading.voltage:.2f} V ({reading.cell_voltage:.2f} V/cell, "
                f"{reading.cells}S, ~{reading.soc * 100:.0f}%)\n"
                f"LAND at {thresholds['land']:.2f} V, MIN at {thresholds['min']:.2f} V.")
            self._start_notice_deadline = time.monotonic() + float(
                self.cfg.alerts.get("start_notice_timeout") or 0.0)

    def _flush_start_notice(self) -> None:
        """Send the deferred startup notice once channels are reachable.

        Polls availability at most every 5 s rather than every sample; the
        checks touch the filesystem and PATH, which is not worth doing at the
        sampling rate.
        """
        if self._pending_start_notice is None:
            return

        now = time.monotonic()
        expired = now >= self._start_notice_deadline
        if not expired:
            if now - self._last_channel_poll < 5.0:
                return
            self._last_channel_poll = now
            if self.notifier.check_availability():
                return

        title, body = self._pending_start_notice
        self._pending_start_notice = None
        self.notifier.send("ok", title, body)

    def step(self) -> Reading | None:
        """One sample + alert cycle. Returns None if the read failed."""
        try:
            reading = self.read()
        except INA238Error as exc:
            self._read_errors += 1
            # A transient bus glitch is common; a persistent one means the
            # monitor is blind, which the pilot must be told about.
            if self._read_errors in (1, 10) or self._read_errors % 100 == 0:
                log.error("sensor read failed (%d in a row): %s", self._read_errors, exc)
            if self._read_errors == 10:
                self.notifier.send(
                    "warn", "Battery monitor: sensor unreachable",
                    f"{self._read_errors} consecutive INA238 read failures on "
                    f"i2c-{self.cfg.sensor['bus']}. Voltage is NOT being monitored.")
            return None

        if self._read_errors:
            log.info("sensor recovered after %d failed reads", self._read_errors)
            self._read_errors = 0

        level = self.update(reading)
        if level is not None:
            title, body = self.describe(reading, level)
            log.warning("%s | %s", title, body.replace("\n", " | "))
            self.notifier.send(level, title, body)

        self._log_csv(reading)
        self._heartbeat(reading)
        self._flush_start_notice()
        return reading

    def _heartbeat(self, reading: Reading) -> None:
        """Periodic liveness line, so a quiet journal means 'nothing wrong'
        rather than 'the monitor died an hour ago'."""
        every = float(self.cfg.logging.get("heartbeat_seconds") or 0.0)
        if every <= 0:
            return
        if reading.timestamp - self._last_heartbeat < every:
            return
        self._last_heartbeat = reading.timestamp
        log.info("%s", self.status_line(reading))

    def run(self, on_sample=None) -> None:
        """Sample forever. `on_sample(reading)` is called for each good read."""
        problems = self.notifier.check_availability()
        for problem in problems:
            log.warning("notification channel unavailable -- %s", problem)

        first = None
        for _ in range(5):
            try:
                first = self.read()
                break
            except INA238Error as exc:
                log.warning("initial read failed, retrying: %s", exc)
                time.sleep(1.0)
        if first is None:
            raise INA238Error(
                f"cannot read the INA238 on i2c-{self.cfg.sensor['bus']} at "
                f"0x{int(self.cfg.sensor['address']):02X}")

        self.announce_startup(first)

        interval = float(self.cfg.sampling["interval"])
        try:
            while True:
                started = time.monotonic()
                reading = self.step()
                if reading is not None and on_sample is not None:
                    on_sample(reading)
                time.sleep(max(0.0, interval - (time.monotonic() - started)))
        finally:
            self.close()

    def close(self) -> None:
        # Notifications first: an alert raised in the same instant as the
        # SIGTERM is still queued, and must go out before we tear down.
        self.notifier.close()
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
        self.sensor.close()
