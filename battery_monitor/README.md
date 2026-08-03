# Battery Monitor

Watches drone pack voltage on the INA238 power monitor (`/dev/i2c-7`, `0x45`)
and alerts you — desktop popup and terminal broadcast — as the pack crosses
the land and minimum voltages **you** configure, so a pack never gets drained
past the point of damage again.

Runs as a systemd service, independent of ROS 2 and of the network. Low
voltage still has to be announced when everything else on the vehicle has
failed to come up.

```
sudo systemctl status battery_monitor
journalctl -u battery_monitor -f
```

## Configure

Thresholds live in [`config/battery_monitor.yaml`](../config/battery_monitor.yaml).
**Every threshold is volts per cell, not pack volts** — pack voltage is the
per-cell value times `battery.cells`. The file currently holds the 6S2P Li-ion
bench pack:

| Level | Per cell | 6S pack | Meaning |
|---|---|---|---|
| `warn` | 3.00 V | 18.00 V | Getting low, start wrapping up |
| `land` | 2.70 V | 16.20 V | Bring it down now |
| `critical` | 2.60 V | 15.60 V | Land immediately |
| `min` | 2.55 V | 15.30 V | Below this you damage cells |

Only the **series** count goes in `battery.cells`. A 2P (or 3P) pack has more
capacity at the same voltage, so parallel strings do not affect any threshold.

Each level must sit strictly below the one above it. The loader rejects a file
where they are out of order, where a pack voltage was typed into a per-cell
field, where `cells` is implausible, or where a threshold sits below the
chemistry's discharge floor — a bad config fails loudly at startup instead of
silently never alerting.

After editing: `sudo systemctl restart battery_monitor`

### Chemistry

`battery.chemistry` is `lipo` or `li-ion`, and it is not cosmetic. It selects:

- the **state-of-charge curve** — a LiPo is flat at 3.30 V/cell, where an
  18650/21700 Li-ion still holds roughly a fifth of its capacity and keeps
  going to 2.50 V. Scoring a Li-ion pack on the LiPo curve reports 0% across
  the whole bottom of its usable range.
- the **plausible-voltage band** used to sanity-check `cells`.
- the **discharge floor** thresholds are validated against — 3.00 V/cell for
  LiPo, 2.50 V/cell for Li-ion.

That last one exists because the two are easy to confuse when thresholds are
quoted as pack volts: 2.55 V/cell is routine for Li-ion and destroys a LiPo.
Setting Li-ion thresholds while `chemistry: lipo` is a startup error that names
the fix.

For reference, a conservative 6S **LiPo** profile is
`warn 3.70 / land 3.50 / critical 3.40 / min 3.30` (22.20 / 21.00 / 20.40 /
19.80 V pack) — also kept as a comment in the config file.

## Commands

```bash
python3 -m battery_monitor --show-config   # resolved pack voltages and channels
python3 -m battery_monitor --once          # one reading + margin to each level
python3 -m battery_monitor --watch         # live one-line display
python3 -m battery_monitor --test-alerts   # fire every channel at every level
```

`--once` exits 0 when OK, 2 when a threshold is tripped, 1 with no pack
connected, so it drops into a preflight script.

Run `--test-alerts` before you rely on this. A channel that is enabled but
broken is worse than one that is off, and that is what it checks.

## Why it is not a bare comparison

Pack voltage sags hard under throttle. A naive `if volts < land` fires on
every punch-out, and an alert that cries wolf is an alert you learn to ignore
— which is the same failure as having no alert at all. So:

- **EMA smoothing** (`sampling.smoothing`) damps transients.
- **Confirmation** (`sampling.confirm_samples`) requires a level to hold for N
  consecutive samples before it activates.
- **Hysteresis** (`sampling.hysteresis`) requires extra margin to recover
  upward, so a pack hovering on a boundary does not chatter.
- **Latching** remembers the worst level reached for the session. A pack that
  touched CRITICAL under load was stressed; the load going away does not undo
  that.
- **Presence detection** keeps it quiet with no pack attached, and clears the
  filter on unplug so reconnecting cannot fire a phantom CRITICAL.

Optionally, `sampling.sag_compensate` reconstructs open-circuit voltage as
`V_bus + I × internal_resistance`. It ships off: a wrong `internal_resistance`
is worse than no compensation. Measure yours as
`(V_noload − V_loaded) / I_loaded` before turning it on.

Alerts also carry a **time-to-next-threshold** estimate from a least-squares
slope over `sampling.trend_window`, and a state-of-charge estimate from the
per-cell curve for the configured chemistry. Both are labelled estimates — the
SoC curve is only meaningful at low current.

## Notification channels

Configured under `notifications` in the YAML. Each has a `min_level`, and each
is isolated: a broken channel logs once and never stops the others or the
monitor loop.

- **desktop** — GNOME popup. `land` and worse use critical urgency, which stays
  on screen until dismissed. Under systemd there is no session bus in the
  environment, so the address is reconstructed from `/run/user/<uid>/bus`.
  Alerts go out over D-Bus with a `replaces_id`, so a repeating alert updates
  one popup in place instead of stacking dozens of undismissable ones during a
  sustained low-voltage event. `notify-send` is the fallback if PyGObject is
  missing (0.7.9 predates `--replace-id`, so the fallback does stack).
- **wall** — broadcast to every terminal and SSH session. This is the one that
  reaches you headless in the field.

There is no audio channel: this Jetson has no usable sound output (the only
playback card is the Tegra HDMI transmitter, and the attached display carries
no audio), so a beep channel could only ever report healthy while being
inaudible. There is no push channel by choice. Alerts are visual, plus the
journal.

journald is always on regardless: the monitor writes every event to stdout and
systemd captures it, so the log path exists even with all channels disabled.

Severity also drives how hard it nags — `alerts.repeat_seconds` re-alerts every
120 s at WARN down to every 10 s at MIN.

The **startup notice** is deferred, not fired immediately: this service is
running seconds after boot, before the desktop session exists, so sending it
straight away lost the popup on every cold start. It is held until every
enabled channel is reachable, or until `alerts.start_notice_timeout` (120 s)
elapses — whichever comes first, so a machine that never logs in still gets it
on the channels that do work. **Threshold alerts are never delayed by this**;
only the informational startup notice is.

Delivery runs on a **worker thread**, because a channel can block for a long
time: both shell out or make a blocking D-Bus call, with timeouts up to 10 s,
and a wedged session bus can hit them. Inline dispatch would stall sampling,
and worst at the most severe levels where alerts repeat fastest — the monitor
would go half-blind exactly when readings matter most. The queue is bounded at
16; a wedged channel drops alerts with a log line rather than growing memory
without limit. On shutdown the queue is flushed first, so an alert raised in
the same instant as a SIGTERM still gets out.

## Footprint

~30 MB RSS, ~0.3% CPU at the default 2 s sampling interval.

The run script deliberately does **not** source the workspace venv, unlike the
other services here. The venv ships `jetson_cudss_bootstrap.pth`, which
preloads the cuDSS CUDA runtime at interpreter startup — 138 MB RSS versus
19 MB, for a process whose whole job is reading four I²C registers, on a box
that also has to hold YOLO, torch and cuVSLAM. The monitor needs only `smbus2`
and `PyYAML`, both in system site-packages on the ARK JetPack image; the venv
is sourced only if those imports are genuinely missing.

## Files

| Path | |
|---|---|
| `ina238.py` | Sensor driver, cloned from ARK-OS `platform/jetson/scripts/ina238_test.py` |
| `config.py` | Defaults, YAML merge, validation |
| `monitor.py` | Sampling, threshold state machine, trend, CSV log |
| `notify.py` | Notification channels |
| `test_monitor.py` | State-machine tests against a fake sensor |
| `../config/battery_monitor.yaml` | **The file you edit** |
| `../services/battery_monitor.service.in` | systemd unit template |
| `../services/run_battery_monitor.sh` | What the unit executes |

### Changes from the ARK-OS original

`ina238.py` keeps the register map and conversion factors verbatim, and adds:

- **Sub-zero die temperature fix.** The original read `DIETEMP` as signed, then
  `>> 4`; a negative value sign-extends, so the `& 0x800` two's-complement
  branch never fires and below-freezing temperatures decoded as large
  positives. Now read unsigned and sign-corrected explicitly.
- `VBUS` read unsigned (bit 15 is reserved, and it is not a signed quantity).
- `probe()` checks the manufacturer ID, so a wrong bus or address is caught
  rather than read as a plausible voltage.
- `INA238Error` on I2C failure, distinguishing a dead bus from a bad reading,
  and a context manager for the bus handle.
- `from_shunt_rating()` for the `I_max = sqrt(P/R)` calculation the original
  did inline in `__main__`.

## Tests

```bash
python3 -m battery_monitor.test_monitor    # no pytest required
python3 -m pytest battery_monitor/test_monitor.py
```

48 tests covering threshold crossing, sag rejection, hysteresis, latching,
repeat cadence, pack presence and reconnect, trend estimation, cell-count
plausibility, both SoC curves, config validation, async dispatch and queue
bounding, deferred startup notice, and desktop-session recovery — all against a fake sensor, since the
alerting logic is exactly what cannot be exercised by draining a real pack.
