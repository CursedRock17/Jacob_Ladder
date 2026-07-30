# Drone Companion Computer Setup

Steps to configure a fresh Jetson (Ubuntu 22.04, ROS 2 Humble) for autonomous boot.
Performed on `usmsm-jetson` 2026-07-06. Replace `usmsm` with the drone's username
where it appears (`whoami` to check).

Prerequisites: ROS 2 Humble installed, `Jacob_Ladder` workspace cloned to
`~/Jacob_Ladder` and built (`install/setup.bash` exists), `MicroXRCEAgent`
installed (`which MicroXRCEAgent`).

## 1. WiFi hotspot on boot

A NetworkManager hotspot connection must already exist (created via GUI or
`nmcli device wifi hotspot ...`). Then enable autoconnect:

```bash
nmcli connection show                       # find the hotspot's name (here: "Hotspot")
nmcli connection modify Hotspot connection.autoconnect yes connection.autoconnect-priority 100
```

## 2. Desktop autologin (GDM)

Edit `/etc/gdm3/custom.conf`, in the `[daemon]` section set:

```ini
AutomaticLoginEnable=true
AutomaticLogin=<the drone's login user>
```

One-liner (only works if the stock commented lines are present) — uses the
current user, so run it while logged in as the drone account:

```bash
sudo sed -i "s/^#  AutomaticLoginEnable = true/AutomaticLoginEnable=true/; s/^#  AutomaticLogin = user1/AutomaticLogin=$USER/" /etc/gdm3/custom.conf
```

## 3. Permanent /dev/ttyUSB0 access (no sudo)

`chmod 666 /dev/ttyUSB0` does NOT persist — udev recreates the node on every
boot/replug. Do both of these instead:

```bash
# a) join the serial-device group (takes effect after logout/reboot)
sudo usermod -aG dialout $USER

# b) udev rule making all ttyUSB* devices world-read/writable
echo 'KERNEL=="ttyUSB[0-9]*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ttyusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

## 4. XRCE-DDS agent + translation node as boot services

Unit templates and launch scripts live in `Jacob_Ladder/services/`. The units run
as the user who owns the workspace and point at the scripts in the repo (single
source of truth — edit in the repo, then re-run the installer below).

No username or path is baked into git. The `run_*.sh` scripts resolve the
workspace from their own location (via `jl_env.sh`), and the `.service.in`
templates carry `@JL_WS_ROOT@` / `@JL_USER@` placeholders that
`install_services.sh` fills in for the machine it runs on. So on a new drone
there is nothing to `sed` — just install and enable:

```bash
cd ~/Jacob_Ladder
./services/install_services.sh --dry-run   # inspect the generated units
./services/install_services.sh             # render + install all of them
sudo systemctl enable --now dds_agent.service translation_node.service
```

Pass unit names to install a subset (`./services/install_services.sh dds_agent
translation_node`), or set `JL_SERVICE_USER` / `JL_SERVICE_GROUP` to run them as
somebody other than the workspace owner. `--enable` installs *and* enables and
starts in one go.

Alongside the units, the installer also puts in place the two things they depend
on: the `99-oak-usb-power.rules` udev rule, and `/var/log/ros2` (which
`aruco_tracker.service` sets `ROS_LOG_DIR` to — rclcpp will not create it, and
the unit runs as a non-root user that cannot create it under `/var/log`).

### ⚠ Conflict: the ARK image ships its own XRCE-DDS agent

**The ARK Electronics Jetson image already runs an XRCE-DDS agent, and it will
fight ours over the serial port.** Theirs is a *user*-level unit:

| | ARK's | Ours |
|---|---|---|
| unit | `dds-agent.service` (hyphen), user-level | `dds_agent.service` (underscore), system-level |
| enabled by default | yes | no |
| launcher | `~/.local/bin/start_dds_agent.sh` | `services/run_dds_agent.sh` |
| binary | eProsima snap, via `/usr/local/bin/MicroXRCEAgent` | `install/microxrcedds_agent/bin/MicroXRCEAgent` |
| command | `MicroXRCEAgent serial -b 3000000 -D /dev/ttyTHS1` | `MicroXRCEAgent serial --dev $XRCE_DEV -b $XRCE_BAUD` (default `/dev/ttyTHS1` @ 921600) |

Because the names differ and one is user-level while the other is system-level,
systemd sees two unrelated units and will happily run both. They then open the
same `/dev/ttyTHS1` at **different baud rates**; whichever loses the race either
fails to open the port or garbles the link. **Run exactly one.**

```bash
# check what is running right now
systemctl --user status dds-agent.service
ps aux | grep MicroXRCEAgent

# option A - keep ARK's: just never enable ours
# option B - keep ours:
systemctl --user disable --now dds-agent.service
sudo systemctl enable --now dds_agent.service
```

`install_services.sh` detects this: it warns if ARK's agent is enabled or active,
and refuses `--enable dds_agent` outright rather than creating the race.

Whichever you keep, **its baud must match the flight controller's
`SER_TEL2_BAUD`.** ARK's launcher hardcodes 3000000; ours defaults to 921600 and
is overridable via `XRCE_BAUD`. A mismatch looks exactly like dead wiring.

Notes:
- `dds_agent.service` runs
  `MicroXRCEAgent serial --dev "${XRCE_DEV:-/dev/ttyTHS1}" -b "${XRCE_BAUD:-921600}"`.
  The default device is the **GPIO UART** `/dev/ttyTHS1`, not `/dev/ttyUSB0` —
  set `XRCE_DEV=/dev/ttyUSB0` via a `systemctl edit dds_agent` drop-in to go back
  to the USB-TTL adapter. No sudo needed thanks to step 3. If the flight
  controller isn't plugged in, the unit restarts every 5 s until it appears.
- Camera / aruco / precision-land / cuVSLAM are still launched manually via
  `launch_scripts/super_real.sh` (tmux). Their service files
  (`usb_cam.service.in`, `aruco_tracker.service.in`) exist in `services/` but
  were not enabled in this setup.
- `usb_cam.service` runs the OAK-D visual-odometry publisher, not a USB camera
  driver — the unit name is historical. Until 2026-07-30 `run_usb_cam.sh` named
  `vo_publisher_px4_node`, which has never existed, so the unit failed with
  `No executable found` and restarted every 5 s; it now runs
  `cuvslam_publisher_px4_node`. Cross-check against
  `ros2 pkg executables oak_d_visual_odometry` if it ever misbehaves again.
- `super_real.sh` no longer launches the agent or translation node itself —
  its first two tmux windows just tail the systemd service logs
  (`journalctl -u ... -f`), so there's no duplicate-process risk.

## Alternative FC link: Jetson GPIO UART (`/dev/ttyTHS1`) instead of USB-TTL

Removes the USB-TTL adapter — the OrangeCube+ `TELEM2` and the Jetson 40-pin
header are both 3.3 V TTL UARTs, so they wire together directly. No logic-level
converter, no `ttyUSB` node.

### Wiring (OrangeCube+ `TELEM2` JST-GH → Jetson Orin Nano 40-pin header)

Cross TX↔RX. **Do not connect the FC 5 V pin** — GND + TX + RX only (3 wires,
same as the USB adapter; XRCE-DDS serial does not use RTS/CTS flow control).

| OrangeCube+ TELEM2 | wire     | Jetson 40-pin        |
| :----------------- | :------- | :------------------- |
| pin 2 — TX (out)   | ───────▶ | pin 10 — UART1_RXD   |
| pin 3 — RX (in)    | ◀─────── | pin 8  — UART1_TXD   |
| pin 6 — GND        | ─────    | pin 6  — GND         |
| pin 1 — 5 V        | ✗ leave disconnected |          |

Jetson pins 8/10 are `UART1` (`serial@3100000` → `/dev/ttyTHS1`).

### Jetson side is ready out of the box (verified 2026-07-09)

Unlike `ttyUSB` (steps 3), `ttyTHS1` needs **no** udev rule or getty tweak:

- `/dev/ttyTHS1` exists, is `root:dialout`, and opens at 921600
  (`stty -F /dev/ttyTHS1 921600` → OK). Your user is already in `dialout`.
- Nothing squats on it: the serial console is on `ttyTCU0`
  (`cat /proc/cmdline` → `console=ttyTCU0`), and `nvgetty` targets `ttyTHS0`.

### PX4 side — no change needed

Already correct in `config/params/v1_16_0_oakd_vio.params`: `UXRCE_DDS_CFG=102`
(TELEM2), `SER_TEL2_BAUD=921600`, `MAV_1_CONFIG=0`/`MAV_2_CONFIG=0`. The FC still
speaks XRCE-DDS on TELEM2 — only the Jetson end of the cable moves.

### Verify the pins electrically (loopback — do before trusting a flight)

Unplug the FC from pins 8/10 and jumper **pin 8 ↔ pin 10** directly, then:

```bash
stty -F /dev/ttyTHS1 921600 raw -echo
timeout 2 cat /dev/ttyTHS1 | xxd | head &
printf 'PINGTEST' > /dev/ttyTHS1
wait
```

Seeing `PINGTEST` back proves pins 8/10 are muxed to UART1 and wired correctly.
Nothing back → run `sudo /opt/nvidia/jetson-io/jetson-io.py` and confirm the
40-pin header UART1 is enabled. Remove the jumper and reconnect the FC after.

With the FC wired and PX4 booted, you should see PX4's DDS bytes streaming in:

```bash
timeout 3 cat /dev/ttyTHS1 | xxd | head    # non-zero bytes = link is live
```

### Point the agent at `ttyTHS1`

The service device is set by `XRCE_DEV` (default `/dev/ttyTHS1`). Quick manual run:

```bash
XRCE_DEV=/dev/ttyTHS1 bash ~/Jacob_Ladder/services/run_dds_agent.sh
```

Make it permanent for the boot service with a drop-in (survives repo edits):

```bash
sudo systemctl edit dds_agent.service
# add these two lines, save:
#   [Service]
#   Environment=XRCE_DEV=/dev/ttyTHS1
sudo systemctl restart dds_agent.service
journalctl -u dds_agent -f          # expect "session established"
```

Roll back to the USB-TTL adapter by removing the drop-in
(`sudo systemctl revert dds_agent.service`) and rewiring.

### Troubleshooting: agent stuck at `running...`, no `session established`

Symptom seen 2026-07-09: over GPIO the agent logged only `running... fd:3` and
never `create_client`/`session established`, while the USB-TTL adapter worked.

Diagnosis (reproduce by passively reading the port at the matched baud):

```bash
python3 - <<'PY'
import serial,time
s=serial.Serial('/dev/ttyTHS1',921600,timeout=0.1); s.reset_input_buffer()
t0=time.time()
while time.time()-t0<6: 
    b=s.read(64)
    if b: print(round(time.time()-t0,2), b.hex(' '))
s.close()
PY
```

If you see a **byte-identical ~23-byte frame starting `7e` arriving exactly once
per second**, that is the PX4 uXRCE-DDS client stuck "Running, disconnected",
re-requesting a session every 1 s because it never hears the agent's reply. That
means the link is **one-way**: RX (FC→Jetson pin 10) is good (you're receiving
clean frames), but **TX (Jetson pin 8 → FC TELEM2 RX) is not getting through**,
so the handshake never completes and no topics appear.

**The prime suspect is Jetson pin 8 itself, not the wire.** If the same three
known-good leads worked on the USB-TTL adapter, the wire is fine — the endpoints
differ. The adapter (FTDI/CP210x) is a dedicated UART chip whose TX pin is an
always-on push-pull driver. Jetson pin 8 is a shared SoC pad that only drives
UART TX if its **pinmux** says so; otherwise it's a high-impedance input that
looks like an open circuit to the FC's RX. Critically, **RX working does not
prove TX works** — pin 8 (TX) and pin 10 (RX) are separate pads with separate
pinmux registers, so RX can be muxed to UART while TX is left as a GPIO.

Localise it, most-likely first:

1. **Is pin 8 actually transmitting? (multimeter, no rewire)** Measure pin 8 →
   pin 6 (GND). An active UART TX **idles HIGH (~3.3 V)**. ~0 V or floating →
   pin 8 is not a UART transmitter → **pinmux** (go to 3).
2. **Loopback (definitive):** power down, unplug the FC from pins 8 & 10, jumper
   **pin 8 ↔ pin 10**, run `python3 flight_logs/nv_gpio_uart_test.py`.
   - Passes → pin 8 transmits cleanly → fault is the **header socket contact**
     for pin 8 (reseat the JST-GH / DuPont), not the pinmux.
   - Garbled data → TX baud/electrical issue.
   - "No data received" → **pinmux** (go to 3).
3. **Pinmux** (only if loopback returns nothing): re-run
   `sudo /opt/nvidia/jetson-io/jetson-io.py`, enable header **UART1** (muxes both
   8 and 10), save, **reboot**. Gotcha: this board boots a custom Arducam DTB
   (`/boot/extlinux/extlinux.conf` → `FDT /boot/arducam/dts/.../...-super.dtb`);
   jetson-io may patch the *default* DTB, not the booted one, so confirm the
   change landed in the DTB you actually boot.

Pinmux was checked on 2026-07-09 and is **symmetric**: `UART1_TX_PR2` (pin 110)
and `UART1_RX_PR3` (pin 111) show identical state (`MUX UNCLAIMED / GPIO
UNCLAIMED`, no GPIO consumer) — the boot pinmux configures the pair together and
RX works, so pin 8 is almost certainly muxed as TX too. That makes a **physical
pin-8 connection** the top suspect, not software. Read it yourself with:
`sudo cat /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins | grep -i uart1`.

So verify the crossover **carefully at pin 8** (the one endpoint that changed
vs the USB adapter): Jetson pin 8 (TX)→TELEM2 pin 3 (RX); pin 10→TELEM2 pin 2
(TX); pin 6→TELEM2 pin 6 (GND); FC 5 V unconnected. Confirm the TX lead is on
pin 8 (not an adjacent hole) and fully seated, and buzz it through to TELEM2
pin 3 — RX working only proves pin 10 is correct, not pin 8.

Note: a clean 1 Hz RX proves the common ground is fine (a bad ground corrupts
RX). The `useful_commands.txt` snippet is not a valid check here — it uses
115200 (vs the FC's 921600) and isn't physically jumpered.

## 5. Reboot and verify

### Current boot configuration (as of 2026-07-30)

| Unit | Boot | Why |
|---|---|---|
| `dds_agent.service` | **enabled** | the XRCE-DDS link to the FC, `/dev/ttyTHS1` @ 921600 |
| `translation_node.service` | **enabled** | PX4 message translation |
| `aruco_tracker.service` | disabled | grabs a camera; launched via `super_real.sh` instead |
| `usb_cam.service` | disabled | grabs the OAK-D; would fight `super_real.sh` |
| `dds-agent.service` (ARK's, user-level) | **disabled** | conflicts — see the warning in section 4 |

```bash
sudo reboot
# after boot:
nmcli connection show --active            # Hotspot active
systemctl status dds_agent translation_node
journalctl -u dds_agent -u translation_node -f
ls -l /dev/ttyTHS1                        # crw-rw---- root:dialout
groups | grep dialout

# exactly one agent, and it must be the system one:
pgrep -ax MicroXRCEAgent
systemctl --user is-enabled dds-agent.service   # expect: disabled
```

### Confirming the FC link is actually up

**`ros2 topic list` showing `/fmu/*` topics does NOT mean the link works.**
`translation_node` registers those topic names itself the moment it starts, so
they appear even with the flight controller unplugged. This is an easy way to
convince yourself a dead link is healthy.

Only two things prove a live link:

```bash
# 1. the agent logs a session
journalctl -u dds_agent.service | grep -i "session established"

# 2. data actually arrives from the FC
ros2 topic echo /fmu/out/vehicle_status --once
```

If the agent log shows only `running... | fd: 3` and the `echo` times out, the
serial link is down regardless of what `ros2 topic list` says — go to the
troubleshooting section below.
