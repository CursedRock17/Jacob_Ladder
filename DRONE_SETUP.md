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
AutomaticLogin=usmsm
```

One-liner (only works if the stock commented lines are present):

```bash
sudo sed -i 's/^#  AutomaticLoginEnable = true/AutomaticLoginEnable=true/; s/^#  AutomaticLogin = user1/AutomaticLogin=usmsm/' /etc/gdm3/custom.conf
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

Unit files and launch scripts live in `~/Jacob_Ladder/services/`. The units run
as the local user and point at the scripts in the repo (single source of truth
— edit in the repo, then re-copy the .service files and `daemon-reload`).

On a new drone, first fix the username/paths inside the four files if they
differ (check `User=`, `Group=` in the `.service` files and the
`cd /home/<user>/Jacob_Ladder` + `ExecStart=` paths):

```bash
cd ~/Jacob_Ladder/services
sed -i "s|/home/OLDUSER/|/home/$USER/|g" run_dds_agent.sh run_translation_node.sh dds_agent.service translation_node.service
sed -i "s/^User=.*/User=$USER/; s/^Group=.*/Group=$USER/" dds_agent.service translation_node.service
chmod +x run_dds_agent.sh run_translation_node.sh
```

Then install and enable:

```bash
sudo cp dds_agent.service translation_node.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable dds_agent.service translation_node.service
```

Notes:
- `dds_agent.service` runs `MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600`.
  No sudo needed thanks to step 3. If the flight controller isn't plugged in,
  the unit restarts every 5 s until it appears.
- Camera / aruco / precision-land / cuVSLAM are still launched manually via
  `launch_scripts/super_real.sh` (tmux). Their service files
  (`usb_cam.service`, `aruco_tracker.service`) exist in `services/` but were
  not enabled in this setup.
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

The service device is set by `XRCE_DEV` (default `/dev/ttyUSB0`). Quick manual run:

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

```bash
sudo reboot
# after boot:
nmcli connection show --active            # Hotspot active
systemctl status dds_agent translation_node
journalctl -u dds_agent -u translation_node -f
ls -l /dev/ttyUSB0                        # crw-rw-rw-
groups | grep dialout
```
