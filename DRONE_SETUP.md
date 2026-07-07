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
