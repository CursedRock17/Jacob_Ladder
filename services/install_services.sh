#!/bin/bash
# Generate and install the systemd units for this workspace.
#
# systemd cannot expand variables in ExecStart/User, so the units are kept as
# *.service.in templates with @JL_WS_ROOT@ / @JL_USER@ / @JL_GROUP@ placeholders
# and rendered here against the machine actually running the script. No path in
# git ever names a specific user.
#
# Besides the units this also installs the pieces they depend on: the OAK USB
# power-management udev rule and the /var/log/ros2 directory that
# aruco_tracker.service points ROS_LOG_DIR at.
#
# Usage:
#   ./services/install_services.sh                      # all units
#   ./services/install_services.sh dds_agent            # just these units
#   JL_SERVICE_USER=pilot ./services/install_services.sh
#   ./services/install_services.sh --dry-run            # print, install nothing
#   ./services/install_services.sh --enable             # also enable + start them
#
# Installing a unit does not start it. Either pass --enable, or afterwards:
#   sudo systemctl enable --now dds_agent translation_node

set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

SERVICES_DIR="$JL_WS_ROOT/services"
UNIT_DIR="${JL_SYSTEMD_DIR:-/etc/systemd/system}"

# Run the services as the user who owns the workspace, not as whoever typed
# sudo (SUDO_USER) and not as root.
JL_SERVICE_USER="${JL_SERVICE_USER:-$(stat -c '%U' "$JL_WS_ROOT")}"
JL_SERVICE_GROUP="${JL_SERVICE_GROUP:-$(id -gn "$JL_SERVICE_USER")}"

dry_run=0
do_enable=0
units=()
for arg in "$@"; do
    case "$arg" in
        --dry-run) dry_run=1 ;;
        --enable) do_enable=1 ;;
        -h|--help) sed -n '2,26p' "$0"; exit 0 ;;
        *) units+=("${arg%.service}") ;;
    esac
done

if [ ${#units[@]} -eq 0 ]; then
    for template in "$SERVICES_DIR"/*.service.in; do
        unit="$(basename "$template")"
        units+=("${unit%.service.in}")
    done
fi

echo "Workspace : $JL_WS_ROOT"
echo "Run as    : $JL_SERVICE_USER:$JL_SERVICE_GROUP"
echo "Unit dir  : $UNIT_DIR"
echo

for unit in "${units[@]}"; do
    template="$SERVICES_DIR/$unit.service.in"
    if [ ! -f "$template" ]; then
        echo "install_services: no template $template" >&2
        exit 1
    fi

    rendered="$(sed \
        -e "s|@JL_WS_ROOT@|$JL_WS_ROOT|g" \
        -e "s|@JL_USER@|$JL_SERVICE_USER|g" \
        -e "s|@JL_GROUP@|$JL_SERVICE_GROUP|g" \
        "$template")"

    if [ "$dry_run" -eq 1 ]; then
        echo "----- $unit.service"
        echo "$rendered"
        echo
        continue
    fi

    echo "$rendered" | sudo tee "$UNIT_DIR/$unit.service" >/dev/null
    echo "installed $UNIT_DIR/$unit.service"
done

# --- Conflict check: ARK's own DDS agent ------------------------------------
#
# The ARK Electronics Jetson image ships its own Micro-XRCE-DDS agent as a
# *user*-level unit, `dds-agent.service` (hyphen), enabled by default, running
#   MicroXRCEAgent serial -b 3000000 -D /dev/ttyTHS1
# via ~/.local/bin/start_dds_agent.sh and the eProsima snap.
#
# Ours is `dds_agent.service` (underscore) at the *system* level. systemd sees
# two unrelated units, so nothing stops them both starting -- and they open the
# same serial device at different baud rates. Whichever loses the race either
# fails to open the port or garbles the link. Only one may run.
#
# The check reads the filesystem rather than asking systemctl. This script is
# normally run under sudo, and `systemctl --user` then talks to *root's* user
# manager (or dies with "Failed to connect to bus"), so it would never see the
# service user's units -- the guard would silently pass exactly when it matters.
ark_unit="dds-agent.service"
ark_home="$(getent passwd "$JL_SERVICE_USER" | cut -d: -f6)"
ark_conflict=0
for unit in "${units[@]}"; do
    [ "$unit" = "dds_agent" ] || continue
    # enabled == a wants/ symlink on disk; running == a live agent process
    # Enabled == a wants/ symlink on disk.
    if [ -e "$ark_home/.config/systemd/user/default.target.wants/$ark_unit" ]; then
        ark_conflict=1
    fi
    # Running == an agent process whose cgroup is ARK's unit. Matching on the
    # process name alone is not enough: our own dds_agent.service runs a
    # MicroXRCEAgent too, and would trigger a false conflict against itself.
    for pid in $(pgrep -x 'MicroXRCEAgent' 2>/dev/null); do
        if grep -qs "/$ark_unit\$\|/$ark_unit/" "/proc/$pid/cgroup"; then
            ark_conflict=1
        fi
    done
done

if [ "$ark_conflict" -eq 1 ]; then
    echo "WARNING: ARK's $ark_unit (user-level) is enabled/active and drives the" >&2
    echo "         same serial device as our dds_agent.service, at a different baud" >&2
    echo "         rate (ARK 3000000 vs \$XRCE_BAUD default 921600). Run only one:" >&2
    echo "" >&2
    echo "         Keep ARK's  : do not enable dds_agent" >&2
    echo "         Keep ours   : systemctl --user disable --now $ark_unit" >&2
    echo "" >&2
    echo "         Whichever you keep, its baud must match the flight controller's" >&2
    echo "         SER_TEL2_BAUD. See DRONE_SETUP.md." >&2
    echo "" >&2
    if [ "$do_enable" -eq 1 ]; then
        echo "         Refusing to --enable dds_agent while ARK's agent is active." >&2
        exit 1
    fi
fi

# --- Supporting bits the units depend on -----------------------------------

UDEV_RULE="$SERVICES_DIR/99-oak-usb-power.rules"
UDEV_DIR="${JL_UDEV_DIR:-/etc/udev/rules.d}"
# aruco_tracker.service sets ROS_LOG_DIR here; rclcpp will not create it, and
# the unit runs as a non-root user that cannot mkdir under /var/log itself.
ROS_LOG_DIR="${JL_ROS_LOG_DIR:-/var/log/ros2}"

if [ "$dry_run" -eq 1 ]; then
    echo "----- would install $UDEV_DIR/$(basename "$UDEV_RULE")"
    echo "----- would create $ROS_LOG_DIR (owned by $JL_SERVICE_USER:$JL_SERVICE_GROUP)"
    [ "$do_enable" -eq 1 ] && echo "----- would enable + start: ${units[*]}"
    exit 0
fi

if [ -f "$UDEV_RULE" ]; then
    sudo install -m 0644 "$UDEV_RULE" "$UDEV_DIR/"
    sudo udevadm control --reload-rules
    echo "installed $UDEV_DIR/$(basename "$UDEV_RULE") (replug the OAK, or reboot, to apply)"
fi

sudo install -d -o "$JL_SERVICE_USER" -g "$JL_SERVICE_GROUP" -m 0755 "$ROS_LOG_DIR"
echo "created $ROS_LOG_DIR"

sudo systemctl daemon-reload

if [ "$do_enable" -eq 1 ]; then
    echo
    sudo systemctl enable --now "${units[@]}"
    echo
    for unit in "${units[@]}"; do
        printf '%-20s %s\n' "$unit" "$(systemctl is-active "$unit" 2>&1)"
    done
else
    echo
    echo "Next: sudo systemctl enable --now ${units[*]}"
fi
