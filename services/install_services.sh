#!/bin/bash
# Generate and install the systemd units for this workspace.
#
# systemd cannot expand variables in ExecStart/User, so the units are kept as
# *.service.in templates with @JL_WS_ROOT@ / @JL_USER@ / @JL_GROUP@ placeholders
# and rendered here against the machine actually running the script. No path in
# git ever names a specific user.
#
# Usage:
#   ./services/install_services.sh                      # all units
#   ./services/install_services.sh dds_agent            # just these units
#   JL_SERVICE_USER=pilot ./services/install_services.sh
#   ./services/install_services.sh --dry-run            # print, install nothing
#
# Then: sudo systemctl enable --now dds_agent translation_node

set -e
source "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../jl_env.sh"

SERVICES_DIR="$JL_WS_ROOT/services"
UNIT_DIR="${JL_SYSTEMD_DIR:-/etc/systemd/system}"

# Run the services as the user who owns the workspace, not as whoever typed
# sudo (SUDO_USER) and not as root.
JL_SERVICE_USER="${JL_SERVICE_USER:-$(stat -c '%U' "$JL_WS_ROOT")}"
JL_SERVICE_GROUP="${JL_SERVICE_GROUP:-$(id -gn "$JL_SERVICE_USER")}"

dry_run=0
units=()
for arg in "$@"; do
    case "$arg" in
        --dry-run) dry_run=1 ;;
        -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
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

if [ "$dry_run" -eq 0 ]; then
    sudo systemctl daemon-reload
    echo
    echo "Next: sudo systemctl enable --now ${units[*]}"
fi
