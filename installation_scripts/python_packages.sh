#!/usr/bin/env bash
# =============================================================================
# 05_python_packages.sh
# Source: Dockerfile_base-jammy-arm64 / Dockerfile (main)
#
# Creates the project uv virtual environment with access to system ROS/JetPack
# Python packages.
#
# Run as your NORMAL USER (not root).
#
# VENV DESIGN — why --system-site-packages:
#   A plain venv on Jetson severs access to:
#     - ROS 2 Python bindings  (/opt/ros/humble/lib/python3/dist-packages)
#     - JetPack CUDA Python libs (pycuda, jetson-stats, etc.)
#   --system-site-packages inherits those while still isolating pip packages
#   from the system Python, preventing dependency conflicts.
#
# PYTORCH / YOLO:
#   Do not add ultralytics to the default uv sync unless the torch source is
#   pinned for Jetson. The generic PyPI dependency path can install a mismatched
#   CUDA/PyTorch/OpenCV stack and conflict with ROS cv_bridge.
# =============================================================================
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${VENV_DIR:-${PROJECT_ROOT}/.venv}"

# ---------------------------------------------------------------------------
# 1. Create the venv
# ---------------------------------------------------------------------------
echo ">>> [05] Creating uv virtual environment at ${VENV_DIR}..."
cd "${PROJECT_ROOT}"
uv venv --python 3.10 --system-site-packages "${VENV_DIR}"

# Activate for the remainder of this script
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo ">>> [05] Syncing uv dependencies..."
uv sync

# ---------------------------------------------------------------------------
# 5. Write a shell snippet to auto-activate the venv in new terminals
# ---------------------------------------------------------------------------
ACTIVATE_SNIPPET="${HOME}/.jacob_ladder_venv_activate"
cat > "${ACTIVATE_SNIPPET}" <<SNIPPET
# Auto-activate Jacob Ladder uv venv (sourced from .bashrc)
if [ -f "${VENV_DIR}/bin/activate" ]; then
    source "${VENV_DIR}/bin/activate"
fi
SNIPPET

if ! grep -q "jacob_ladder_venv_activate" "${HOME}/.bashrc"; then
    echo "source ${ACTIVATE_SNIPPET}" >> "${HOME}/.bashrc"
    echo ">>> [05] Added venv auto-activate to ~/.bashrc"
fi

echo ""
echo ">>> [05] Done. Activate manually with:"
echo "         source ${VENV_DIR}/bin/activate"
echo "         Or open a new terminal (auto-activated via ~/.bashrc)"
