#!/usr/bin/env bash
# =============================================================================
# 05_python_packages.sh
# Source: Dockerfile_base-jammy-arm64 / Dockerfile (main)
#
# Creates a virtual environment and installs all Python packages,
# using NVIDIA's Jetson-native PyTorch wheel for CUDA support.
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
# PYTORCH — why not PyPI:
#   PyPI torch is built for x86 with generic CUDA. On Jetson (ARM64) you need
#   NVIDIA's wheel built against the onboard CUDA/cuDNN/TensorRT stack.
#   JetPack 6.2 targets CUDA 12.6; the wheel below matches that.
#   Verify the latest wheel at: https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
# =============================================================================
set -euo pipefail

VENV_DIR="${HOME}/drone_venv"

# ---------------------------------------------------------------------------
# 1. Create the venv
# ---------------------------------------------------------------------------
echo ">>> [05] Creating virtual environment at ${VENV_DIR}..."
python3 -m venv --system-site-packages "${VENV_DIR}"

# Activate for the remainder of this script
# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"

echo ">>> [05] Upgrading pip, wheel, setuptools inside venv..."
pip install --upgrade pip wheel setuptools

# ---------------------------------------------------------------------------
# 2. Install NVIDIA Jetson PyTorch wheel (CUDA-enabled)
#    JetPack 6.2 = CUDA 12.6, Python 3.10, ARM64
#    Check https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
#    for updated wheel URLs if this version is superseded.
# ---------------------------------------------------------------------------
echo ">>> [05] Installing PyTorch (Jetson CUDA wheel)..."
pip install --no-cache-dir \
    torch \
    torchvision \
    --index-url https://developer.download.nvidia.com/compute/redist/jp/v62/pytorch/

# Sanity check
python3 - <<'EOF'
import torch
print(f"PyTorch version : {torch.__version__}")
print(f"CUDA available  : {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"CUDA device     : {torch.cuda.get_device_name(0)}")
EOF

# ---------------------------------------------------------------------------
# 3. PX4 dev Python dependencies
# ---------------------------------------------------------------------------
echo ">>> [05] Installing PX4 dev Python dependencies..."
pip install \
    argparse \
    argcomplete \
    coverage \
    cerberus \
    "empy==3.3.4" \
    jinja2 \
    kconfiglib \
    "matplotlib>=3.0" \
    numpy \
    "nunavut>=1.1.0" \
    packaging \
    pkgconfig \
    pyros-genmsg \
    pyulog \
    pyyaml \
    requests \
    pyserial \
    six \
    toml \
    psutil \
    wheel \
    jsonschema \
    pynacl \
    lxml

# ---------------------------------------------------------------------------
# 4. ultralytics (YOLO) — installed AFTER torch so it picks up the CUDA wheel
# ---------------------------------------------------------------------------
echo ">>> [05] Installing ultralytics (YOLO)..."
pip install --ignore-installed ultralytics

# ---------------------------------------------------------------------------
# 5. Write a shell snippet to auto-activate the venv in new terminals
# ---------------------------------------------------------------------------
ACTIVATE_SNIPPET="${HOME}/.drone_venv_activate"
cat > "${ACTIVATE_SNIPPET}" <<SNIPPET
# Auto-activate drone venv (sourced from .bashrc)
if [ -f "${VENV_DIR}/bin/activate" ]; then
    source "${VENV_DIR}/bin/activate"
fi
SNIPPET

if ! grep -q "drone_venv_activate" "${HOME}/.bashrc"; then
    echo "source ${ACTIVATE_SNIPPET}" >> "${HOME}/.bashrc"
    echo ">>> [05] Added venv auto-activate to ~/.bashrc"
fi

echo ""
echo ">>> [05] Done. Activate manually with:"
echo "         source ${VENV_DIR}/bin/activate"
echo "         Or open a new terminal (auto-activated via ~/.bashrc)"
