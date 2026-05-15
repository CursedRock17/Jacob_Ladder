#!/usr/bin/env bash
# =============================================================================
# 05_python_packages.sh
# Source: Dockerfile_base-jammy-arm64 / Dockerfile (main)
#
# Installs Python pip packages.
# Run as your NORMAL USER (not root) to install into the user environment,
# or adjust as needed for a venv.
#
# NOTE: ultralytics (YOLO) will pull in torch. On Jetson, it's recommended
#   to install PyTorch from NVIDIA's Jetson wheel index instead of PyPI
#   for CUDA support. See the note below before running this script.
# =============================================================================
set -euo pipefail

echo ">>> [05] Upgrading pip, wheel, setuptools..."
python3 -m pip install --upgrade pip wheel setuptools

echo ">>> [05] Installing PX4 dev Python dependencies..."
python3 -m pip install \
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

echo ">>> [05] Installing ultralytics (YOLO)..."
# -------------------------------------------------------------------------
# JETSON NOTE: If you want GPU-accelerated inference, install PyTorch first
# from NVIDIA's Jetson index BEFORE running this line:
#
#   pip install --no-cache-dir \
#     torch torchvision \
#     --index-url https://developer.download.nvidia.com/compute/redist/jp/v62
#
# Then run: python3 -m pip install --ignore-installed ultralytics
# -------------------------------------------------------------------------
python3 -m pip install --ignore-installed ultralytics

echo ">>> [05] Python packages installed."
