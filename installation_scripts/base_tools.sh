#!/usr/bin/env bash
# =============================================================================
# 01_base_tools.sh
# Source: Dockerfile_jacobladder / Dockerfile_base-jammy-arm64 (Step 1)
#
# Installs base PX4 development tools.
# JetPack 6.2 already provides cmake, gcc/g++, git, python3, python3-pip —
# apt will skip those if already installed.
# =============================================================================
set -euo pipefail

export DEBIAN_FRONTEND=noninteractive

echo ">>> [01] Installing base build tools..."

apt-get update && apt-get -y --quiet --no-install-recommends install \
    ant \
    binutils-dev \
    ca-certificates \
    ccache \
    cmake \
    cppcheck \
    curl \
    dirmngr \
    doxygen \
    g++ \
    gcc \
    gdb \
    gettext \
    git \
    gnupg \
    gosu \
    lcov \
    libelf-dev \
    libexpat-dev \
    libvecmath-java \
    libfreetype6-dev \
    libgmp-dev \
    libgtest-dev \
    libisl-dev \
    libmpc-dev \
    libmpfr-dev \
    libpng-dev \
    libssl-dev \
    lsb-release \
    make \
    neovim \
    ninja-build \
    openssh-client \
    openjdk-11-jre \
    openjdk-11-jdk \
    python3 \
    python3-dev \
    python3-pip \
    python3-venv \
    rsync \
    screen \
    shellcheck \
    texinfo \
    tzdata \
    u-boot-tools \
    unzip \
    util-linux \
    valgrind \
    vim \
    wget \
    xsltproc \
    zip \
    && apt-get -y autoremove \
    && apt-get clean autoclean \
    && rm -rf /var/lib/apt/lists/*

# Set Java 11 as default
update-alternatives --set java "$(update-alternatives --list java | grep "java-11")"

# Build and install gtest libraries
cd /usr/src/gtest \
    && mkdir -p build && cd build \
    && cmake .. && make -j"$(nproc)" \
    && find . -name "*.a" -exec cp {} /usr/lib \; \
    && cd .. && rm -rf build

# Manual ccache symlinks
ln -sf /usr/bin/ccache /usr/lib/ccache/cc
ln -sf /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz \
    -O /tmp/astyle.tar.gz \
    && cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
    && make -f ../build/gcc/Makefile -j"$(nproc)" && cp bin/astyle /usr/local/bin \
    && rm -rf /tmp/astyle* /tmp/astyle.tar.gz

echo ">>> [01] Base tools installed."
