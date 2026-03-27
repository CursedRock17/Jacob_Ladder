#
# PX4 base development environment
#

# How to build:
# docker buildx build --platform linux/arm64 -t lucaswendland/jacob_ladder:latest .
# docker login -u lucaswendland
# Create some PAT on dockerhub (with Read/Write access)
# docker tag lucaswendland/jacob_ladder:latest lucaswendland/jacob_ladder:latest
# docker push lucaswendland/jacob_ladder:latest


# =============================================================================
# Step 1  - Base PX4 Image
# =============================================================================
FROM --platform=linux/arm64 ubuntu:22.04
LABEL maintainer="Lucas Wendland"

# Setup environment variables ahead of time
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    CCACHE_UMASK=000 \
    TERM=xterm \
    TZ=UTC \
    PATH="/usr/lib/ccache:$PATH" \
    QT_X11_NO_MITSHM=1 \
    JAVA_TOOL_OPTIONS=-Dfile.encoding=UTF8 \
    ROS_DISTRO=humble \
    GZ_VERSION=harmonic \
    OPENCV_VERSION=4.10.0

# 1. Base Installations
RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    ant binutils-dev ca-certificates ccache cmake cppcheck curl dirmngr doxygen \
    g++ gcc gdb gettext git gnupg gosu lcov libelf-dev libexpat-dev libvecmath-java \
    libfreetype6-dev libgmp-dev libgtest-dev libisl-dev libmpc-dev libmpfr-dev \
    libpng-dev libssl-dev lsb-release make ninja-build neovim openssh-client \
    openjdk-11-jre openjdk-11-jdk python3 python3-dev python3-pip python3-venv \
    rsync screen shellcheck tzdata texinfo u-boot-tools util-linux unzip valgrind \
    vim wget xsltproc zip locales \
    && apt-get -y autoremove && apt-get clean && rm -rf /var/lib/apt/lists/*

# Setup Locales properly for ROS
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ---------------------------------------------------------------------------
# 0. Environment & tools
# ---------------------------------------------------------------------------
# Set java 11 as default
RUN update-alternatives --set java $(update-alternatives --list java | grep "java-11")

# gtest
RUN cd /usr/src/gtest \
	&& mkdir build && cd build \
	&& cmake .. && make -j$(nproc) \
	&& find . -name \*.a -exec cp {} /usr/lib \; \
	&& cd .. && rm -rf build

# Install Python 3 pip build dependencies first.
RUN python3 -m pip install --upgrade pip wheel setuptools

# Python 3 dependencies installed by pip
RUN python3 -m pip install argparse argcomplete coverage cerberus empy==3.3.4 jinja2 kconfiglib \
    matplotlib>=3.0.* numpy nunavut>=1.1.0 packaging pkgconfig pyros-genmsg pyulog \
    pyyaml requests serial six toml psutil pyulog wheel jsonschema pynacl lxml

#RUN python3 -m pip install argcomplete argparse>=1.2 cerberus coverage empy>=3.3 future \
  #jinja2>=2.8 jsonschema kconfiglib lxml matplotlib>=3.0.* numpy>=1.13 nunavut>=1.1.0 \
  #packaging pandas>=0.21 pkgconfigpsutil pygments wheel>=0.31.1 pymavlink pyros-genmsg \
  #pyserial pyulog>=0.5.0 pyyaml requests setuptools>=39.2.0 six>=1.12.0 toml>=0.9 sympy>=1.10.1

# manual ccache setup
RUN ln -s /usr/bin/ccache /usr/lib/ccache/cc \
	&& ln -s /usr/bin/ccache /usr/lib/ccache/c++

# astyle v3.1
RUN wget -q https://downloads.sourceforge.net/project/astyle/astyle/astyle%203.1/astyle_3.1_linux.tar.gz -O /tmp/astyle.tar.gz \
	&& cd /tmp && tar zxf astyle.tar.gz && cd astyle/src \
	&& make -f ../build/gcc/Makefile -j$(nproc) && cp bin/astyle /usr/local/bin \
	&& rm -rf /tmp/*

# create user with id 1001 (jenkins docker workflow default)
RUN useradd --shell /bin/bash -u 1001 -c "" -m user && usermod -a -G dialout user

# setup virtual X server
RUN mkdir /tmp/.X11-unix && \
	chmod 1777 /tmp/.X11-unix && \
	chown -R root:root /tmp/.X11-unix
ENV DISPLAY :99

# SITL UDP PORTS
EXPOSE 14556/udp
EXPOSE 14557/udp

# =============================================================================
# Step 2  - Harmonic/Jammy
# =============================================================================
# Install JSBSim (Optional: Consider building from source for newer Ubuntu releases)
#RUN wget https://github.com/JSBSim-Team/jsbsim/releases/download/v1.1.1a/JSBSim-devel_1.1.1-134.focal.amd64.deb \
    #&& dpkg -i JSBSim-devel_1.1.1-134.focal.amd64.deb \
    #&& rm JSBSim-devel_1.1.1-134.focal.amd64.deb

# =============================================================================
# Step 3  - Humble ARM64
# =============================================================================

# ---------------------------------------------------------------------------
# 1.  Add the official ROS apt repository + install ROS 2 Humble
#     (follows the “ros-apt-source” mechanism you quoted)
# ---------------------------------------------------------------------------
# 3. Gazebo Harmonic & ROS 2 Humble
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN export ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F \"tag_name\" | awk -F'\"' '{print $4}')" && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop ros-humble-ros-gz gz-harmonic \
    python3-vcstool python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb

RUN echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /etc/bash.bashrc

# ---------------------------------------------------------------------------
# 5.  Install Foxglove support in container
# ---------------------------------------------------------------------------
RUN curl -fL -o /tmp/foxglove-agent.deb \
    "https://get.foxglove.dev/agent/v1.4.7/foxglove-agent_1.4.7-1_arm64.deb" \
    && dpkg -i /tmp/foxglove-agent.deb \
    && rm /tmp/foxglove-agent.deb \
    && apt-get update \
    && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*
# =============================================================================
# Step 4  - OpenCV
# =============================================================================

WORKDIR /opt
RUN wget -q -O opencv.zip https://github.com/opencv/opencv/archive/4.10.0.zip && unzip -q opencv.zip
WORKDIR /opt/opencv-4.10.0/build
RUN cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D WITH_OPENCL=OFF \
          -D BUILD_opencv_java=OFF \
          -D BUILD_EXAMPLES=OFF .. \
          -D BUILD_opencv_python3=ON \
    && make -j2 \
    && make install && ldconfig \
    && cd /opt && rm -rf opencv-4.10.0 opencv.zip
# ---------------------------------------------------------------------------
# 3. Cleanup build artifacts (optional but recommended)
# ---------------------------------------------------------------------------
RUN rm -rf /opt/opencv /opt/opencv_contrib

# ---------------------------------------------------------------------------
# 4. Test OpenCV install (optional)
# ---------------------------------------------------------------------------
RUN python3 -c "import cv2; print('OpenCV version:', cv2.__version__)" || echo "Python OpenCV binding not found"

COPY scripts/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["/bin/bash"]
