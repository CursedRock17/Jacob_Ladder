# docker login -u lucaswendland
# Create some PAT on dockerhub (with Read/Write access)
# docker tag lucaswendland/jacob_ladder:latest lucaswendland/jacob_ladder:latest
# Build without using local cache
# docker build --no-cache -t lucaswendland/jacob_ladder:latest .
# Push the new version
# docker push lucaswendland/jacob_ladder:latest

FROM jacobsafeer/px4-dev-harmonic-jammy-humble-opencv:latest

ENV XDG_RUNTIME_DIR=/tmp/runtime

RUN apt-get update && apt-get -y --quiet --no-install-recommends install \
    xdg-utils \
    libnotify4 \
    #apt install ros-$ROS_DISTRO-foxglove-bridge \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean

RUN curl -fL -o /tmp/foxglove-studio.deb \
    "https://get.foxglove.dev/desktop/v2.48.0/foxglove-studio-2.48.0-linux-amd64.deb" \
    && dpkg -i /tmp/foxglove-studio.deb \
	  && rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

#RUN git clone https://github.com/foxglove/foxglove-sdk \
    #&& cd foxglove-sdk/ros \
    #&& source /opt/ros/humble/setup.bash \
    #&& make


# Create dir and set ownership to user 1000:1000
RUN mkdir -p ${XDG_RUNTIME_DIR} && chown 1000:1000 ${XDG_RUNTIME_DIR} && chmod 700 ${XDG_RUNTIME_DIR}

CMD ["/bin/bash"]

