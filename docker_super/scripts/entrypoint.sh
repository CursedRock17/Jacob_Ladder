#!/bin/bash

# Start virtual X server in the background
# - DISPLAY default is :99, set in dockerfile
# - Users can override with `-e DISPLAY=` in `docker run` command to avoid
#   running Xvfb and attach their screen
if [[ -x "$(command -v Xvfb)" && "$DISPLAY" == ":99" ]]; then
	echo "Starting Xvfb"
	Xvfb :99 -screen 0 1600x1200x24+32 &
fi

# Check if the ROS_DISTRO is passed and use it
# to source the ROS environment
if [ -n "${ROS_DISTRO}" ]; then
	source "/opt/ros/$ROS_DISTRO/setup.bash"
fi

# Use the LOCAL_USER_ID if passed in at runtime
if [ -n "${LOCAL_USER_ID}" ]; then
    echo "Starting with UID : $LOCAL_USER_ID"
    # Only modify if UID differs
    if [ "$(id -u user)" != "$LOCAL_USER_ID" ]; then
        usermod -u $LOCAL_USER_ID -o user 2>/dev/null
        groupmod -g $LOCAL_USER_ID -o user 2>/dev/null
        chown -R $LOCAL_USER_ID:$LOCAL_USER_ID /home/user
    fi
    exec gosu user "$@"
else
    exec "$@"
fi
