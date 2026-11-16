#!/bin/bash
# Block until the system clock has been set, then exit 0.
#
# The Jetson has no battery-backed clock: after a cold boot it starts at
# 1970-01-01 and chrony later steps it forward ~56 years, once it reaches an
# NTP source (the internet over ethernet, or the laptop when it joins the
# hotspot). If VIO is already running when that happens, the RealSense frame
# timestamps jump and cuVSLAM rejects every frame from then on
# ("Dropping non-monotonic stereo frame timestamp"), so PX4 gets no position.
#
# vio.service runs this as ExecStartPre, so VIO starts only once the clock is
# real. A warm reboot keeps the RTC, so the clock is already valid and this
# returns at once. With no NTP source at all, VIO waits (and says so in the
# journal) until one appears, e.g. until the laptop joins the hotspot.

MIN_YEAR="${JL_MIN_CLOCK_YEAR:-2025}"

waited=0
while [ "$(date +%Y)" -lt "$MIN_YEAR" ]; do
    if [ $((waited % 30)) -eq 0 ]; then
        echo "wait_for_clock: system clock reads $(date --iso-8601=seconds);" \
             "waiting for NTP before starting (${waited}s so far)"
    fi
    sleep 1
    waited=$((waited + 1))
done

[ "$waited" -gt 0 ] && echo "wait_for_clock: clock set ($(date --iso-8601=seconds)) after ${waited}s"
exit 0
