#!/usr/bin/env bash
# SafeCollab container entrypoint (AGENTS.md §8).
# Source the ROS 2 Jazzy underlay and the colcon overlay, then exec the CMD.
set -e

# ROS 2 base install (the underlay).
source /opt/ros/jazzy/setup.bash

# The colcon overlay built into the image. Guard its presence so the container
# still starts (e.g. for `bash`/introspection) before any package is built.
OVERLAY="${ROS_WS:-/opt/safecollab_ws}/install/setup.bash"
if [ -f "${OVERLAY}" ]; then
    source "${OVERLAY}"
fi

exec "$@"
