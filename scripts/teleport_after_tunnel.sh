#!/usr/bin/env bash
set -e

# Load ROS 2 environment
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [ -f /ws_build/install/setup.bash ]; then
    source /ws_build/install/setup.bash
elif [ -f /ws/install/setup.bash ]; then
    source /ws/install/setup.bash
fi

python3 /ws/scripts/teleport_after_tunnel.py "$@"
