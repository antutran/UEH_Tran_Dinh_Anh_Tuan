#!/usr/bin/env bash
# Auto-drive node inside the container
source /opt/ros/humble/setup.bash
[ -f /ws_build/install/setup.bash ] && source /ws_build/install/setup.bash
exec ros2 run crc_sim starter "$@"
