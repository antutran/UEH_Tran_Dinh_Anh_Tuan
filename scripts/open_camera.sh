#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
[ -f /ws_build/install/setup.bash ] && source /ws_build/install/setup.bash
exec ros2 run rqt_image_view rqt_image_view /camera/lane_debug
