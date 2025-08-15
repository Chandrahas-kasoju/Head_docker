#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace setup, if it exists
if [ -f /home/rosuser/ros2_ws/install/setup.bash ]; then
  source /home/rosuser/ros2_ws/install/setup.bash
fi

exec "$@"