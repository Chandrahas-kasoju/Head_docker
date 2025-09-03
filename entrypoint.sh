#!/bin/bash
set -e

# setup ros environment
export ROS_DISTRO=jazzy
export ROS_DOMAIN_ID=0
source "/opt/ros/$ROS_DISTRO/setup.bash"
export TAPPAS_POST_PROC_DIR=$(pkg-config --variable=tappas_postproc_lib_dir hailo-tappas-core)
/usr/bin/supervisord
exec "$@"