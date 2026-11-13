#!/bin/bash
# Container command: brings up the whole head.
# With `restart: unless-stopped` in docker-compose.yaml this runs on every boot,
# so the head no longer has to be started by hand.
# entrypoint.sh has already sourced ROS and the workspace before this runs.

# The robot's router boots slower than the Pi. If the nodes start before WiFi is
# up, CycloneDDS binds to docker0 and the body cannot see the head's topics.
echo "[start_head] Waiting for network..."
for i in $(seq 1 150); do
    if awk '$2 == "00000000" { found = 1 } END { exit !found }' /proc/net/route; then
        echo "[start_head] Network is up."
        break
    fi
    sleep 2
done

# The face is drawn on the Pi's own screen, which only exists after desktop login.
echo "[start_head] Waiting for the display..."
for i in $(seq 1 60); do
    if [ -S /tmp/.X11-unix/X0 ]; then
        echo "[start_head] Display is up."
        sleep 2
        break
    fi
    sleep 2
done

if [ ! -f /home/docker_user/ros2_ws_head/install/setup.bash ]; then
    echo "[start_head] Workspace is not built. Run colcon build in ~/ros2_ws_head first."
    exit 1
fi

exec ros2 launch head_bringup head_bringup_launch.py
