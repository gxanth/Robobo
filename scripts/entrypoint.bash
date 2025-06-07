#!/usr/bin/env bash
# exit on error
set -e
# Base ROS
source /opt/ros/noetic/setup.bash

# Your current workspace
cd /workspace/catkin_ws
    catkin_make
source /workspace/catkin_ws/devel/setup.bash
# source /root/catkin_ws/setup.bash

# ── Optional: Print confirmation ──────────────────────────
echo "[entrypoint] ✅ ROS and catkin workspace sourced"
echo "[entrypoint] You are now in the container shell."
# rosrun learning_machines learning_robobo_controller.py "$@"
exec "$@"