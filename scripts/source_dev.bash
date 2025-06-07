#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /workspace/catkin_ws/devel/setup.bash
export PYTHONPATH="/workspace/catkin_ws/src:$PYTHONPATH"

echo "[source_env] ✅ Environment sourced."
echo "[source_env] 🧠 Ready to run ROS nodes or scripts."