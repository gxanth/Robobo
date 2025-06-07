#!/bin/bash
#!/usr/bin/env bash
set -e

# Source global ROS install
source /opt/ros/noetic/setup.bash

# Source your catkin workspace
if [ -f /workspace/catkin_ws/devel/setup.bash ]; then
  source /workspace/catkin_ws/devel/setup.bash
  export PYTHONPATH="/workspace/catkin_ws/src:${PYTHONPATH}"
  echo "[source_env] ✅ Workspace sourced"
else
  echo "[source_env] ⚠️ Workspace not built. Run catkin_make?"
fi
