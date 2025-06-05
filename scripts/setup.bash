#!/usr/bin/env bash
# ── Network/IP settings ────────────────────────────────────────────────
export ROS_MASTER_URI="http://localhost:11311"
export COPPELIA_SIM_IP="192.168.0.100"        # ← your laptop’s IP

# ── ROS environment (adjust 'noetic' if you use another distro) ────────
source /opt/ros/noetic/setup.bash

# ── Your catkin workspace (needs catkin_make run at least once) ────────
source /workspace/catkin_ws/devel/setup.bash   # inside the container path

# ── Python search path: put inner src/ folders FIRST ───────────────────
export PYTHONPATH=/workspace/catkin_ws/src/robobo_interface/src:\
/workspace/catkin_ws/src/learning_machines/src:\
/workspace/catkin_ws/src:${PYTHONPATH}
