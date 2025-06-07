#!/usr/bin/env bash
set -e

# ── Source base ROS ───────────────────────────────────────────────────────
source /opt/ros/noetic/setup.bash

# ── Check if devel/setup.bash exists ─────────────────────────────────────
if [ ! -f /workspace/catkin_ws/devel/setup.bash ]; then
    echo "[entrypoint] 🛠️ devel/setup.bash not found — running catkin_make"
    cd /workspace/catkin_ws
    catkin_make
else
    echo "[entrypoint] ✅ Workspace already built — skipping catkin_make"
fi

# ── Source the built workspace ───────────────────────────────────────────
source /workspace/catkin_ws/devel/setup.bash

# ── Optional: Set Python path ────────────────────────────────────────────
export PYTHONPATH=/workspace/catkin_ws/src/robobo_interface/src:\
/workspace/catkin_ws/src/learning_machines/src:\
/workspace/catkin_ws/src:${PYTHONPATH}

echo "[entrypoint] 🧠 Environment ready"
exec "$@"
