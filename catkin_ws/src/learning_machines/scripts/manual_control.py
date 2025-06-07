#!/usr/bin/env python

import time
from robobo_interface import SimulationRobobo  # or HardwareRobobo

# Connect to robot
robert = SimulationRobobo()
robert.play_simulation()

# Define simple action space
ACTIONS = {
    "forward": (10, 10),
    "backward": (-10, -10),
    "left": (-5, 5),
    "right": (5, -5),
    "stop": (0, 0),
}

# Choose action sequence
sequence = ["forward", "left", "forward", "right", "stop"]

print("Starting manual control...")

for action_name in sequence:
    print(f"→ Action: {action_name}")
    left_speed, right_speed = ACTIONS[action_name]
    robert.move_blocking(left_speed, right_speed, 1000)  # Move for 1 second

    # Read pose
    pos = robert.get_position()
    orient = robert.get_orientation()
    print(f"    Position: {pos}")
    print(f"    Orientation: {orient}")

# Stop robot at the end
robert.move_blocking(0, 0, 100)  # Stop briefly
print("✅ Manual control done.")
