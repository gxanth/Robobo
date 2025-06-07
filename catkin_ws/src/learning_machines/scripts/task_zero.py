#!/usr/bin/env python

import time
from robobo_interface import SimulationRobobo  # or HardwareRobobo
from robobo_interface import IRobobo


def go_straight_and_turn_on_ir(
    rob: IRobobo,
    ir_threshold: float = 100,
    forward_speed: int = 30,
    turn_speed: int = 30,
    step_time: int = 200
):
    """
    Move forward until an IR sensor detects an object, then turn right.
    """
    print("🚗 Starting hard-coded IR-based navigation...")
    while True:
        irs = rob.read_irs()
        print("🔍 IR sensor readings:", irs)

        # Check for obstacle detection
        if any(ir is not None and ir > ir_threshold for ir in irs):
            print("⚠️ Obstacle detected! Turning right.")
            rob.move_blocking(-turn_speed*10, -turn_speed*10, step_time*10)
            break
        else:
            rob.move_blocking(forward_speed, forward_speed, step_time)

    print("✅ IR-avoidance behavior completed.")


if __name__ == "__main__":
    # Connect to simulator
    robot = SimulationRobobo()
    robot.play_simulation()

    try:
        go_straight_and_turn_on_ir(robot)
    finally:
        # Always stop safely
        robot.stop_simulation()
        print("🛑 Simulation ended.")
