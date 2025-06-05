# Instructions for Programming with Robobo & Stable Baselines

## 1. Project Overview
This project involves programming a Robobo robot (both in simulation via CoppeliaSim and hardware). It leverages **Stable Baselines** for reinforcement learning to control the robot.

The main tasks involve interacting with the robot's hardware (IR sensors, camera, motors), running simulations, and applying reinforcement learning algorithms to control the robot's behavior.

### Key Areas:
- **Simulation with CoppeliaSim**
- **Real robot interaction via ROS**
- **Reinforcement Learning using Stable Baselines**
  
## 2. File Structure
You will primarily work with the following files and directories:

├── catkin_ws
│ ├── src
│ │ ├── learning_machines
│ │ │ ├── scripts
│ │ │ │ └── learning_robobo_controller.py
│ │ │ ├── src
│ │ │ │ └── learning_machines
│ │ │ │ ├── init.py
│ │ │ │ └── test_actions.py
│ │ │ ├── CMakeLists.txt
│ │ │ ├── package.xml
│ │ │ └── setup.py
│ │ ├── robobo_interface
│ │ │ ├── src
│ │ │ │ └── robobo_interface
│ │ │ │ ├── utils
│ │ │ │ ├── init.py
│ │ │ │ ├── base.py
│ │ │ │ ├── datatypes.py
│ │ │ │ ├── hardware.py
│ │ │ │ └── simulation.py
│ │ └── data_files
│ ├── .catkin_workspace
├── models
├── scenes
├── scripts
│ ├── entrypoint.bash
│ ├── run.sh
│ ├── setup.bash
│ ├── start_coppelia_sim.sh
│ └── start_coppelia_sim.zsh
└── Dockerfile
└── requirements.txt

perl
Copy
Edit

### Primary Files to Focus On:
- **learning_machines/scripts/learning_robobo_controller.py**: This file will contain your code for controlling the robot, running RL algorithms, and interacting with the simulation or real robot.
- **robobo_interface**: This package handles communication between the software and the robot (both hardware and simulation). Focus on:
  - **hardware.py** for real robot interaction.
  - **simulation.py** for simulating the robot in CoppeliaSim.
- **test_actions.py**: Use this for testing your control actions before moving to full integration with the robot.
- **Dockerfile and requirements.txt**: Ensure the environment is correctly set up. Docker is already built, so focus on the programming aspects now.

## 3. Setup for Programming

1. **Simulation Setup**:
   - Ensure **CoppeliaSim** is installed and placed under `./CoppeliaSim` or `./coppeliaSim.app`.
   - Update **setup.bash** with the correct IP for the simulation:
     ```bash
     COPPELIA_SIM_IP="your_computer_ip"
     ```

2. **Run CoppeliaSim**:
   - **Linux**:
     ```bash
     bash ./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt
     ```
   - **macOS**:
     ```zsh
     zsh ./scripts/start_coppelia_sim.zsh ./scenes/Robobo_Scene.ttt
     ```

3. **Run the Example with Simulation**:
   - **Linux/macOS**:
     ```bash
     bash ./scripts/run.sh --simulation
     ```

4. **Running with Hardware (If applicable)**:
   - If you’re using the actual hardware, make sure the robot and computer are on the same network, and update the `ROS_MASTER_URI` in `setup.bash`.
   - **Linux/macOS**:
     ```bash
     bash ./scripts/run.sh --hardware
     ```

## 4. Key Files for Development

### **learning_robobo_controller.py**:
This is where your RL agent’s code lives. The task is to:
- Control the robot using **Stable Baselines** reinforcement learning algorithms (e.g., PPO, DQN).
- Utilize the robot’s sensors and cameras for decision-making.
- Ensure that the robot can perform tasks autonomously in both simulation and hardware.

### **Interaction with the Robot**:
- **hardware.py** (Robobo Interface): This file defines the functions for interacting with the robot’s motors, sensors, and camera. Common functions include:
  - `move()`
  - `read_orientation()`
  - `get_image()`
  - Use these functions in `learning_robobo_controller.py` for your RL agent to control the robot.

- **simulation.py** (Robobo Interface): This is used for controlling the robot in CoppeliaSim (simulated environment). Functions are similar to `hardware.py` but may include simulation-specific methods such as `start_simulation()` and `stop_simulation`.

### **Stable Baselines**:
- Use Stable Baselines for your RL algorithms. Ensure that your `learning_robobo_controller.py` imports the appropriate models:
  ```python
  from stable_baselines3 import PPO  # Example: PPO for policy gradient
  from robobo_interface.simulation import SimulationRobobo
