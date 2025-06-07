cmake_minimum_required(VERSION 2.8.3)
project(learning_machines)

# Specify rospy as dependency
find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()
catkin_package()

# ─────────────────────────────────────────────────────────────
# Automatically install every .py under scripts/
# ─────────────────────────────────────────────────────────────
file(GLOB NODE_SCRIPTS
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.py"
)

catkin_install_python(
  PROGRAMS ${NODE_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)