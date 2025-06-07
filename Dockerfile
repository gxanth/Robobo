# 1. [HOST] Build context: This Dockerfile is built from your host machine (e.g., VS Code or docker build)
FROM ros:noetic

# 2. [CONTAINER] Expose ports for Robobo/ROS communication
EXPOSE 45100
EXPOSE 45101

# 3. [CONTAINER] Remove conflicting ROS apt sources and keys to avoid repo issues
RUN rm /etc/apt/sources.list.d/ros1-latest.list \
    && rm /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# 4. [CONTAINER] Install base system dependencies (curl, ca-certificates, etc.)
RUN apt-get update \
    && apt-get install -y ca-certificates curl

# 5. [CONTAINER] Download and install latest ROS apt source for up-to-date ROS packages
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb
    
# 6. [CONTAINER] Install all required system packages (Python, OpenCV, dos2unix, etc.)
RUN apt-get update && apt-get install -y \
    python3 python3-pip git \
    ffmpeg libsm6 libxext6 \
    ros-noetic-opencv-apps \
    dos2unix && \
    ln -sf /usr/bin/python3 /usr/bin/python && \
    rm -rf /var/lib/apt/lists/*



# 7. [HOST→CONTAINER] Copy Python requirements and install all Python dependencies
COPY ./requirements.txt /requirements.txt
RUN pip3 install --no-cache-dir -r /requirements.txt && rm /requirements.txt

# 8. [HOST→CONTAINER] Set the working directory to /workspace/catkin_ws
WORKDIR /workspace/catkin_ws
COPY ./catkin_ws .

# 9. [HOST→CONTAINER] Copy all helper scripts (entrypoint, source_env, etc.)
COPY ./scripts /workspace/scripts/

# 10. Make all your scripts and nodes executable
RUN \
  # top-level scripts
  chmod +x /workspace/scripts/*.bash /workspace/scripts/*.sh && \
  # ROS package Python nodes
  chmod +x /workspace/catkin_ws/src/learning_machines/scripts/*.py

# 11. Normalize line endings & build workspace
RUN find . -type f \( -name '*.py' -o -name '*.bash' -o -name '*.sh' \) -exec dos2unix '{}' \; \
 && bash -c "source /opt/ros/noetic/setup.bash && catkin_make"



# 12. Overwrite /root/.bashrc so every new shell is ready
RUN echo "source /opt/ros/noetic/setup.bash"                    > /root/.bashrc && \
    echo "source /workspace/catkin_ws/devel/setup.bash"        >> /root/.bashrc && \
    echo 'echo "[bashrc] ✅ Sourced ROS and workspace setup files"' >> /root/.bashrc && \
    echo 'echo -e "[bashrc] ✅ Use the command \"./scripts/start_coppelia_sim.sh ./scenes/Robobo_Scene.ttt\" in your \033[1mHOST\033[0m terminal to start the Coppelia simulation"' >> /root/.bashrc &&\
    echo 'echo "[bashrc] ✅ Use the command \"rosrun learning_machines learning_robobo_controller.py --simulation\" to observe Robobo actions"' >> /root/.bashrc

# 13. [CONTAINER] (Optional) Set the entrypoint to your custom script for production use
# ENTRYPOINT ["/workspace/scripts/entrypoint.bash"]
