FROM ros:noetic

# Open ports used in robobo communication
EXPOSE 45100
EXPOSE 45101

# Clean up conflicting ROS keys if present
RUN rm -f /etc/apt/sources.list.d/ros1-latest.list \
    && rm -f /usr/share/keyrings/ros1-latest-archive-keyring.gpg

# Set up secure sources for ROS
RUN apt-get update \
    && apt-get install -y ca-certificates curl

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') ;\
    curl -L -s -o /tmp/ros-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update \
    && apt-get install /tmp/ros-apt-source.deb \
    && rm -f /tmp/ros-apt-source.deb

# Install ROS tutorials and base dependencies
RUN apt-get update \
    && apt-get install -y ros-noetic-roscpp-tutorials

# Install Python + Git
RUN apt-get update && apt-get install -y python3 python3-pip git && rm -rf /var/lib/apt/lists/*

# Install system dependencies needed for Python packages
RUN apt-get update && apt-get install -y \
    ffmpeg libsm6 libxext6 \
    ros-noetic-opencv-apps \
    dos2unix && rm -rf /var/lib/apt/lists/*

# Python dependencies
COPY ./requirements.txt /requirements.txt
RUN python3 -m pip install -r /requirements.txt && rm /requirements.txt

# Set working directory to catkin workspace
WORKDIR /root/catkin_ws

# Copy entire catkin workspace
COPY ./catkin_ws .

# Copy setup and entrypoint scripts to root
COPY ./scripts/setup.bash /setup.bash
COPY ./scripts/entrypoint.bash /entrypoint.bash

# Convert Windows line endings if any
RUN find . -type f \( -name '*.py' -o -name '*.bash' \) -exec dos2unix -l -- '{}' \; \
    && apt-get --purge remove -y dos2unix \
    && rm -rf /var/lib/apt/lists/*

# Build the catkin workspace
RUN bash -c 'source /opt/ros/noetic/setup.bash && catkin_make'

# Make everything executable just in case
RUN chmod -R u+x /root/catkin_ws/
RUN chmod +x /entrypoint.bash

# Optional: source ROS setup files in shell sessions
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc
RUN echo 'source /root/catkin_ws/setup.bash' >> /root/.bashrc

# Actual script to run when container starts
ENTRYPOINT ["./entrypoint.bash"]

