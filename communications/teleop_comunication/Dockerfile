FROM osrf/ros:humble-desktop-full


SHELL ["/bin/bash", "-c"]


ENV ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive


RUN apt-get update && apt-get install -y \
    curl \
    lsb-release \
    gnupg \
    python3-colcon-common-extensions \
    git \
    python3-rosdep \
    build-essential \
    && rm -rf /var/lib/apt/lists/*


RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# Install ROS2 Gazebo packages and other dependencies needed for Unitree Go2 CHAMP config
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros2-control \
    ros-humble-xacro \
    ros-humble-robot-localization \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-velodyne \
    ros-humble-velodyne-gazebo-plugins \
    ros-humble-velodyne-description \
    && rm -rf /var/lib/apt/lists/*

# Create a ROS2 workspace and clone the Unitree Go2 CHAMP config repository
WORKDIR /ros2_ws/src
RUN git clone -b humble https://github.com/anujjain-dev/unitree-go2-ros2.git


WORKDIR /ros2_ws

# Install dependencies via rosdep
RUN apt-get update && apt-get install -y python3-rosdep
RUN rosdep update 
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace using colcon, ensuring the ROS2 environment is sourced.
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# Automatically source ROS2 and the workspace environment on container startup
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Expose the WebSocket relay port (if you plan to run your teleop server inside the container)
EXPOSE 9090

# Set the default command to bash so you can interact with the container
CMD ["bash"]
