FROM osrf/ros:noetic-desktop-full

ENV ROS_DISTRO=noetic
ENV CATKIN_WS=/root/catkin_ws
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    git \
    python3-rosdep \
    python3-catkin-tools \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-rviz \
    ros-noetic-teleop-twist-keyboard \
    libcanberra-gtk-module \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep if not already initialized
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

WORKDIR /root
RUN mkdir -p $CATKIN_WS/src

# Clone unitree
WORKDIR $CATKIN_WS/src
RUN git clone --recurse-submodules https://github.com/unitreerobotics/unitree_ros.git

# Install dependencies
WORKDIR $CATKIN_WS
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# Build
RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"

# Automatically source ROS
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc
RUN echo "source $CATKIN_WS/devel/setup.bash" >> /root/.bashrc

WORKDIR /media

# entrypoint
CMD ["/bin/bash"]

