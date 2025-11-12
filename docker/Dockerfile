FROM ros:humble-ros-base

# Install basic requirements
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libboost-all-dev \
    libyaml-cpp-dev \
    ros-humble-rviz2 \
    software-properties-common \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-foxglove-bridge \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-realsense2-camera \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    python3-vcstool \
    git \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install specific numpy version
RUN pip install 'numpy<2.0.0'

# Create workspace
WORKDIR /ros2_ws

# Copy dependency files
COPY dependencies.repos /tmp/dependencies.repos

# Clone third-party repositories
RUN mkdir -p src && \
    cd src && \
    vcs import < /tmp/dependencies.repos && \
    git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git

# Install dependencies
RUN apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install

# RUN apt-get update && apt-get install -y ros-humble-robot-self-filter

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set default command
CMD ["bash"]