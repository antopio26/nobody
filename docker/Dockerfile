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
    ffmpeg

# Install specific numpy version first
RUN pip install 'numpy<2.0.0'

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-pointcloud-to-laserscan \
        ros-humble-realsense2-camera

# Create workspace
WORKDIR /ros2_ws

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc