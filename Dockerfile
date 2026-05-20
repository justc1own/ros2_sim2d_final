# Build environment for sim2d_hardware_interface
FROM rwthika/ros2:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    ros-jazzy-ros2-control \
    ros-jazzy-rclcpp-lifecycle \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m -s /bin/bash dev
USER dev
WORKDIR /home/dev/ws

# Source ROS automatically in interactive shells
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/dev/.bashrc

CMD ["bash"]
