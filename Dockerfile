FROM ros:iron-ros-base

ARG NETRC

SHELL ["bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-serial \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root

RUN mkdir -p ~/ros2_ws/src \
    && cd ~/ros2_ws/src \ 
    && git clone -b iron_docker https://github.com/lolddohaja/bumper_collision_detector.git \ 
    && cd ~/ros2_ws \ 
    && . /opt/ros/$ROS_DISTRO/setup.sh \
    && colcon build

RUN sed -i '$isource "/root/ros2_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]