ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-actuator-msgs \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-ros-gz \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-xacro && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/antsy

COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "antsy_simulation", "simulator.launch.xml"]
