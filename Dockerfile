ARG ARCH=""
ARG ROS_DISTRO=humble
FROM ${ARCH}ros:${ROS_DISTRO} AS base

SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO=humble
ARG USERNAME=user
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      bluetooth \
      bluez \
      curl \
      gdb \
      git \
      libgl1 \
      libbluetooth-dev \
      libglfw3 \
      libosmesa6 \
      python3-pip \
      rsync \
      sudo \
      udev \
      vim \
      ros-${ROS_DISTRO}-actuator-msgs \
      ros-${ROS_DISTRO}-desktop \
      ros-${ROS_DISTRO}-joint-state-publisher \
      ros-${ROS_DISTRO}-joint-state-publisher-gui \
      ros-${ROS_DISTRO}-joy \
      ros-${ROS_DISTRO}-plotjuggler-ros \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-teleop-twist-joy \
      ros-${ROS_DISTRO}-xacro \
      ros-dev-tools && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir \
      ds4drv \
      ipympl \
      jupytext \
      mujoco \
      notebook \
      rockit-meco

RUN echo 'KERNEL=="uinput", GROUP="input", MODE="0666"' > /etc/udev/rules.d/99-uinput.rules

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} --create-home --shell /bin/bash ${USERNAME} && \
    usermod -aG dialout,input ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN useradd -m ds4user && \
    usermod -aG input,sudo ds4user

RUN echo 'PS1="(container) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "[ -f /home/antsy/install/setup.bash ] && source /home/antsy/install/setup.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /home/antsy

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER ${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
