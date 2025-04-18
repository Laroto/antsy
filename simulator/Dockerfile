FROM ros:humble

RUN apt update && apt install -y \
    lsb-release \
    gnupg \
    curl

RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get install -y \
    ignition-fortress

ENV IGNITION_VERSION=fortress

RUN apt-get install -y \
    ros-humble-ros-gz
RUN apt-get install -y \
    ros-humble-rmw-cyclonedds-cpp

WORKDIR /home/antsy

RUN echo 'PS1="(container) ${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ "' >> /home/${USERNAME}/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source install/setup.bash" >> /home/$USERNAME/.bashrc

COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
