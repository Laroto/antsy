#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f /home/antsy/install/setup.bash ]; then
  source /home/antsy/install/setup.bash
fi

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

if [ "${ANTSY_AUTOSTART:-0}" != "1" ]; then
  exec bash
fi

if [ ! -f /home/antsy/install/setup.bash ]; then
  echo "Workspace is not built yet. Run colcon build first." >&2
  exit 1
fi

declare -a pids=()

launch_bg() {
  echo "Starting: $*"
  "$@" &
  pids+=("$!")
}

if [ "${ANTSY_RUN_SIM:-0}" = "1" ]; then
  launch_bg ros2 launch antsy_simulation simulator.launch.xml \
    publish_description:=false \
    start_controller:=false
fi

if [ "${ANTSY_RUN_DESCRIPTION:-1}" = "1" ]; then
  launch_bg ros2 launch antsy_description description.launch.py
fi

if [ "${ANTSY_RUN_CONTROL:-1}" = "1" ]; then
  launch_bg ros2 launch antsy_control follow_velocity_rectangle.launch.xml
fi

if [ "${ANTSY_RUN_TELEOP:-1}" = "1" ]; then
  launch_bg ros2 launch ds4_launcher joy_teleop.launch.py
fi

if [ "${ANTSY_RUN_HARDWARE:-0}" = "1" ]; then
  launch_bg ros2 run hiwonder_ros2 write_only
fi

wait "${pids[@]}"
