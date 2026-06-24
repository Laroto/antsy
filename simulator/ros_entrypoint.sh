#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

if [ -f /home/antsy/install/setup.bash ]; then
  source /home/antsy/install/setup.bash
fi

exec "$@"
