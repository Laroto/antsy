#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DOMAIN="${ANTSY_TEST_ROS_DOMAIN_ID:-93}"

cd "${ROOT_DIR}"

docker compose run --rm antsy bash -lc "
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  colcon build --packages-up-to antsy_description antsy_control hiwonder_ros2 ds4_launcher --symlink-install
  source install/setup.bash
  set -u
  export ROS_DOMAIN_ID=${ROS_DOMAIN}
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ANTSY_TEST_LOG_DIR=/tmp/antsy_smoke_tests
  python3 scripts/smoke_control_pipeline.py
  python3 scripts/smoke_hiwonder_writer.py
"
