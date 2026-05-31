#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch emrmf_core emrmf_sim.launch.py "$@"
