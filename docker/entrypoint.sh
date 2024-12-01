#!/bin/sh
# Source ROS Humble setup
source /opt/ros/humble/setup.bash
# Source the Kobuki workspace setup

source ./kobuki_ws/install/setup.bash
exec "$@"