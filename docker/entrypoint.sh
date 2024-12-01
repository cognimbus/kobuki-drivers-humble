#!/bin/sh
# Source ROS Humble setup
source /opt/ros/humble/setup.bash
# Source the Kobuki workspace setup

source ./kobuki-drivers-humble/install/setup.bash
exec "$@"