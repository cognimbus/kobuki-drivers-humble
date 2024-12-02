#!/bin/bash
# Source ROS Humble setup
source /opt/ros/humble/setup.bash
# Source the Kobuki workspace setup

source ./install/setup.bash
exec "$@"