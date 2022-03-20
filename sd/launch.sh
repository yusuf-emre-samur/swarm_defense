#!/bin/bash
# vars
ROS_VERSION=galactic
# source ros
source /opt/ros/$ROS_VERSION/setup.bash
# source local if exists
FILE=$PWD/install/setup.bash
if test -f "$FILE"; then
    source install/setup.bash
fi
# launch
ros2 launch sd_simulation simulation.launch.py