#!/bin/bash
source $PWD/sd/ros_source.bash
# build packages
declare -a PACKAGES=(sd_simulation sd_interfaces sd_gazebo_ros_plugins sd_flight_controller)
colcon build --symlink-install --packages-select ${PACKAGES[@]}