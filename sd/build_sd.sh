#!/bin/bash
# source ros
source $PWD/sd/ros_source.bash

# build packages
declare -a INTERFACES=(sd_interfaces)
declare -a PLUGINS=(sd_actor_plugin sd_drone_plugin sd_world_plugin sd_gazebo_ros_plugins)
declare -a PACKAGES=(sd_simulation sd_flight_controller sd_actor_controller)

colcon build --symlink-install --packages-select ${PACKAGES[@]} ${PLUGINS[@]} ${INTERFACES[@]}