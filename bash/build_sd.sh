#!/bin/bash
source /opt/ros/galactic/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select sd_interfaces sd_simulation sd_flight_controller sd_gazebo_ros_plugins