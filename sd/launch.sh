#!/bin/bash
# source
source $PWD/sd/ros_source.bash
source $PWD/sd/gazebo_source.bash
# launch
ros2 launch sd_simulation simulation.launch.py world:=world_v2.world