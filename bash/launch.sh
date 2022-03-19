#!/bin/bash
export GAZEBO_MODEL_PATH=~/local/swarm_defense/models
source /opt/ros/galactic/setup.bash
source install/setup.bash
ros2 launch sd_simulation simulation.launch.py