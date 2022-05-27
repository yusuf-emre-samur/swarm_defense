for d in $PWD/swarm_defense_gazebo/models/* ; do
    if [ -d "$d" ]; then
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${d}
    fi
done
source /usr/share/gazebo/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$PWD/swarm_defense_gazebo/worlds