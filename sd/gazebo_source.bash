for d in $PWD/swarm_defense_models/* ; do
    if [ -d "$d" ]; then
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${d}
    fi
done
worlds=$PWD/worlds
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:${worlds}