for d in $PWD/swarm_defense_models/* ; do
    if [ -d "$d" ]; then
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${d}
    fi
done
