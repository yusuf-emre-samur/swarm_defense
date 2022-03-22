#!/bin/bash
for d in $PWD/models/* ; do
    if [ -d "$d" ]; then
        export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${d}
    fi
done
