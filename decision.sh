#!/bin/bash

source install/setup.sh

ros2 launch rm_behavior_tree rm_behavior_tree.launch.py \
    style:=rmul \
    use_sim_time:=True