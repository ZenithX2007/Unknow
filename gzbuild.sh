#!/bin/bash

# Update the workspace path variable
WORKSPACE=~/gen0_gz_sim_ros2

cd "$WORKSPACE"


mkdir -p "$WORKSPACE/src/gen0_gz_sim_ros2/gen0_gz_sim_ros2/gz_plugins/build"
cd "$WORKSPACE/src/gen0_gz_sim_ros2/gen0_gz_sim_ros2/gz_plugins/build" || exit
cmake ..
make
cd ..
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$(pwd)/build

cd "$WORKSPACE" || exit
colcon build
source install/setup.bash

ros2 launch gen0_main spawn.launch.py world:=san_full actors_scenario:=walking_actors3 ground_truth_localization:=true

# rviz:=true
