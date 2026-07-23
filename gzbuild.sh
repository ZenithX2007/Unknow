#!/bin/bash

# Use the workspace that contains this script.
WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$WORKSPACE"


mkdir -p "$WORKSPACE/gen0_gz_sim_ros2/gz_plugins/build"
cd "$WORKSPACE/gen0_gz_sim_ros2/gz_plugins/build" || exit
cmake ..
make
cd ..
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$(pwd)/build

cd "$WORKSPACE" || exit
source /opt/ros/humble/setup.bash
colcon build --packages-ignore race_plan_control
source install/setup.bash

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

ros2 launch gen0_main spawn.launch.py world:=my_map ground_truth_localization:=true

# rviz:=true
