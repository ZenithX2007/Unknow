#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GZ_PLUGIN_DIR="$WORKSPACE/gen0_gz_sim_ros2/gz_plugins"
GZ_PLUGIN_BUILD_DIR="$GZ_PLUGIN_DIR/build"
WORLD="${GEN0_WORLD:-my_map}"
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO:-}"
GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}"

cd "$WORKSPACE"

cmake -S "$GZ_PLUGIN_DIR" -B "$GZ_PLUGIN_BUILD_DIR"
cmake --build "$GZ_PLUGIN_BUILD_DIR" -j"$(nproc)"

set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install --packages-up-to gen0_main fast_lio --packages-ignore race_plan_control

set +u
source "$WORKSPACE/install/setup.bash"
set -u

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GPU_ADAPTER"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$GZ_PLUGIN_BUILD_DIR"

gazebo_launch=(
  ros2 launch gen0_main spawn.launch.py
  world:="$WORLD"
  ground_truth_localization:=true
  render_env:=unset
)
if [[ -n "$GPU_ADAPTER" ]]; then
  gazebo_launch+=(d3d12_adapter:="$GPU_ADAPTER")
fi
if [[ -n "$ACTORS_SCENARIO" ]]; then
  gazebo_launch+=(actors_scenario:="$ACTORS_SCENARIO")
fi

"${gazebo_launch[@]}"
