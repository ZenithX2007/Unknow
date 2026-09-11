#!/usr/bin/env bash
set -Eeuo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
web_port="${GEN0_WEB_PORT:-8000}"
rosbridge_port="${GEN0_ROSBRIDGE_PORT:-9090}"
# The manually cleaned my_map grid puts the Gen0 spawn at its origin with the
# initial road heading along +x. Ground-truth odometry uses Gazebo world coords.
# These values map (-20.6991, -22.4324, -0.5406 rad) to (0, 0, 0 rad).
: "${GEN0_MAP_TO_ODOM_X:=6.2025628587}"
: "${GEN0_MAP_TO_ODOM_Y:=29.8863434457}"
: "${GEN0_MAP_TO_ODOM_YAW:=0.5406}"
: "${GEN0_OLLAMA_BASE_URL:=http://127.0.0.1:11434}"
: "${GEN0_OLLAMA_MODEL:=qwen2.5:3b}"
export GEN0_MAP_TO_ODOM_X GEN0_MAP_TO_ODOM_Y GEN0_MAP_TO_ODOM_YAW
export GEN0_OLLAMA_BASE_URL GEN0_OLLAMA_MODEL

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Please run 'conda deactivate' before starting ROS 2."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

# This mode uses Gazebo ground-truth odometry and a static map.  It deliberately
# avoids the online SLAM / laser conversion stack to keep the first web Nav2
# test light and independent of localization convergence.
for package_name in ros_gz_bridge gen0_main sweeper_integration rosbridge_server gen0_llm_agent nav2_bringup; do
  if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    echo "Missing ROS package: ${package_name}" >&2
    echo "Install the missing Humble package, then retry." >&2
    exit 1
  fi
done

export GEN0_PROJECT_ROOT="${project_root}"
source "${project_root}/scripts/manage_llm_web_stack.sh"
GEN0_WEB_STACK_CHILD_PIDS=()
gen0_prepare_web_stack
trap gen0_cleanup_web_stack EXIT INT TERM

# The bridged PoseArray discards link names and ordering; ground-truth odometry
# uses its configured continuous tracking mode instead of a fixed link index.
gen0_start_managed ros2 launch sweeper_integration web_control_manual.launch.py \
  world:=my_map \
  gazebo_gui:=false \
  render_engine:=ogre \
  render_env:=unset \
  rosbridge_port:="${rosbridge_port}" \
  web_port:="${web_port}" \
  ground_truth_odometry:=true

gen0_start_managed ros2 launch sweeper_integration navigation_ground_truth.launch.py \
  use_sim_time:=true \
  map:="${project_root}/gen0_gz_sim_ros2/gen0_main/maps/my_map_scurm_latest.yaml" \
  params_file:="${project_root}/gen0_gz_sim_ros2/sweeper_integration/config/nav2_ground_truth.yaml" \
  map_to_odom_x:="${GEN0_MAP_TO_ODOM_X}" \
  map_to_odom_y:="${GEN0_MAP_TO_ODOM_Y}" \
  map_to_odom_yaw:="${GEN0_MAP_TO_ODOM_YAW}"

gen0_start_managed ros2 run gen0_llm_agent llm_node --ros-args \
  -p provider:=ollama \
  -p planning_only:=false \
  -p use_sim_time:=false

echo "Headless web-navigation + camera + local LLM mode is starting."
echo "No Gazebo GUI, RViz, EPSILON, or YOLO will be started."
echo "Wait 20-40 seconds for /map and Nav2 lifecycle activation."
echo "Web:       http://localhost:${web_port}"
echo "rosbridge: ws://localhost:${rosbridge_port}"
echo "Nav2:      /navigate_to_pose"
echo "Model:     ${GEN0_OLLAMA_MODEL} (validated Nav2 commands enabled)"
echo "Press Ctrl+C to stop all navigation services."

wait -n "${GEN0_WEB_STACK_CHILD_PIDS[@]}"
