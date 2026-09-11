#!/usr/bin/env bash
set -Eeuo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
web_port="${GEN0_WEB_PORT:-8000}"
rosbridge_port="${GEN0_ROSBRIDGE_PORT:-9090}"
: "${GEN0_OLLAMA_BASE_URL:=http://127.0.0.1:11434}"
: "${GEN0_OLLAMA_MODEL:=qwen2.5:3b}"
export GEN0_OLLAMA_BASE_URL GEN0_OLLAMA_MODEL

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Please run 'conda deactivate' before starting ROS 2."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u

for package_name in ros_gz_bridge gen0_main sweeper_integration rosbridge_server gen0_llm_agent; do
  if ! ros2 pkg prefix "${package_name}" >/dev/null 2>&1; then
    echo "Missing ROS package: ${package_name}" >&2
    exit 1
  fi
done

export GEN0_PROJECT_ROOT="${project_root}"
source "${project_root}/scripts/manage_llm_web_stack.sh"
GEN0_WEB_STACK_CHILD_PIDS=()
gen0_prepare_web_stack
trap gen0_cleanup_web_stack EXIT INT TERM

gen0_start_managed ros2 launch sweeper_integration web_control_manual.launch.py \
  world:=my_map \
  gazebo_gui:=false \
  render_engine:=ogre \
  render_env:=unset \
  rosbridge_port:="${rosbridge_port}" \
  web_port:="${web_port}"

gen0_start_managed ros2 run gen0_llm_agent llm_node --ros-args \
  -p provider:=ollama \
  -p planning_only:=true \
  -p use_sim_time:=false

echo "Headless driving-camera + LLM web mode is starting."
echo "No Gazebo GUI, RViz, SLAM, Nav2, EPSILON, or YOLO will be started."
echo "Wait about 12 seconds, then open http://localhost:${web_port}"
echo "rosbridge: ws://localhost:${rosbridge_port}"
echo "Camera:    /gen0_model/driver_camera/compressed"
echo "Model:     ${GEN0_OLLAMA_MODEL} (planning only)"
echo "Press Ctrl+C to stop the camera simulation and web services."

wait -n "${GEN0_WEB_STACK_CHILD_PIDS[@]}"
