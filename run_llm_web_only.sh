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

export GEN0_PROJECT_ROOT="${project_root}"
source "${project_root}/scripts/manage_llm_web_stack.sh"
GEN0_WEB_STACK_CHILD_PIDS=()
gen0_prepare_web_stack
trap gen0_cleanup_web_stack EXIT INT TERM

gen0_start_managed ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=127.0.0.1 port:="${rosbridge_port}"

gen0_start_managed /usr/bin/python3 -m http.server "${web_port}" \
  --directory "${project_root}/web_control"

gen0_start_managed ros2 run gen0_llm_agent llm_node --ros-args \
  -p provider:=ollama \
  -p planning_only:=true \
  -p use_sim_time:=false

echo "Lightweight LLM web mode is starting."
echo "Web:       http://localhost:${web_port}"
echo "rosbridge: ws://localhost:${rosbridge_port}"
echo "Model:     ${GEN0_OLLAMA_MODEL} (planning only)"
echo "Press Ctrl+C to stop all lightweight services."

wait -n "${GEN0_WEB_STACK_CHILD_PIDS[@]}"
