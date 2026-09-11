#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Please run 'conda deactivate' before starting ROS 2."
  exit 1
fi

: "${GEN0_OLLAMA_BASE_URL:=http://127.0.0.1:11434}"
: "${GEN0_OLLAMA_MODEL:=qwen2.5:3b}"
export GEN0_OLLAMA_BASE_URL GEN0_OLLAMA_MODEL

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u
exec ros2 launch gen0_llm_agent llm_agent.launch.py provider:=ollama
