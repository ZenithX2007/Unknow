#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ -n "${CONDA_PREFIX:-}" ]]; then
  echo "Please run 'conda deactivate' before starting ROS 2."
  exit 1
fi

: "${GEN0_LLM_BASE_URL:=https://superaichao.xin/openai}"
export GEN0_LLM_BASE_URL

if [[ -z "${GEN0_LLM_API_KEY:-}" ]]; then
  read -r -s -p "Intermediate API key: " GEN0_LLM_API_KEY
  echo
  export GEN0_LLM_API_KEY
fi

if [[ -z "${GEN0_LLM_MODEL:-}" ]]; then
  read -r -p "Model ID from the intermediate service: " GEN0_LLM_MODEL
  export GEN0_LLM_MODEL
fi

set +u
source /opt/ros/humble/setup.bash
source "${project_root}/install/setup.bash"
set -u
exec ros2 launch gen0_llm_agent llm_agent.launch.py provider:=openai_compatible
