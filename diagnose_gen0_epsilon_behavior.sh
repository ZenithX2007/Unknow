#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

python3 "$WORKSPACE/tools/diagnose_epsilon_behavior.py" \
  --log-dir "$LOG_DIR" \
  "$@"
