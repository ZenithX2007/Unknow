#!/usr/bin/env bash
set -Eeuo pipefail

QUIET=false
if [[ "${1:-}" == "--quiet" ]]; then
  QUIET=true
fi

log() {
  if [[ "$QUIET" != "true" ]]; then
    printf '%s\n' "$*"
  fi
}

stop_pattern() {
  local pattern="$1"
  local label="$2"

  if pgrep -f "$pattern" >/dev/null 2>&1; then
    log "Stopping $label"
    pkill -TERM -f "$pattern" 2>/dev/null || true
  fi
}

kill_pattern() {
  local pattern="$1"
  local label="$2"

  if pgrep -f "$pattern" >/dev/null 2>&1; then
    log "Killing stubborn $label"
    pkill -KILL -f "$pattern" 2>/dev/null || true
  fi
}

patterns=(
  "ros2 launch epsilon_planning epsilon_planning.launch.py"
  "run_gen0_epsilon.sh"
  "epsilon_scene_bridge_node"
  "epsilon_integrated_planner_node"
  "epsilon_cmd_vel_mux_node"
  "qcnet_prediction_node"
)

for pattern in "${patterns[@]}"; do
  stop_pattern "$pattern" "$pattern"
done

sleep 2

for pattern in "${patterns[@]}"; do
  kill_pattern "$pattern" "$pattern"
done

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
  timeout 5s ros2 daemon stop >/dev/null 2>&1 || true
fi

log "Done."
