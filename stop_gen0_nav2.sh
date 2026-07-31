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
  "ros2 launch gen0_main gen0_navigation.launch.py"
  "run_gen0_nav2.sh"
  "map_server"
  "controller_server"
  "smoother_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "velocity_smoother"
  "lifecycle_manager_navigation"
  "cmdvel_to_vehicle"
  "nav2_pose_guard"
  "identity_map_to_odom"
  "nav2_projected_map_relay"
  "nav2_costmap_2d"
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
  ros2 daemon stop >/dev/null 2>&1 || true
fi

log "Done."
