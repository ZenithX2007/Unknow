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
  "ros2 launch gen0_main spawn.launch.py"
  "ros2 launch gen0_main gen0_fast_lio_mapping.launch.py"
  "ros2 launch gen0_main gen0_mapping_drive.launch.py"
  "ign gazebo .*gen0_main.*/worlds"
  "pointcloud_accumulator_preview"
  "gen0_simulated_world_lidar"
  "gazebo_livox_adapter"
  "fastlio_mapping"
  "gen0_mapping_drive"
  "cmdvel_to_vehicle"
  "vehicle_movement_interface"
  "rviz2 .*gen0_3d_mapping"
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
