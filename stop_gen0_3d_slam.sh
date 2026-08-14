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
  "ros2 launch gen0_main gen0_navigation.launch.py"
  "run_gen0_nav2.sh"
  "robot_state_publisher"
  "ign gazebo .*gen0_main.*/worlds"
  "ign gazebo server"
  "ign gazebo gui"
  "pointcloud_accumulator_preview"
  "gen0_scurm_terrain_analysis"
  "terrainAnalysis"
  "projected_terrain_map"
  "gen0_scurm_terrain_analysis_ext"
  "terrainAnalysisExt"
  "gen0_actor_obstacle_costmap"
  "actor_obstacle_costmap"
  "gen0_scurm_map_to_odom"
  "gen0_scurm_exchange_field"
  "exchangeField"
  "gen0_scurm_sensor_scan_generation"
  "sensorScanGeneration"
  "gen0_scurm_octomap_server"
  "octomap_server_node"
  "terrain_map_ext_preview"
  "terrain_map_preview"
  "cloud_registered_preview"
  "raw_front3d_preview"
  "fast_lio_map_preview"
  "stable_odom"
  "odom_registered_scan"
  "nav2_costmap_2d"
  "nav2_lifecycle_manager"
  "map_server"
  "controller_server"
  "smoother_server"
  "planner_server"
  "behavior_server"
  "bt_navigator"
  "waypoint_follower"
  "velocity_smoother"
  "ground_truth_publisher"
  "pose_publisher"
  "gen0_simulated_world_lidar"
  "gazebo_livox_adapter"
  "gen0_icp_relocalization"
  "gen0_icp_transform_publisher"
  "icp_node"
  "transform_publisher"
  "fastlio_mapping"
  "gen0_mapping_drive"
  "cmdvel_to_vehicle"
  "vehicle_movement_interface"
  "nav2_pose_guard"
  "identity_map_to_odom"
  "nav2_projected_map_relay"
  "ros_gz_bridge"
  "rviz2 .*gen0_3d_mapping"
  "rviz2 .*gen0_relocalization"
  "rviz2 .*gen0_nav2_default_view"
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
