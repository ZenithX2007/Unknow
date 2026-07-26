#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="${GEN0_WORKSPACE:-$HOME/gen0_gz_sim_ros2}"
WORLD="${GEN0_WORLD:-my_map}"
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO:-}"
GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}"
DRIVE_SPEED="${GEN0_DRIVE_SPEED:-0.25}"
PREVIEW_MAX_POINTS="${GEN0_PREVIEW_MAX_POINTS:-40000}"
PREVIEW_VOXEL_SIZE="${GEN0_PREVIEW_VOXEL_SIZE:-0.65}"
PREVIEW_PERIOD="${GEN0_PREVIEW_PERIOD:-1.0}"
RVIZ_CONFIG="${GEN0_RVIZ_CONFIG:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_3d_mapping_preview.rviz}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
SIMULATED_LIDAR="${GEN0_SIMULATED_LIDAR:-true}"
WORLD_OBJ_PATH="${GEN0_WORLD_OBJ_PATH:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.obj}"

PIDS=()
NAMES=()

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

require_file() {
  if [[ ! -e "$1" ]]; then
    printf 'Missing required file: %s\n' "$1" >&2
    exit 1
  fi
}

check_existing_ros_nodes() {
  if [[ "${GEN0_ALLOW_EXISTING_NODES:-false}" == "true" ]]; then
    return
  fi

  local existing
  existing="$(
    ros2 node list 2>/dev/null \
      | grep -E '(^/gen0_simulated_world_lidar$|^/gen0_gazebo_livox_adapter$|^/gen0_fast_lio$|^/gen0_mapping_drive$|^/pointcloud_accumulator_preview$|^/rviz$|^/vehicle_movement_interface$)' \
      || true
  )"
  if [[ -n "$existing" ]]; then
    printf 'Existing Gen0/RViz ROS nodes are still running:\n%s\n\n' "$existing" >&2
    printf 'Run ./stop_gen0_3d_slam.sh first, or set GEN0_ALLOW_EXISTING_NODES=true to bypass this check.\n' >&2
    exit 1
  fi
}

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

start_process() {
  local name="$1"
  shift

  log "Starting $name"
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("$name")
  log "$name pid=${PIDS[-1]} log=$LOG_DIR/$name.log"
}

process_group_alive() {
  local pid="$1"
  kill -0 -- "-$pid" 2>/dev/null || kill -0 "$pid" 2>/dev/null
}

signal_process_group() {
  local signal="$1"
  local pid="$2"
  kill "-$signal" -- "-$pid" 2>/dev/null || kill "-$signal" "$pid" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if ((${#PIDS[@]} > 0)); then
    log "Stopping launched processes"
    for pid in "${PIDS[@]}"; do
      if process_group_alive "$pid"; then
        signal_process_group INT "$pid"
      fi
    done
    sleep 4
    for pid in "${PIDS[@]}"; do
      if process_group_alive "$pid"; then
        signal_process_group TERM "$pid"
      fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
      if process_group_alive "$pid"; then
        signal_process_group KILL "$pid"
      fi
    done
    wait "${PIDS[@]}" 2>/dev/null || true
  fi

  exit "$status"
}

trap cleanup EXIT INT TERM

cd "$WORKSPACE"
require_file /opt/ros/humble/setup.bash
require_file "$WORKSPACE/install/setup.bash"
require_file "$RVIZ_CONFIG"
if [[ "$SIMULATED_LIDAR" == "true" ]]; then
  require_file "$WORLD_OBJ_PATH"
fi

mkdir -p "$LOG_DIR"

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"
check_existing_ros_nodes

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GPU_ADAPTER"

log "Workspace: $WORKSPACE"
log "World: $WORLD, actors_scenario: $ACTORS_SCENARIO"
log "Simulated lidar: $SIMULATED_LIDAR, world_obj_path: $WORLD_OBJ_PATH"
log "GPU adapter: $MESA_D3D12_DEFAULT_ADAPTER_NAME"
log "Logs: $LOG_DIR"

gazebo_launch=(
  ros2 launch gen0_main spawn.launch.py
  world:="$WORLD"
  rviz:=false
  ground_truth_localization:=true
  render_env:=unset
)
if [[ -n "$GPU_ADAPTER" ]]; then
  gazebo_launch+=(d3d12_adapter:="$GPU_ADAPTER")
fi
if [[ -n "$ACTORS_SCENARIO" ]]; then
  gazebo_launch+=(actors_scenario:="$ACTORS_SCENARIO")
fi

start_process gazebo "${gazebo_launch[@]}"

sleep 12

fast_lio_launch=(
  ros2 launch gen0_main gen0_fast_lio_mapping.launch.py
  rviz:=false
  simulated_lidar:="$SIMULATED_LIDAR"
  terrain_analysis:=false
  local_costmap:=false
)
if [[ "$SIMULATED_LIDAR" == "true" ]]; then
  fast_lio_launch+=(world_obj_path:="$WORLD_OBJ_PATH")
fi

start_process fast_lio_3d_slam "${fast_lio_launch[@]}"

sleep 8

start_process mapping_drive \
  ros2 launch gen0_main gen0_mapping_drive.launch.py \
    enabled:=true \
    drive_speed:="$DRIVE_SPEED"

sleep 3

start_process pointcloud_preview \
  ros2 run gen0_main pointcloud_accumulator_preview --ros-args \
    -p input_topic:=/gen0_mapping/cloud_registered \
    -p output_topic:=/gen0_mapping/rviz/fast_lio_map \
    -p max_points:="$PREVIEW_MAX_POINTS" \
    -p voxel_size:="$PREVIEW_VOXEL_SIZE" \
    -p publish_period:="$PREVIEW_PERIOD"

sleep 2

start_process rviz \
  rviz2 -d "$RVIZ_CONFIG"

log "Stack is running. Press Ctrl+C in this terminal to stop everything launched by this script."
log "Preview topic: /gen0_mapping/rviz/fast_lio_map"

while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    if ! kill -0 "$pid" 2>/dev/null; then
      set +e
      wait "$pid"
      status=$?
      set -e
      log "$name exited with status $status. Check $LOG_DIR/$name.log"
      exit "$status"
    fi
  done
  sleep 3
done
