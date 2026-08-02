#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
WORLD="${GEN0_WORLD:-my_map}"
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO:-}"
TRASH_SCENARIO="${GEN0_TRASH_SCENARIO-small_trash_dense}"
TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-true}"
TRASH_VEHICLE_LENGTH="${GEN0_TRASH_VEHICLE_LENGTH:-4.40}"
TRASH_VEHICLE_WIDTH="${GEN0_TRASH_VEHICLE_WIDTH:-2.20}"
TRASH_VEHICLE_CENTER_OFFSET_X="${GEN0_TRASH_VEHICLE_CENTER_OFFSET_X:-0.0}"
TRASH_VEHICLE_CENTER_OFFSET_Y="${GEN0_TRASH_VEHICLE_CENTER_OFFSET_Y:-0.0}"
TRASH_COVERAGE_MARGIN="${GEN0_TRASH_COVERAGE_MARGIN:-0.0}"
TRASH_USE_MESH_VISUAL_CENTER="${GEN0_TRASH_USE_MESH_VISUAL_CENTER:-true}"
TRASH_VEHICLE_POSE_TOPIC="${GEN0_TRASH_VEHICLE_POSE_TOPIC-/gen0_model/links/poses}"
TRASH_VEHICLE_POSE_INDEX="${GEN0_TRASH_VEHICLE_POSE_INDEX:-15}"
TRASH_DEBUG_ITEM="${GEN0_TRASH_DEBUG_ITEM:-}"
TRASH_DEBUG_PERIOD="${GEN0_TRASH_DEBUG_PERIOD:-1.0}"
GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}"
PARTITION="${GEN0_PARTITION:-gen0_scurm_demo}"
GAZEBO_GUI="${GEN0_GAZEBO_GUI:-true}"
GROUND_TRUTH_LOCALIZATION="${GEN0_GROUND_TRUTH_LOCALIZATION:-false}"
STATIC_ODOM_BASE="${GEN0_STATIC_ODOM_BASE:-false}"
GAZEBO_BRIDGE_FILE="${GEN0_GAZEBO_BRIDGE_FILE:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/bridge_no_gz_odom_tf.yaml}"
MAPPING_DRIVE="${GEN0_MAPPING_DRIVE:-true}"
DRIVE_SPEED="${GEN0_DRIVE_SPEED:-0.25}"
RVIZ="${GEN0_RVIZ:-true}"
RVIZ_CONFIG="${GEN0_RVIZ_CONFIG:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_3d_mapping.rviz}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
FAST_LIO_ODOM_TOPIC="${GEN0_FAST_LIO_ODOM_TOPIC:-/gen0_mapping/fast_lio/odom}"
TERRAIN_ANALYSIS="${GEN0_TERRAIN_ANALYSIS:-true}"
TERRAIN_ANALYSIS_EXT="${GEN0_TERRAIN_ANALYSIS_EXT:-true}"
PROJECTED_MAP="${GEN0_PROJECTED_MAP:-true}"
PROJECTED_MAP_BACKEND="${GEN0_PROJECTED_MAP_BACKEND:-python}"
LOCAL_COSTMAP="${GEN0_LOCAL_COSTMAP:-true}"
PROJECTED_MAP_ODOM_GUARD="${GEN0_PROJECTED_MAP_ODOM_GUARD:-true}"
PROJECTED_MAP_REFERENCE_ODOM_TOPIC="${GEN0_PROJECTED_MAP_REFERENCE_ODOM_TOPIC:-/odom}"
PROJECTED_MAP_MAX_ODOM_ERROR="${GEN0_PROJECTED_MAP_MAX_ODOM_ERROR:-3.0}"
PROJECTED_MAP_MAX_YAW_ERROR="${GEN0_PROJECTED_MAP_MAX_YAW_ERROR:-0.75}"
PROJECTED_MAP_ODOM_TIMEOUT="${GEN0_PROJECTED_MAP_ODOM_TIMEOUT:-2.0}"
RELOCALIZATION="${GEN0_RELOCALIZATION:-false}"
PRIOR_MAP_PATH="${GEN0_PRIOR_MAP_PATH:-$HOME/SCURM_SentryNavigation/sentry_bringup/maps/GlobalMap.pcd}"
RELOCALIZATION_INITIAL_X="${GEN0_RELOCALIZATION_INITIAL_X:-0.0}"
RELOCALIZATION_INITIAL_Y="${GEN0_RELOCALIZATION_INITIAL_Y:-0.0}"
RELOCALIZATION_INITIAL_Z="${GEN0_RELOCALIZATION_INITIAL_Z:-0.0}"
RELOCALIZATION_INITIAL_A="${GEN0_RELOCALIZATION_INITIAL_A:-0.0}"
RELOCALIZATION_FITNESS_SCORE_THRESHOLD="${GEN0_RELOCALIZATION_FITNESS_SCORE_THRESHOLD:-1.0}"
RELOCALIZATION_CONVERGED_COUNT_THRESHOLD="${GEN0_RELOCALIZATION_CONVERGED_COUNT_THRESHOLD:-3}"
RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE="${GEN0_RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE:-2.0}"
RELOCALIZATION_INPUT_CLOUD_TO_BASE_X="${GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_X:-1.9}"
RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y="${GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y:-0.0}"
RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z="${GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z:-1.9}"
RELOCALIZATION_LEGACY_LIVOX_ROLL_180="${GEN0_RELOCALIZATION_LEGACY_LIVOX_ROLL_180:-false}"
RELOCALIZATION_WAIT_TIMEOUT="${GEN0_RELOCALIZATION_WAIT_TIMEOUT:-90}"
FAST_LIO_MAP_FILE_PATH="${GEN0_FAST_LIO_MAP_FILE_PATH:-/tmp/gen0_fast_lio_map.pcd}"
FAST_LIO_PCD_SAVE="${GEN0_FAST_LIO_PCD_SAVE:-false}"
FAST_LIO_PCD_SAVE_INTERVAL="${GEN0_FAST_LIO_PCD_SAVE_INTERVAL:--1}"
SIMULATED_LIDAR="${GEN0_SIMULATED_LIDAR:-true}"
GAZEBO_FRONT3D_TOPIC="${GEN0_GAZEBO_FRONT3D_TOPIC:-/gen0_model/front3d/lidar/points}"
SIMULATED_FRONT3D_TOPIC="${GEN0_SIMULATED_FRONT3D_TOPIC:-/gen0_mapping/simulated_front3d/lidar/points}"
FRONT3D_SOURCE_TOPIC="${GEN0_FRONT3D_SOURCE_TOPIC:-}"
WORLD_OBJ_PATH="${GEN0_WORLD_OBJ_PATH:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.obj}"
SIM_LIDAR_MAX_POINTS="${GEN0_SIM_LIDAR_MAX_POINTS:-48000}"
SIM_LIDAR_MAX_RANGE="${GEN0_SIM_LIDAR_MAX_RANGE:-35.0}"
SIM_LIDAR_HORIZONTAL_MIN="${GEN0_SIM_LIDAR_HORIZONTAL_MIN:--3.14159}"
SIM_LIDAR_HORIZONTAL_MAX="${GEN0_SIM_LIDAR_HORIZONTAL_MAX:-3.14159}"
SIM_LIDAR_VERTICAL_MIN="${GEN0_SIM_LIDAR_VERTICAL_MIN:--0.55}"
SIM_LIDAR_VERTICAL_MAX="${GEN0_SIM_LIDAR_VERTICAL_MAX:-0.55}"
SIM_LIDAR_WORLD_VOXEL_SIZE="${GEN0_SIM_LIDAR_WORLD_VOXEL_SIZE:-0.08}"
SIM_LIDAR_SURFACE_SAMPLING="${GEN0_SIM_LIDAR_SURFACE_SAMPLING:-true}"
SIM_LIDAR_SURFACE_SAMPLES="${GEN0_SIM_LIDAR_SURFACE_SAMPLES:-1000000}"
SIM_LIDAR_ADD_OBSTACLE_COLUMNS="${GEN0_SIM_LIDAR_ADD_OBSTACLE_COLUMNS:-false}"

PIDS=()
NAMES=()

if [[ -z "$FRONT3D_SOURCE_TOPIC" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" ]]; then
    FRONT3D_SOURCE_TOPIC="$SIMULATED_FRONT3D_TOPIC"
  else
    FRONT3D_SOURCE_TOPIC="$GAZEBO_FRONT3D_TOPIC"
  fi
fi

case "$PROJECTED_MAP_BACKEND" in
  octomap|python) ;;
  *)
    printf 'Invalid GEN0_PROJECTED_MAP_BACKEND=%s. Use octomap or python.\n' "$PROJECTED_MAP_BACKEND" >&2
    exit 1
    ;;
esac

STABLE_SIM_ODOM="${GEN0_STABLE_SIM_ODOM:-}"
if [[ -z "$STABLE_SIM_ODOM" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" && "$RELOCALIZATION" != "true" ]]; then
    STABLE_SIM_ODOM="true"
  else
    STABLE_SIM_ODOM="false"
  fi
fi
STABLE_ODOM_INPUT_TOPIC="${GEN0_STABLE_ODOM_INPUT_TOPIC:-/odom}"
STABLE_ODOM_TOPIC="${GEN0_STABLE_ODOM_TOPIC:-/gen0_mapping/stable_odom}"
STABLE_REGISTERED_SCAN_TOPIC="${GEN0_STABLE_REGISTERED_SCAN_TOPIC:-/gen0_mapping/stable_registered_scan}"
SCURM_ODOM_TOPIC="${GEN0_SCURM_ODOM_TOPIC:-}"
SCURM_REGISTERED_SCAN_TOPIC="${GEN0_SCURM_REGISTERED_SCAN_TOPIC:-}"
if [[ -z "$SCURM_ODOM_TOPIC" ]]; then
  if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
    SCURM_ODOM_TOPIC="$STABLE_ODOM_TOPIC"
  else
    SCURM_ODOM_TOPIC="$FAST_LIO_ODOM_TOPIC"
  fi
fi
if [[ -z "$SCURM_REGISTERED_SCAN_TOPIC" ]]; then
  if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
    SCURM_REGISTERED_SCAN_TOPIC="$STABLE_REGISTERED_SCAN_TOPIC"
  else
    SCURM_REGISTERED_SCAN_TOPIC="/gen0_mapping/cloud_registered"
  fi
fi
FAST_LIO_SEND_ODOM_BASE_TF="${GEN0_FAST_LIO_SEND_ODOM_BASE_TF:-}"
if [[ -z "$FAST_LIO_SEND_ODOM_BASE_TF" ]]; then
  if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
    FAST_LIO_SEND_ODOM_BASE_TF="false"
  else
    FAST_LIO_SEND_ODOM_BASE_TF="true"
  fi
fi
FAST_LIO_SENSOR_FRAME_ID="${GEN0_FAST_LIO_SENSOR_FRAME_ID:-}"
if [[ -z "$FAST_LIO_SENSOR_FRAME_ID" ]]; then
  if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
    FAST_LIO_SENSOR_FRAME_ID="fast_lio_base_link"
  else
    FAST_LIO_SENSOR_FRAME_ID="base_link"
  fi
fi
TRASH_ODOM_TOPIC="${GEN0_TRASH_ODOM_TOPIC:-$SCURM_ODOM_TOPIC}"

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
      | grep -E '(^/pose_publisher$|^/gen0_simulated_world_lidar$|^/gen0_gazebo_livox_adapter$|^/gen0_stable_odom$|^/gen0_odom_registered_scan$|^/gen0_icp_transform_publisher$|^/gen0_icp_relocalization$|^/gen0_fast_lio$|^/gen0_mapping_drive$|^/gen0_trash_cleanup$|^/gen0_scurm_terrain_analysis$|^/gen0_scurm_terrain_analysis_ext$|^/gen0_scurm_map_to_odom$|^/gen0_scurm_exchange_field$|^/gen0_scurm_sensor_scan_generation$|^/gen0_scurm_octomap_server$|^/gen0_projected_terrain_map$|^/costmap/costmap$|^/lifecycle_manager_costmap$|^/raw_front3d_preview$|^/cloud_registered_preview$|^/terrain_map_preview$|^/terrain_map_ext_preview$|^/fast_lio_map_preview$|^/pointcloud_accumulator_preview$|^/rviz$|^/rviz2$|^/vehicle_movement_interface$)' \
      || true
  )"
  if [[ -n "$existing" ]]; then
    printf 'Existing Gen0/RViz ROS nodes are still running:\n%s\n\n' "$existing" >&2
    printf 'Run ./stop_gen0_3d_slam.sh first, or set GEN0_ALLOW_EXISTING_NODES=true to bypass this check.\n' >&2
    exit 1
  fi
}

check_existing_gazebo_processes() {
  if [[ "${GEN0_ALLOW_EXISTING_NODES:-false}" == "true" ]]; then
    return
  fi

  local existing
  existing="$(
    pgrep -af 'ign gazebo( server| gui| .*gen0_main.*/worlds)' 2>/dev/null \
      | grep -v "pgrep -af" \
      || true
  )"
  if [[ -n "$existing" ]]; then
    printf 'Existing Gazebo processes are still running:\n%s\n\n' "$existing" >&2
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

wait_for_relocalized_odometry() {
  local fast_lio_pid="$1"

  if [[ "$RELOCALIZATION" != "true" || "$RELOCALIZATION_WAIT_TIMEOUT" == "0" ]]; then
    return 0
  fi

  log "Waiting for relocalized FAST-LIO odometry on $FAST_LIO_ODOM_TOPIC before starting drive (timeout=${RELOCALIZATION_WAIT_TIMEOUT}s)"
  local deadline=$((SECONDS + RELOCALIZATION_WAIT_TIMEOUT))
  while ((SECONDS < deadline)); do
    if ! process_group_alive "$fast_lio_pid"; then
      log "fast_lio_3d_slam exited before relocalized odometry became available"
      return 1
    fi
    if timeout 2s ros2 topic echo --once "$FAST_LIO_ODOM_TOPIC" --field header >/dev/null 2>&1; then
      log "Relocalized FAST-LIO odometry is available."
      return 0
    fi
    sleep 1
  done

  log "Timed out waiting for relocalized odometry. Check $LOG_DIR/fast_lio_3d_slam.log; mapping_drive and trash_cleanup will not start."
  return 1
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
require_file "$GAZEBO_BRIDGE_FILE"
if [[ "$SIMULATED_LIDAR" == "true" ]]; then
  require_file "$WORLD_OBJ_PATH"
fi
if [[ "$RELOCALIZATION" == "true" ]]; then
  require_file "$PRIOR_MAP_PATH"
fi
if [[ "$FAST_LIO_PCD_SAVE" == "true" ]]; then
  mkdir -p "$(dirname "$FAST_LIO_MAP_FILE_PATH")"
fi

mkdir -p "$LOG_DIR"
mkdir -p "$ROS_LOG_DIR"

export ROS_LOG_DIR

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"
check_existing_ros_nodes
check_existing_gazebo_processes

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GPU_ADAPTER"
export IGN_PARTITION="$PARTITION"
export GZ_PARTITION="$PARTITION"

log "Workspace: $WORKSPACE"
log "World: $WORLD, actors_scenario: $ACTORS_SCENARIO, trash_scenario=$TRASH_SCENARIO, trash_cleanup=$TRASH_CLEANUP, trash_vehicle=${TRASH_VEHICLE_LENGTH}x${TRASH_VEHICLE_WIDTH}, trash_center_offset=(${TRASH_VEHICLE_CENTER_OFFSET_X},${TRASH_VEHICLE_CENTER_OFFSET_Y}), coverage_margin=$TRASH_COVERAGE_MARGIN, mesh_visual_center=$TRASH_USE_MESH_VISUAL_CENTER, trash_vehicle_pose=${TRASH_VEHICLE_POSE_TOPIC:-<odom>}[$TRASH_VEHICLE_POSE_INDEX], gazebo_gui=$GAZEBO_GUI, partition=$PARTITION"
log "Simulated lidar: $SIMULATED_LIDAR, world_obj_path: $WORLD_OBJ_PATH"
log "Front 3D source topic: $FRONT3D_SOURCE_TOPIC, simulated_topic=$SIMULATED_FRONT3D_TOPIC, gazebo_topic=$GAZEBO_FRONT3D_TOPIC"
log "TF localization: ground_truth_localization=$GROUND_TRUTH_LOCALIZATION, static_odom_base=$STATIC_ODOM_BASE"
log "Gazebo bridge file: $GAZEBO_BRIDGE_FILE"
log "Mapping drive: $MAPPING_DRIVE, drive_speed=$DRIVE_SPEED, command_topic=/cmd_vel"
log "SCURM view: terrain_analysis=$TERRAIN_ANALYSIS, terrain_analysis_ext=$TERRAIN_ANALYSIS_EXT, projected_map=$PROJECTED_MAP, local_costmap=$LOCAL_COSTMAP, relocalization=$RELOCALIZATION, rviz=$RVIZ"
log "Projected map backend: $PROJECTED_MAP_BACKEND"
log "Odometry chain: fast_lio_output=$FAST_LIO_ODOM_TOPIC, scurm_odom=$SCURM_ODOM_TOPIC, stable_sim_odom=$STABLE_SIM_ODOM"
if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
  log "Stable sim odom: input=$STABLE_ODOM_INPUT_TOPIC, output=$STABLE_ODOM_TOPIC, registered_scan=$STABLE_REGISTERED_SCAN_TOPIC, scurm_registered_scan=$SCURM_REGISTERED_SCAN_TOPIC"
  log "FAST-LIO TF isolation: send_odom_base_tf=$FAST_LIO_SEND_ODOM_BASE_TF, sensor_frame_id=$FAST_LIO_SENSOR_FRAME_ID"
fi
if [[ "$PROJECTED_MAP_ODOM_GUARD" == "true" ]]; then
  log "Projected-map odom guard: reference=$PROJECTED_MAP_REFERENCE_ODOM_TOPIC, max_xy_error=${PROJECTED_MAP_MAX_ODOM_ERROR}m, max_yaw_error=${PROJECTED_MAP_MAX_YAW_ERROR}rad"
else
  log "Projected-map odom guard: disabled"
  PROJECTED_MAP_REFERENCE_ODOM_TOPIC=""
  PROJECTED_MAP_MAX_ODOM_ERROR="0.0"
  PROJECTED_MAP_MAX_YAW_ERROR="0.0"
fi
log "FAST-LIO map save: pcd_save=$FAST_LIO_PCD_SAVE, map_file_path=$FAST_LIO_MAP_FILE_PATH, interval=$FAST_LIO_PCD_SAVE_INTERVAL"
if [[ "$RELOCALIZATION" == "true" ]]; then
  log "SCURM relocalization: prior_map_path=$PRIOR_MAP_PATH, initial=($RELOCALIZATION_INITIAL_X, $RELOCALIZATION_INITIAL_Y, $RELOCALIZATION_INITIAL_Z, $RELOCALIZATION_INITIAL_A), fitness_threshold=$RELOCALIZATION_FITNESS_SCORE_THRESHOLD, converged_count_threshold=$RELOCALIZATION_CONVERGED_COUNT_THRESHOLD, max_correspondence_distance=$RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE"
  log "SCURM relocalization input cloud: lidar_to_base=($RELOCALIZATION_INPUT_CLOUD_TO_BASE_X, $RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y, $RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z), legacy_livox_roll_180=$RELOCALIZATION_LEGACY_LIVOX_ROLL_180, wait_timeout=$RELOCALIZATION_WAIT_TIMEOUT"
fi
if [[ "$SIMULATED_LIDAR" == "true" ]]; then
  log "Simulated lidar scan: max_points=$SIM_LIDAR_MAX_POINTS, max_range=$SIM_LIDAR_MAX_RANGE, h=[$SIM_LIDAR_HORIZONTAL_MIN, $SIM_LIDAR_HORIZONTAL_MAX], v=[$SIM_LIDAR_VERTICAL_MIN, $SIM_LIDAR_VERTICAL_MAX]"
  log "Simulated lidar mesh: world_voxel_size=$SIM_LIDAR_WORLD_VOXEL_SIZE, surface_sampling=$SIM_LIDAR_SURFACE_SAMPLING, surface_samples=$SIM_LIDAR_SURFACE_SAMPLES, add_obstacle_columns=$SIM_LIDAR_ADD_OBSTACLE_COLUMNS"
fi
log "GPU adapter: $MESA_D3D12_DEFAULT_ADAPTER_NAME"
log "Logs: $LOG_DIR"
log "ROS logs: $ROS_LOG_DIR"

gazebo_launch=(
  ros2 launch gen0_main spawn.launch.py
  world:="$WORLD"
  gazebo_gui:="$GAZEBO_GUI"
  partition:="$PARTITION"
  rviz:=false
  ground_truth_localization:="$GROUND_TRUTH_LOCALIZATION"
  static_odom_base:="$STATIC_ODOM_BASE"
  bridge_file:="$GAZEBO_BRIDGE_FILE"
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

if [[ -n "$TRASH_SCENARIO" ]]; then
  log "Spawning trash scenario: $TRASH_SCENARIO"
  ros2 launch gen0_main trash_spawn.launch.py \
    world:="$WORLD" \
    trash_scenario:="$TRASH_SCENARIO" \
    partition:="$PARTITION" \
    >"$LOG_DIR/trash_spawner.log" 2>&1 || {
      log "trash_spawner failed. Check $LOG_DIR/trash_spawner.log"
      exit 1
    }
fi

fast_lio_launch=(
  ros2 launch gen0_main gen0_fast_lio_mapping.launch.py
  odom_output_topic:="$FAST_LIO_ODOM_TOPIC"
  fast_lio_send_odom_base_tf:="$FAST_LIO_SEND_ODOM_BASE_TF"
  fast_lio_sensor_frame_id:="$FAST_LIO_SENSOR_FRAME_ID"
  stable_sim_odom:="$STABLE_SIM_ODOM"
  stable_odom_input_topic:="$STABLE_ODOM_INPUT_TOPIC"
  stable_odom_output_topic:="$STABLE_ODOM_TOPIC"
  stable_registered_scan_topic:="$STABLE_REGISTERED_SCAN_TOPIC"
  scurm_odom_topic:="$SCURM_ODOM_TOPIC"
  scurm_registered_scan_topic:="$SCURM_REGISTERED_SCAN_TOPIC"
  rviz:="$RVIZ"
  rviz_config:="$RVIZ_CONFIG"
  simulated_lidar:="$SIMULATED_LIDAR"
  terrain_analysis:="$TERRAIN_ANALYSIS"
  terrain_analysis_ext:="$TERRAIN_ANALYSIS_EXT"
  projected_map:="$PROJECTED_MAP"
  projected_map_backend:="$PROJECTED_MAP_BACKEND"
  projected_map_reference_odom_topic:="$PROJECTED_MAP_REFERENCE_ODOM_TOPIC"
  projected_map_max_reference_odom_error:="$PROJECTED_MAP_MAX_ODOM_ERROR"
  projected_map_max_reference_yaw_error:="$PROJECTED_MAP_MAX_YAW_ERROR"
  projected_map_reference_odom_timeout:="$PROJECTED_MAP_ODOM_TIMEOUT"
  local_costmap:="$LOCAL_COSTMAP"
  front3d_source_topic:="$FRONT3D_SOURCE_TOPIC"
  simulated_lidar_output_topic:="$SIMULATED_FRONT3D_TOPIC"
  relocalization:="$RELOCALIZATION"
  prior_map_path:="$PRIOR_MAP_PATH"
  relocalization_initial_x:="$RELOCALIZATION_INITIAL_X"
  relocalization_initial_y:="$RELOCALIZATION_INITIAL_Y"
  relocalization_initial_z:="$RELOCALIZATION_INITIAL_Z"
  relocalization_initial_a:="$RELOCALIZATION_INITIAL_A"
  relocalization_fitness_score_threshold:="$RELOCALIZATION_FITNESS_SCORE_THRESHOLD"
  relocalization_converged_count_threshold:="$RELOCALIZATION_CONVERGED_COUNT_THRESHOLD"
  relocalization_max_correspondence_distance:="$RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE"
  relocalization_input_cloud_to_base_x:="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_X"
  relocalization_input_cloud_to_base_y:="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y"
  relocalization_input_cloud_to_base_z:="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z"
  relocalization_legacy_livox_roll_180:="$RELOCALIZATION_LEGACY_LIVOX_ROLL_180"
  fast_lio_map_file_path:="$FAST_LIO_MAP_FILE_PATH"
  fast_lio_pcd_save:="$FAST_LIO_PCD_SAVE"
  fast_lio_pcd_save_interval:="$FAST_LIO_PCD_SAVE_INTERVAL"
)
if [[ "$SIMULATED_LIDAR" == "true" ]]; then
  fast_lio_launch+=(world_obj_path:="$WORLD_OBJ_PATH")
  fast_lio_launch+=(simulated_lidar_max_points:="$SIM_LIDAR_MAX_POINTS")
  fast_lio_launch+=(simulated_lidar_max_range:="$SIM_LIDAR_MAX_RANGE")
  fast_lio_launch+=(simulated_lidar_horizontal_min_angle:="$SIM_LIDAR_HORIZONTAL_MIN")
  fast_lio_launch+=(simulated_lidar_horizontal_max_angle:="$SIM_LIDAR_HORIZONTAL_MAX")
  fast_lio_launch+=(simulated_lidar_vertical_min_angle:="$SIM_LIDAR_VERTICAL_MIN")
  fast_lio_launch+=(simulated_lidar_vertical_max_angle:="$SIM_LIDAR_VERTICAL_MAX")
  fast_lio_launch+=(simulated_lidar_world_voxel_size:="$SIM_LIDAR_WORLD_VOXEL_SIZE")
  fast_lio_launch+=(simulated_lidar_surface_sampling:="$SIM_LIDAR_SURFACE_SAMPLING")
  fast_lio_launch+=(simulated_lidar_surface_samples:="$SIM_LIDAR_SURFACE_SAMPLES")
  fast_lio_launch+=(simulated_lidar_add_obstacle_columns:="$SIM_LIDAR_ADD_OBSTACLE_COLUMNS")
fi

start_process fast_lio_3d_slam "${fast_lio_launch[@]}"
FAST_LIO_PID="${PIDS[-1]}"

if wait_for_relocalized_odometry "$FAST_LIO_PID"; then
  if [[ "$RELOCALIZATION" != "true" ]]; then
    sleep 8
  fi

  if [[ "$MAPPING_DRIVE" == "true" ]]; then
    start_process mapping_drive \
      ros2 launch gen0_main gen0_mapping_drive.launch.py \
        enabled:=true \
        drive_speed:="$DRIVE_SPEED" \
        front3d_topic:="$FRONT3D_SOURCE_TOPIC"
  else
    log "Skipping mapping_drive because GEN0_MAPPING_DRIVE=$MAPPING_DRIVE"
  fi

  if [[ -n "$TRASH_SCENARIO" && "$TRASH_CLEANUP" == "true" ]]; then
    trash_cleanup_launch=(
      ros2 launch gen0_main trash_cleanup.launch.py \
      world:="$WORLD" \
      trash_scenario:="$TRASH_SCENARIO" \
      partition:="$PARTITION" \
      odom_topic:="$TRASH_ODOM_TOPIC" \
      vehicle_pose_topic:="$TRASH_VEHICLE_POSE_TOPIC" \
      vehicle_pose_index:="$TRASH_VEHICLE_POSE_INDEX" \
      vehicle_length:="$TRASH_VEHICLE_LENGTH" \
      vehicle_width:="$TRASH_VEHICLE_WIDTH" \
      vehicle_center_offset_x:="$TRASH_VEHICLE_CENTER_OFFSET_X" \
      vehicle_center_offset_y:="$TRASH_VEHICLE_CENTER_OFFSET_Y" \
      coverage_margin:="$TRASH_COVERAGE_MARGIN" \
      use_mesh_visual_center:="$TRASH_USE_MESH_VISUAL_CENTER" \
      debug_period:="$TRASH_DEBUG_PERIOD"
    )
    if [[ -n "$TRASH_DEBUG_ITEM" ]]; then
      trash_cleanup_launch+=(debug_item:="$TRASH_DEBUG_ITEM")
    fi
    start_process trash_cleanup "${trash_cleanup_launch[@]}"
  fi
fi

log "Stack is running. Press Ctrl+C in this terminal to stop everything launched by this script."
log "Preview topic: /gen0_mapping/rviz/fast_lio_map"
if [[ "$PROJECTED_MAP_BACKEND" == "octomap" ]]; then
  log "Projected map backend: SCURM octomap chain, topics: /projected_map and /projected_map_updates"
else
  log "Projected map backend: SCURM terrain projection, topics: /projected_map and /projected_costmap"
fi

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
