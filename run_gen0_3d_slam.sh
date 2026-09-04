#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
WORLD="${GEN0_WORLD:-my_map}"
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO:-}"
TRASH_SCENARIO="${GEN0_TRASH_SCENARIO-small_trash_dense}"
TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-true}"
TRASH_VEHICLE_LENGTH="${GEN0_TRASH_VEHICLE_LENGTH:-2.40}"
TRASH_VEHICLE_WIDTH="${GEN0_TRASH_VEHICLE_WIDTH:-1.65}"
TRASH_VEHICLE_CENTER_OFFSET_X="${GEN0_TRASH_VEHICLE_CENTER_OFFSET_X:-0.0}"
TRASH_VEHICLE_CENTER_OFFSET_Y="${GEN0_TRASH_VEHICLE_CENTER_OFFSET_Y:-0.0}"
TRASH_COVERAGE_MARGIN="${GEN0_TRASH_COVERAGE_MARGIN:-0.0}"
TRASH_USE_MESH_VISUAL_CENTER="${GEN0_TRASH_USE_MESH_VISUAL_CENTER:-true}"
TRASH_VEHICLE_POSE_TOPIC="${GEN0_TRASH_VEHICLE_POSE_TOPIC-/gen0_model/links/poses}"
TRASH_VEHICLE_POSE_INDEX="${GEN0_TRASH_VEHICLE_POSE_INDEX:-15}"
TRASH_DEBUG_ITEM="${GEN0_TRASH_DEBUG_ITEM:-}"
TRASH_DEBUG_PERIOD="${GEN0_TRASH_DEBUG_PERIOD:-1.0}"
TRASH_FUSION_DETECTION="${GEN0_TRASH_FUSION_DETECTION:-false}"
TRASH_FUSION_MODEL_PATH="${GEN0_TRASH_FUSION_MODEL_PATH:-$WORKSPACE/best.pt}"
TRASH_FUSION_OUTPUT_FRAME="${GEN0_TRASH_FUSION_OUTPUT_FRAME:-}"
TRASH_FUSION_IMAGE_TOPIC="${GEN0_TRASH_FUSION_IMAGE_TOPIC:-/gen0_model/front_camera}"
TRASH_FUSION_CAMERA_INFO_TOPIC="${GEN0_TRASH_FUSION_CAMERA_INFO_TOPIC:-/gen0_model/camera_info}"
TRASH_FUSION_POINTCLOUD_TOPIC="${GEN0_TRASH_FUSION_POINTCLOUD_TOPIC:-}"
GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}"
PARTITION="${GEN0_PARTITION:-gen0_scurm_demo}"
GAZEBO_GUI="${GEN0_GAZEBO_GUI:-true}"
GROUND_TRUTH_LOCALIZATION="${GEN0_GROUND_TRUTH_LOCALIZATION:-false}"
STATIC_ODOM_BASE="${GEN0_STATIC_ODOM_BASE:-false}"
GAZEBO_BRIDGE_FILE="${GEN0_GAZEBO_BRIDGE_FILE:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/bridge_no_gz_odom_tf.yaml}"
MAPPING_DRIVE="${GEN0_MAPPING_DRIVE:-false}"
DRIVE_SPEED="${GEN0_DRIVE_SPEED:-0.25}"
RVIZ="${GEN0_RVIZ:-true}"
DEFAULT_RVIZ_CONFIG="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_3d_mapping.rviz"
DEFAULT_RELOCALIZATION_RVIZ_CONFIG="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_relocalization_loam_livox.rviz"
RVIZ_CONFIG="${GEN0_RVIZ_CONFIG:-}"
RVIZ_RENDER_ENV="${GEN0_RVIZ_RENDER_ENV:-passthrough}"
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
PRIOR_MAP_PATH="${GEN0_PRIOR_MAP_PATH:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/maps/prior_map.pcd}"
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
GAZEBO_RENDER_ENV="${GEN0_GAZEBO_RENDER_ENV:-unset}"
FAST_LIO_MAP_FILE_PATH="${GEN0_FAST_LIO_MAP_FILE_PATH:-/tmp/gen0_fast_lio_map.pcd}"
FAST_LIO_PCD_SAVE="${GEN0_FAST_LIO_PCD_SAVE:-false}"
FAST_LIO_PCD_SAVE_INTERVAL="${GEN0_FAST_LIO_PCD_SAVE_INTERVAL:--1}"
SIMULATED_LIDAR="${GEN0_SIMULATED_LIDAR:-true}"
GAZEBO_FRONT3D_TOPIC="${GEN0_GAZEBO_FRONT3D_TOPIC:-/gen0_model/front3d/lidar/points}"
SIMULATED_FRONT3D_TOPIC="${GEN0_SIMULATED_FRONT3D_TOPIC:-/gen0_mapping/simulated_front3d/lidar/points}"
SIMULATED_FILTERED_FRONT3D_TOPIC="${GEN0_SIMULATED_FILTERED_FRONT3D_TOPIC:-/gen0_mapping/simulated_front3d/lidar/points_no_trash}"
FRONT3D_SOURCE_TOPIC="${GEN0_FRONT3D_SOURCE_TOPIC:-}"
REGISTERED_SCAN_INPUT_TOPIC="${GEN0_REGISTERED_SCAN_INPUT_TOPIC:-}"
REGISTERED_SCAN_ODOM_TOPIC="${GEN0_REGISTERED_SCAN_ODOM_TOPIC:-}"
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
DYNAMIC_ACTOR_TOPICS="${GEN0_DYNAMIC_ACTOR_TOPICS:-}"
ACTOR_COSTMAP_POSE_TOPICS="${GEN0_ACTOR_COSTMAP_POSE_TOPICS:-}"
DYNAMIC_VEHICLE_TOPICS="${GEN0_DYNAMIC_VEHICLE_TOPICS:-/car/car_008/pose,/car/car_009/pose}"
ACTOR_COSTMAP="${GEN0_ACTOR_COSTMAP:-true}"
ACTOR_OBSTACLE_TOPIC="${GEN0_ACTOR_OBSTACLE_TOPIC:-/gen0_mapping/actor_obstacles}"
ACTOR_OBSTACLE_FRAME="${GEN0_ACTOR_OBSTACLE_FRAME:-odom}"
ACTOR_WORLD_SDF_PATH="${GEN0_ACTOR_WORLD_SDF_PATH:-}"
ACTOR_WORLD_VEHICLE_NAME="${GEN0_ACTOR_WORLD_VEHICLE_NAME:-gen0_model}"
ACTOR_WORLD_TO_OUTPUT="${GEN0_ACTOR_WORLD_TO_OUTPUT:-true}"
ACTOR_OUTPUT_ORIGIN_XY="${GEN0_ACTOR_OUTPUT_ORIGIN_XY:-0.0,0.0}"
ACTOR_COLLISION_MONITOR="${GEN0_ACTOR_COLLISION_MONITOR:-true}"
ACTOR_COLLISION_EVENT_TOPIC="${GEN0_ACTOR_COLLISION_EVENT_TOPIC:-/gen0_validation/actor_collision_events}"
ACTOR_COLLISION_NEAR_MARGIN="${GEN0_ACTOR_COLLISION_NEAR_MARGIN:-0.75}"
ACTOR_SOFT_STOP="${GEN0_ACTOR_SOFT_STOP:-false}"
ACTOR_SOFT_STOP_MARGIN="${GEN0_ACTOR_SOFT_STOP_MARGIN:-0.25}"
ACTOR_SOFT_STOP_RELEASE_MARGIN="${GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN:-0.85}"
ACTOR_SOFT_STOP_VEHICLE_NAME="${GEN0_ACTOR_SOFT_STOP_VEHICLE_NAME:-gen0_model}"
TRASH_SCENARIO_PATH="${GEN0_TRASH_SCENARIO_PATH:-}"

if [[ -z "${GEN0_TRASH_SCENARIO+x}" && "$RELOCALIZATION" == "true" && "$MAPPING_DRIVE" != "true" ]]; then
  TRASH_SCENARIO=""
fi

PIDS=()
NAMES=()
CRITICALS=()

if [[ -z "$FRONT3D_SOURCE_TOPIC" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" ]]; then
    FRONT3D_SOURCE_TOPIC="$SIMULATED_FRONT3D_TOPIC"
  else
    FRONT3D_SOURCE_TOPIC="$GAZEBO_FRONT3D_TOPIC"
  fi
fi
if [[ -z "$REGISTERED_SCAN_INPUT_TOPIC" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" ]]; then
    REGISTERED_SCAN_INPUT_TOPIC="$SIMULATED_FILTERED_FRONT3D_TOPIC"
  else
    REGISTERED_SCAN_INPUT_TOPIC="$FRONT3D_SOURCE_TOPIC"
  fi
fi
if [[ -z "$TRASH_FUSION_POINTCLOUD_TOPIC" ]]; then
  TRASH_FUSION_POINTCLOUD_TOPIC="$FRONT3D_SOURCE_TOPIC"
fi

ACTORS_SCENARIO_PATH=""
if [[ -n "$ACTORS_SCENARIO" ]]; then
  ACTORS_SCENARIO_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/scenarios/$WORLD/$ACTORS_SCENARIO.sdf"
fi

if [[ -z "$ACTOR_COSTMAP_POSE_TOPICS" && -n "$ACTORS_SCENARIO" ]]; then
  for actor_index in {1..20}; do
    actor_topic="/actor/pedestrian_${actor_index}/pose"
    if [[ -n "$ACTOR_COSTMAP_POSE_TOPICS" ]]; then
      ACTOR_COSTMAP_POSE_TOPICS+=","
    fi
    ACTOR_COSTMAP_POSE_TOPICS+="$actor_topic"
  done
fi

if [[ -z "$TRASH_SCENARIO_PATH" && -n "$TRASH_SCENARIO" ]]; then
  TRASH_SCENARIO_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/trash_scenarios/$WORLD/$TRASH_SCENARIO.json"
fi
if [[ -z "$ACTOR_WORLD_SDF_PATH" ]]; then
  ACTOR_WORLD_SDF_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.sdf"
fi

append_launch_arg_if_non_empty() {
  local name="$1"
  local value="$2"
  if [[ -n "$value" ]]; then
    fast_lio_launch+=("$name:=$value")
  fi
}

case "$PROJECTED_MAP_BACKEND" in
  octomap|python) ;;
  *)
    printf 'Invalid GEN0_PROJECTED_MAP_BACKEND=%s. Use octomap or python.\n' "$PROJECTED_MAP_BACKEND" >&2
    exit 1
    ;;
esac

STABLE_SIM_ODOM="${GEN0_STABLE_SIM_ODOM:-}"
if [[ -z "$STABLE_SIM_ODOM" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" ]]; then
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
  if [[ "$SIMULATED_LIDAR" == "true" || "$STABLE_SIM_ODOM" == "true" ]]; then
    SCURM_REGISTERED_SCAN_TOPIC="$STABLE_REGISTERED_SCAN_TOPIC"
  else
    SCURM_REGISTERED_SCAN_TOPIC="/gen0_mapping/cloud_registered"
  fi
fi
if [[ -z "$REGISTERED_SCAN_ODOM_TOPIC" ]]; then
  if [[ "$STABLE_SIM_ODOM" == "true" ]]; then
    REGISTERED_SCAN_ODOM_TOPIC="$STABLE_ODOM_TOPIC"
  else
    REGISTERED_SCAN_ODOM_TOPIC="$SCURM_ODOM_TOPIC"
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
if [[ -z "$ACTOR_OBSTACLE_FRAME" ]]; then
  if [[ "$RELOCALIZATION" == "true" ]]; then
    ACTOR_OBSTACLE_FRAME="map"
  else
    ACTOR_OBSTACLE_FRAME="odom"
  fi
fi
if [[ -z "$TRASH_FUSION_OUTPUT_FRAME" ]]; then
  if [[ "$RELOCALIZATION" == "true" ]]; then
    TRASH_FUSION_OUTPUT_FRAME="map"
  else
    TRASH_FUSION_OUTPUT_FRAME="odom"
  fi
fi

if [[ -z "$RVIZ_CONFIG" ]]; then
  if [[ "$RELOCALIZATION" == "true" ]]; then
    RVIZ_CONFIG="$DEFAULT_RELOCALIZATION_RVIZ_CONFIG"
  else
    RVIZ_CONFIG="$DEFAULT_RVIZ_CONFIG"
  fi
fi

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
    timeout 5s ros2 node list --no-daemon 2>/dev/null \
      | grep -E '(^/pose_publisher$|^/gen0_simulated_world_lidar$|^/gen0_gazebo_livox_adapter$|^/gen0_trash_fusion_detector$|^/gen0_stable_odom$|^/gen0_odom_registered_scan$|^/gen0_icp_transform_publisher$|^/gen0_icp_relocalization$|^/gen0_fast_lio$|^/gen0_mapping_drive$|^/gen0_trash_cleanup$|^/gen0_scurm_terrain_analysis$|^/gen0_scurm_terrain_analysis_ext$|^/gen0_actor_obstacle_costmap$|^/gen0_scurm_map_to_odom$|^/gen0_scurm_exchange_field$|^/gen0_scurm_sensor_scan_generation$|^/gen0_scurm_octomap_server$|^/gen0_projected_terrain_map$|^/costmap/costmap$|^/lifecycle_manager_costmap$|^/raw_front3d_preview$|^/cloud_registered_preview$|^/terrain_map_preview$|^/terrain_map_ext_preview$|^/fast_lio_map_preview$|^/pointcloud_accumulator_preview$|^/gen0_mapping_rviz$|^/gen0_nav2_rviz$|^/rviz$|^/rviz2$|^/vehicle_movement_interface$)' \
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

start_managed_process() {
  local critical="$1"
  local name="$2"
  shift 2

  log "Starting $name"
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("$name")
  CRITICALS+=("$critical")
  log "$name pid=${PIDS[-1]} log=$LOG_DIR/$name.log"
}

start_process() {
  start_managed_process true "$@"
}

start_optional_process() {
  start_managed_process false "$@"
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
    if timeout 2s ros2 topic echo --no-daemon --once "$FAST_LIO_ODOM_TOPIC" --field header >/dev/null 2>&1; then
      log "Relocalized FAST-LIO odometry is available."
      return 0
    fi
    sleep 1
  done

  log "Timed out waiting for relocalized odometry. Check $LOG_DIR/fast_lio_3d_slam.log; mapping_drive and trash_cleanup will not start."
  return 1
}

start_trash_cleanup_if_enabled() {
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
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM

  if ((${#PIDS[@]} > 0)); then
    log "Stopping launched processes"
    for pid in "${PIDS[@]}"; do
      [[ -z "$pid" ]] && continue
      if process_group_alive "$pid"; then
        signal_process_group INT "$pid"
      fi
    done
    sleep 4
    for pid in "${PIDS[@]}"; do
      [[ -z "$pid" ]] && continue
      if process_group_alive "$pid"; then
        signal_process_group TERM "$pid"
      fi
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
      [[ -z "$pid" ]] && continue
      if process_group_alive "$pid"; then
        signal_process_group KILL "$pid"
      fi
    done
    for pid in "${PIDS[@]}"; do
      [[ -z "$pid" ]] && continue
      wait "$pid" 2>/dev/null || true
    done
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
if [[ -n "$ACTORS_SCENARIO_PATH" ]]; then
  require_file "$ACTORS_SCENARIO_PATH"
fi
if [[ -n "$TRASH_SCENARIO_PATH" ]]; then
  require_file "$TRASH_SCENARIO_PATH"
fi
if [[ "$TRASH_FUSION_DETECTION" == "true" ]]; then
  require_file "$TRASH_FUSION_MODEL_PATH"
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
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"
log "Preflight: checking for existing Gen0/RViz ROS nodes and Gazebo processes"
check_existing_ros_nodes
check_existing_gazebo_processes

unset LIBGL_ALWAYS_SOFTWARE
unset MESA_LOADER_DRIVER_OVERRIDE
unset GALLIUM_DRIVER
unset MESA_GL_VERSION_OVERRIDE
unset QT_XCB_GL_INTEGRATION
export IGN_PARTITION="$PARTITION"
export GZ_PARTITION="$PARTITION"

log "Workspace: $WORKSPACE"
log "World: $WORLD, actors_scenario: $ACTORS_SCENARIO, trash_scenario=$TRASH_SCENARIO, trash_cleanup=$TRASH_CLEANUP, trash_vehicle=${TRASH_VEHICLE_LENGTH}x${TRASH_VEHICLE_WIDTH}, trash_center_offset=(${TRASH_VEHICLE_CENTER_OFFSET_X},${TRASH_VEHICLE_CENTER_OFFSET_Y}), coverage_margin=$TRASH_COVERAGE_MARGIN, mesh_visual_center=$TRASH_USE_MESH_VISUAL_CENTER, trash_vehicle_pose=${TRASH_VEHICLE_POSE_TOPIC:-<odom>}[$TRASH_VEHICLE_POSE_INDEX], gazebo_gui=$GAZEBO_GUI, partition=$PARTITION"
log "Simulated lidar: $SIMULATED_LIDAR, world_obj_path: $WORLD_OBJ_PATH"
log "Front 3D source topic: $FRONT3D_SOURCE_TOPIC, simulated_topic=$SIMULATED_FRONT3D_TOPIC, gazebo_topic=$GAZEBO_FRONT3D_TOPIC"
log "TF localization: ground_truth_localization=$GROUND_TRUTH_LOCALIZATION, static_odom_base=$STATIC_ODOM_BASE"
log "Gazebo bridge file: $GAZEBO_BRIDGE_FILE"
log "Gazebo render_env: $GAZEBO_RENDER_ENV"
log "RViz config: $RVIZ_CONFIG"
log "RViz render env: $RVIZ_RENDER_ENV"
log "Mapping drive: $MAPPING_DRIVE, drive_speed=$DRIVE_SPEED, command_topic=/cmd_vel"
log "SCURM view: terrain_analysis=$TERRAIN_ANALYSIS, terrain_analysis_ext=$TERRAIN_ANALYSIS_EXT, projected_map=$PROJECTED_MAP, local_costmap=$LOCAL_COSTMAP, relocalization=$RELOCALIZATION, rviz=$RVIZ"
log "Projected map backend: $PROJECTED_MAP_BACKEND"
log "Odometry chain: fast_lio_output=$FAST_LIO_ODOM_TOPIC, scurm_odom=$SCURM_ODOM_TOPIC, stable_sim_odom=$STABLE_SIM_ODOM"
log "SCURM scan chain: input=$REGISTERED_SCAN_INPUT_TOPIC, odom=$REGISTERED_SCAN_ODOM_TOPIC, output=$SCURM_REGISTERED_SCAN_TOPIC"
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
  log "Simulated lidar dynamic objects: actor_topics=${DYNAMIC_ACTOR_TOPICS:-none}, vehicle_topics=${DYNAMIC_VEHICLE_TOPICS:-none}, trash_scenario_path=${TRASH_SCENARIO_PATH:-none}"
fi
log "Actor costmap source: enabled=$ACTOR_COSTMAP, topic=$ACTOR_OBSTACLE_TOPIC, frame=$ACTOR_OBSTACLE_FRAME, scenario_path=${ACTORS_SCENARIO_PATH:-none}, world_sdf=$ACTOR_WORLD_SDF_PATH, world_to_output=$ACTOR_WORLD_TO_OUTPUT, output_origin_xy=$ACTOR_OUTPUT_ORIGIN_XY"
log "Actor soft-stop: enabled=$ACTOR_SOFT_STOP, vehicle=$ACTOR_SOFT_STOP_VEHICLE_NAME, stop_margin=$ACTOR_SOFT_STOP_MARGIN, release_margin=$ACTOR_SOFT_STOP_RELEASE_MARGIN"
log "Trash fusion detection: enabled=$TRASH_FUSION_DETECTION, model=$TRASH_FUSION_MODEL_PATH, output_frame=$TRASH_FUSION_OUTPUT_FRAME, cloud=$TRASH_FUSION_POINTCLOUD_TOPIC, image=$TRASH_FUSION_IMAGE_TOPIC"
log "GPU adapter: $GPU_ADAPTER"
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
  render_env:="$GAZEBO_RENDER_ENV"
  actor_soft_stop:="$ACTOR_SOFT_STOP"
  actor_soft_stop_margin:="$ACTOR_SOFT_STOP_MARGIN"
  actor_soft_stop_release_margin:="$ACTOR_SOFT_STOP_RELEASE_MARGIN"
  actor_soft_stop_vehicle_name:="$ACTOR_SOFT_STOP_VEHICLE_NAME"
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
  registered_scan_input_topic:="$REGISTERED_SCAN_INPUT_TOPIC"
  registered_scan_odom_topic:="$REGISTERED_SCAN_ODOM_TOPIC"
  scurm_odom_topic:="$SCURM_ODOM_TOPIC"
  scurm_registered_scan_topic:="$SCURM_REGISTERED_SCAN_TOPIC"
  rviz:=false
  rviz_config:="$RVIZ_CONFIG"
  rviz_render_env:="$RVIZ_RENDER_ENV"
  simulated_lidar:="$SIMULATED_LIDAR"
  terrain_analysis:="$TERRAIN_ANALYSIS"
  terrain_analysis_ext:="$TERRAIN_ANALYSIS_EXT"
  projected_map:="$PROJECTED_MAP"
  projected_map_backend:="$PROJECTED_MAP_BACKEND"
  projected_map_max_reference_odom_error:="$PROJECTED_MAP_MAX_ODOM_ERROR"
  projected_map_max_reference_yaw_error:="$PROJECTED_MAP_MAX_YAW_ERROR"
  projected_map_reference_odom_timeout:="$PROJECTED_MAP_ODOM_TIMEOUT"
  local_costmap:="$LOCAL_COSTMAP"
  actor_costmap:="$ACTOR_COSTMAP"
  actor_obstacle_topic:="$ACTOR_OBSTACLE_TOPIC"
  actor_obstacle_frame:="$ACTOR_OBSTACLE_FRAME"
  actor_world_sdf_path:="$ACTOR_WORLD_SDF_PATH"
  actor_world_vehicle_name:="$ACTOR_WORLD_VEHICLE_NAME"
  actor_world_to_output:="$ACTOR_WORLD_TO_OUTPUT"
  actor_output_origin_xy:="$ACTOR_OUTPUT_ORIGIN_XY"
  actor_collision_monitor:="$ACTOR_COLLISION_MONITOR"
  actor_collision_event_topic:="$ACTOR_COLLISION_EVENT_TOPIC"
  actor_collision_near_margin:="$ACTOR_COLLISION_NEAR_MARGIN"
  trash_fusion_detection:="$TRASH_FUSION_DETECTION"
  trash_fusion_model_path:="$TRASH_FUSION_MODEL_PATH"
  trash_fusion_output_frame:="$TRASH_FUSION_OUTPUT_FRAME"
  trash_fusion_pointcloud_topic:="$TRASH_FUSION_POINTCLOUD_TOPIC"
  trash_fusion_image_topic:="$TRASH_FUSION_IMAGE_TOPIC"
  trash_fusion_camera_info_topic:="$TRASH_FUSION_CAMERA_INFO_TOPIC"
  front3d_source_topic:="$FRONT3D_SOURCE_TOPIC"
  simulated_lidar_output_topic:="$SIMULATED_FRONT3D_TOPIC"
  simulated_lidar_filtered_output_topic:="$SIMULATED_FILTERED_FRONT3D_TOPIC"
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
if [[ -n "$ACTORS_SCENARIO_PATH" ]]; then
  fast_lio_launch+=(actors_scenario_path:="$ACTORS_SCENARIO_PATH")
fi
if [[ -n "$ACTOR_COSTMAP_POSE_TOPICS" ]]; then
  fast_lio_launch+=(actor_pose_topics:="$ACTOR_COSTMAP_POSE_TOPICS")
fi
if [[ -n "$DYNAMIC_ACTOR_TOPICS" ]]; then
  fast_lio_launch+=(dynamic_actor_topics:="$DYNAMIC_ACTOR_TOPICS")
fi
if [[ -n "$DYNAMIC_VEHICLE_TOPICS" ]]; then
  fast_lio_launch+=(vehicle_pose_topics:="$DYNAMIC_VEHICLE_TOPICS")
fi
if [[ -n "$PROJECTED_MAP_REFERENCE_ODOM_TOPIC" ]]; then
  fast_lio_launch+=(projected_map_reference_odom_topic:="$PROJECTED_MAP_REFERENCE_ODOM_TOPIC")
fi
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
  append_launch_arg_if_non_empty trash_scenario_path "$TRASH_SCENARIO_PATH"
fi

start_process fast_lio_3d_slam "${fast_lio_launch[@]}"
FAST_LIO_PID="${PIDS[-1]}"

if [[ "$RVIZ" == "true" ]]; then
  start_optional_process rviz \
    ros2 launch gen0_main gen0_3d_rviz.launch.py \
      rviz:=true \
      rviz_config:="$RVIZ_CONFIG" \
      rviz_render_env:="$RVIZ_RENDER_ENV" \
      use_sim_time:=true \
      raw_front3d_input_topic:="$FRONT3D_SOURCE_TOPIC" \
      registered_preview_input_topic:="$SCURM_REGISTERED_SCAN_TOPIC"
fi

if [[ "$RELOCALIZATION" == "true" && "$MAPPING_DRIVE" == "true" ]]; then
  if wait_for_relocalized_odometry "$FAST_LIO_PID"; then
    start_process mapping_drive \
      ros2 launch gen0_main gen0_mapping_drive.launch.py \
        enabled:=true \
        drive_speed:="$DRIVE_SPEED" \
        front3d_topic:="$FRONT3D_SOURCE_TOPIC"
    start_trash_cleanup_if_enabled
  fi
elif [[ "$RELOCALIZATION" == "true" ]]; then
  log "Skipping relocalized odometry wait because GEN0_MAPPING_DRIVE=$MAPPING_DRIVE"
else
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

  start_trash_cleanup_if_enabled
fi

log "Stack is running. Press Ctrl+C in this terminal to stop everything launched by this script."
log "Preview topic: /gen0_mapping/rviz/fast_lio_map"
log "Trash fusion topics: /gen0_perception/trash_markers, /gen0_perception/trash_poses, /gen0_perception/trash_detections, /gen0_perception/trash_debug_image"
if [[ "$PROJECTED_MAP_BACKEND" == "octomap" ]]; then
  log "Projected map backend: SCURM octomap chain, topics: /projected_map and /projected_map_updates"
else
  log "Projected map backend: SCURM terrain projection, topics: /projected_map and /projected_costmap"
fi

while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    critical="${CRITICALS[$i]}"
    [[ -z "$pid" ]] && continue
    if ! kill -0 "$pid" 2>/dev/null; then
      set +e
      wait "$pid"
      status=$?
      set -e
      log "$name exited with status $status. Check $LOG_DIR/$name.log"
      if [[ "$critical" == "true" ]]; then
        exit "$status"
      fi
      PIDS[$i]=""
    fi
  done
  sleep 3
done
