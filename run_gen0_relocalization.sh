#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
WORLD="${GEN0_WORLD:-my_map}"
PARTITION="${GEN0_PARTITION:-gen0_relocalization}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
PRIOR_MAP_PATH="${GEN0_PRIOR_MAP_PATH:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/maps/prior_map.pcd}"

GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}"
GAZEBO_GUI="${GEN0_GAZEBO_GUI:-true}"
GAZEBO_RENDER_ENV="${GEN0_GAZEBO_RENDER_ENV:-unset}"
RVIZ="${GEN0_RVIZ:-true}"
RVIZ_RENDER_ENV="${GEN0_RVIZ_RENDER_ENV:-software}"
RVIZ_CONFIG="${GEN0_RVIZ_CONFIG:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_relocalization_loam_livox.rviz}"

NAV2="${GEN0_RELOCALIZATION_NAV2:-true}"
NAV2_DELAY="${GEN0_RELOCALIZATION_NAV2_DELAY:-10}"
NAV2_PROFILE="${GEN0_NAV2_PROFILE:-scurm_gen0}"
NAV2_RVIZ="${GEN0_NAV2_RVIZ:-true}"
NAV2_RVIZ_RENDER_ENV="${GEN0_NAV2_RVIZ_RENDER_ENV:-software}"
NAV2_USE_RESPAWN="${GEN0_NAV2_USE_RESPAWN:-false}"
NAV2_LOCALIZATION_MODE="${GEN0_NAV2_LOCALIZATION_MODE:-relocalized}"
NAV2_COSTMAP_SOURCE="${GEN0_NAV2_COSTMAP_SOURCE:-scurm_terrain}"
NAV2_MAP_SOURCE="${GEN0_NAV2_MAP_SOURCE:-yaml}"
DEFAULT_NAV2_MAP=""
if [[ "$WORLD" == "my_map" ]]; then
  DEFAULT_NAV2_MAP="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/maps/recovered_projected_20260807_102204.yaml"
fi
NAV2_MAP="${GEN0_NAV2_MAP:-$DEFAULT_NAV2_MAP}"
if [[ "$NAV2_MAP_SOURCE" != "yaml" && -z "${GEN0_NAV2_MAP:-}" ]]; then
  NAV2_MAP=""
fi
NAV2_REFERENCE_ODOM_TOPIC="${GEN0_NAV2_REFERENCE_ODOM_TOPIC:-}"
NAV2_ODOM_HEALTH_GUARD="${GEN0_NAV2_ODOM_HEALTH_GUARD:-false}"
AUTO_STOP="${GEN0_RELOCALIZATION_AUTO_STOP:-true}"

MAPPING_DRIVE="${GEN0_MAPPING_DRIVE:-false}"
TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-false}"
DEFAULT_TRASH_SCENARIO=""
DEFAULT_ACTORS_SCENARIO=""
if [[ "$WORLD" == "my_map" ]]; then
  DEFAULT_TRASH_SCENARIO="small_trash_dense"
  DEFAULT_ACTORS_SCENARIO="walking_actors3"
fi
TRASH_SCENARIO="${GEN0_TRASH_SCENARIO-$DEFAULT_TRASH_SCENARIO}"
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO-$DEFAULT_ACTORS_SCENARIO}"
PROJECTED_MAP_BACKEND="${GEN0_PROJECTED_MAP_BACKEND:-python}"
RELOCALIZATION_WAIT_TIMEOUT="${GEN0_RELOCALIZATION_WAIT_TIMEOUT:-0}"
TERRAIN_ANALYSIS="${GEN0_TERRAIN_ANALYSIS:-true}"
TERRAIN_ANALYSIS_EXT="${GEN0_TERRAIN_ANALYSIS_EXT:-true}"
PROJECTED_MAP="${GEN0_PROJECTED_MAP:-true}"
PROJECTED_MAP_ODOM_GUARD="${GEN0_PROJECTED_MAP_ODOM_GUARD:-false}"
LOCAL_COSTMAP="${GEN0_LOCAL_COSTMAP:-true}"
SIMULATED_LIDAR="${GEN0_SIMULATED_LIDAR:-true}"
STABLE_SIM_ODOM="${GEN0_STABLE_SIM_ODOM:-}"
STABLE_ODOM_TOPIC="${GEN0_STABLE_ODOM_TOPIC:-/gen0_mapping/stable_odom}"
FRONT3D_SOURCE_TOPIC="${GEN0_FRONT3D_SOURCE_TOPIC:-}"
ALLOW_LIVE_LIDAR_STARTUP="${GEN0_RELOCALIZATION_ALLOW_LIVE_LIDAR_STARTUP:-false}"
DYNAMIC_ACTOR_TOPICS="${GEN0_DYNAMIC_ACTOR_TOPICS:-}"
ACTOR_COSTMAP="${GEN0_ACTOR_COSTMAP:-true}"
ACTOR_OBSTACLE_FRAME="${GEN0_ACTOR_OBSTACLE_FRAME:-map}"
ACTOR_OBSTACLE_TOPIC="${GEN0_ACTOR_OBSTACLE_TOPIC:-}"
ACTOR_WORLD_SDF_PATH="${GEN0_ACTOR_WORLD_SDF_PATH:-}"
ACTOR_WORLD_VEHICLE_NAME="${GEN0_ACTOR_WORLD_VEHICLE_NAME:-}"
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

if [[ -z "$DYNAMIC_ACTOR_TOPICS" && -n "$ACTORS_SCENARIO" ]]; then
  for actor_index in {1..20}; do
    actor_topic="/actor/pedestrian_${actor_index}/pose"
    if [[ -n "$DYNAMIC_ACTOR_TOPICS" ]]; then
      DYNAMIC_ACTOR_TOPICS+=","
    fi
    DYNAMIC_ACTOR_TOPICS+="$actor_topic"
  done
fi

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

PIDS=()
NAMES=()
CRITICALS=()

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

require_file() {
  if [[ ! -e "$1" ]]; then
    printf 'Missing required file: %s\n' "$1" >&2
    exit 1
  fi
}

source_ros_setup() {
  set +u
  source "$1"
  set -u
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

wait_for_topic_once() {
  local topic="$1"
  local wait_timeout="$2"
  local description="$3"
  local deadline=$((SECONDS + wait_timeout))

  log "Waiting for $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    if timeout 5s ros2 topic echo --no-daemon --once "$topic" --field header >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_tf() {
  local target_frame="$1"
  local source_frame="$2"
  local wait_timeout="$3"
  local deadline=$((SECONDS + wait_timeout))
  local output

  log "Waiting for TF $target_frame -> $source_frame (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    output="$(timeout 5s ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null || true)"
    if [[ "$output" == *"Translation:"* ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
}

check_existing_processes() {
  if [[ "${GEN0_ALLOW_EXISTING_NODES:-false}" == "true" ]]; then
    return
  fi

  local existing
  existing="$(
    timeout 5s ros2 node list --no-daemon 2>/dev/null \
      | grep -E '(^/pose_publisher$|^/gen0_simulated_world_lidar$|^/gen0_gazebo_livox_adapter$|^/gen0_icp_transform_publisher$|^/gen0_icp_relocalization$|^/gen0_fast_lio$|^/map_server$|^/controller_server$|^/planner_server$|^/bt_navigator$|^/lifecycle_manager_navigation$|^/rviz$|^/rviz2$)' \
      || true
  )"
  if [[ -n "$existing" ]]; then
    printf 'Existing Gen0 relocalization/Nav2/RViz nodes are still running:\n%s\n\n' "$existing" >&2
    printf 'Run ./stop_gen0_3d_slam.sh first, or set GEN0_ALLOW_EXISTING_NODES=true to bypass this check.\n' >&2
    exit 1
  fi

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

trap cleanup EXIT INT TERM

cd "$WORKSPACE"
require_file /opt/ros/humble/setup.bash
require_file "$WORKSPACE/install/setup.bash"
require_file "$PRIOR_MAP_PATH"
require_file "$RVIZ_CONFIG"
if [[ "$AUTO_STOP" == "true" ]]; then
  require_file "$WORKSPACE/stop_gen0_3d_slam.sh"
  log "Stopping existing Gen0/ROS/Gazebo processes before relocalization startup"
  "$WORKSPACE/stop_gen0_3d_slam.sh" --quiet
fi

mkdir -p "$LOG_DIR"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR
export RCUTILS_LOGGING_BUFFERED_STREAM="${RCUTILS_LOGGING_BUFFERED_STREAM:-1}"
export PYTHONUNBUFFERED="${PYTHONUNBUFFERED:-1}"

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"
check_existing_processes

log "Workspace: $WORKSPACE"
log "World: $WORLD, partition=$PARTITION, prior_map=$PRIOR_MAP_PATH"
log "Gazebo: gui=$GAZEBO_GUI, render_env=$GAZEBO_RENDER_ENV, gpu_adapter=$GPU_ADAPTER"
log "Relocalization RViz: enabled=$RVIZ, config=$RVIZ_CONFIG, render_env=$RVIZ_RENDER_ENV"
if [[ "$SIMULATED_LIDAR" != "true" && "$ALLOW_LIVE_LIDAR_STARTUP" != "true" ]]; then
  log "Live 3D lidar was requested for startup, but stable relocalization startup uses the simulated lidar source. Set GEN0_RELOCALIZATION_ALLOW_LIVE_LIDAR_STARTUP=true to force live startup."
  SIMULATED_LIDAR=true
  FRONT3D_SOURCE_TOPIC=""
fi

if [[ -z "$STABLE_SIM_ODOM" ]]; then
  if [[ "$SIMULATED_LIDAR" == "true" ]]; then
    STABLE_SIM_ODOM="true"
  else
    STABLE_SIM_ODOM="false"
  fi
fi

if [[ -n "${GEN0_NAV2_ODOM_TOPIC:-}" ]]; then
  NAV2_ODOM_TOPIC="$GEN0_NAV2_ODOM_TOPIC"
elif [[ "$STABLE_SIM_ODOM" == "true" ]]; then
  NAV2_ODOM_TOPIC="$STABLE_ODOM_TOPIC"
else
  NAV2_ODOM_TOPIC="/gen0_mapping/fast_lio/odom"
fi

log "3D lidar source: simulated_lidar=$SIMULATED_LIDAR, front3d_source=${FRONT3D_SOURCE_TOPIC:-auto}, actors_scenario=${ACTORS_SCENARIO:-none}, trash_scenario=${TRASH_SCENARIO:-none}"
log "SCURM dynamic mapping: terrain_analysis=$TERRAIN_ANALYSIS, terrain_analysis_ext=$TERRAIN_ANALYSIS_EXT, projected_map=$PROJECTED_MAP, projected_map_odom_guard=$PROJECTED_MAP_ODOM_GUARD, local_costmap=$LOCAL_COSTMAP"
log "Simulation odometry isolation: stable_sim_odom=$STABLE_SIM_ODOM, stable_odom_topic=$STABLE_ODOM_TOPIC, nav2_odom_topic=$NAV2_ODOM_TOPIC"
log "Actor costmap: enabled=${ACTOR_COSTMAP:-auto}, frame=${ACTOR_OBSTACLE_FRAME:-auto}, topic=${ACTOR_OBSTACLE_TOPIC:-auto}, world_to_output=${ACTOR_WORLD_TO_OUTPUT:-auto}"
log "Nav2: enabled=$NAV2, delay=${NAV2_DELAY}s, profile=$NAV2_PROFILE, rviz=$NAV2_RVIZ, costmap_source=$NAV2_COSTMAP_SOURCE, map_source=$NAV2_MAP_SOURCE, map=${NAV2_MAP:-auto}"
log "Logs: $LOG_DIR"

start_process relocalization_3d_slam \
  env \
    GEN0_WORKSPACE="$WORKSPACE" \
    GEN0_WORLD="$WORLD" \
    GEN0_PARTITION="$PARTITION" \
    GEN0_PRIOR_MAP_PATH="$PRIOR_MAP_PATH" \
    GEN0_RELOCALIZATION=true \
    GEN0_RELOCALIZATION_WAIT_TIMEOUT="$RELOCALIZATION_WAIT_TIMEOUT" \
    GEN0_RELOCALIZATION_INITIAL_X="$RELOCALIZATION_INITIAL_X" \
    GEN0_RELOCALIZATION_INITIAL_Y="$RELOCALIZATION_INITIAL_Y" \
    GEN0_RELOCALIZATION_INITIAL_Z="$RELOCALIZATION_INITIAL_Z" \
    GEN0_RELOCALIZATION_INITIAL_A="$RELOCALIZATION_INITIAL_A" \
    GEN0_RELOCALIZATION_FITNESS_SCORE_THRESHOLD="$RELOCALIZATION_FITNESS_SCORE_THRESHOLD" \
    GEN0_RELOCALIZATION_CONVERGED_COUNT_THRESHOLD="$RELOCALIZATION_CONVERGED_COUNT_THRESHOLD" \
    GEN0_RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE="$RELOCALIZATION_MAX_CORRESPONDENCE_DISTANCE" \
    GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_X="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_X" \
    GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_Y" \
    GEN0_RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z="$RELOCALIZATION_INPUT_CLOUD_TO_BASE_Z" \
    GEN0_RELOCALIZATION_LEGACY_LIVOX_ROLL_180="$RELOCALIZATION_LEGACY_LIVOX_ROLL_180" \
    GEN0_STABLE_SIM_ODOM="$STABLE_SIM_ODOM" \
    GEN0_STABLE_ODOM_TOPIC="$STABLE_ODOM_TOPIC" \
    GEN0_GPU_ADAPTER="$GPU_ADAPTER" \
    GEN0_GAZEBO_GUI="$GAZEBO_GUI" \
    GEN0_GAZEBO_RENDER_ENV="$GAZEBO_RENDER_ENV" \
    GEN0_RVIZ="$RVIZ" \
    GEN0_RVIZ_CONFIG="$RVIZ_CONFIG" \
    GEN0_RVIZ_RENDER_ENV="$RVIZ_RENDER_ENV" \
    GEN0_MAPPING_DRIVE="$MAPPING_DRIVE" \
    GEN0_TRASH_SCENARIO="$TRASH_SCENARIO" \
    GEN0_TRASH_CLEANUP="$TRASH_CLEANUP" \
    GEN0_ACTORS_SCENARIO="$ACTORS_SCENARIO" \
    GEN0_DYNAMIC_ACTOR_TOPICS="$DYNAMIC_ACTOR_TOPICS" \
    GEN0_TRASH_SCENARIO_PATH="$TRASH_SCENARIO_PATH" \
    GEN0_SIMULATED_LIDAR="$SIMULATED_LIDAR" \
    GEN0_FRONT3D_SOURCE_TOPIC="$FRONT3D_SOURCE_TOPIC" \
    GEN0_TERRAIN_ANALYSIS="$TERRAIN_ANALYSIS" \
    GEN0_TERRAIN_ANALYSIS_EXT="$TERRAIN_ANALYSIS_EXT" \
    GEN0_PROJECTED_MAP="$PROJECTED_MAP" \
    GEN0_PROJECTED_MAP_BACKEND="$PROJECTED_MAP_BACKEND" \
    GEN0_PROJECTED_MAP_ODOM_GUARD="$PROJECTED_MAP_ODOM_GUARD" \
    GEN0_LOCAL_COSTMAP="$LOCAL_COSTMAP" \
    GEN0_ACTOR_COSTMAP="$ACTOR_COSTMAP" \
    GEN0_ACTOR_OBSTACLE_FRAME="$ACTOR_OBSTACLE_FRAME" \
    GEN0_ACTOR_OBSTACLE_TOPIC="$ACTOR_OBSTACLE_TOPIC" \
    GEN0_ACTOR_WORLD_SDF_PATH="$ACTOR_WORLD_SDF_PATH" \
    GEN0_ACTOR_WORLD_VEHICLE_NAME="$ACTOR_WORLD_VEHICLE_NAME" \
    GEN0_ACTOR_WORLD_TO_OUTPUT="$ACTOR_WORLD_TO_OUTPUT" \
    GEN0_ACTOR_OUTPUT_ORIGIN_XY="$ACTOR_OUTPUT_ORIGIN_XY" \
    GEN0_ACTOR_COLLISION_MONITOR="$ACTOR_COLLISION_MONITOR" \
    GEN0_ACTOR_COLLISION_EVENT_TOPIC="$ACTOR_COLLISION_EVENT_TOPIC" \
    GEN0_ACTOR_COLLISION_NEAR_MARGIN="$ACTOR_COLLISION_NEAR_MARGIN" \
    GEN0_ACTOR_SOFT_STOP="$ACTOR_SOFT_STOP" \
    GEN0_ACTOR_SOFT_STOP_MARGIN="$ACTOR_SOFT_STOP_MARGIN" \
    GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN="$ACTOR_SOFT_STOP_RELEASE_MARGIN" \
    GEN0_ACTOR_SOFT_STOP_VEHICLE_NAME="$ACTOR_SOFT_STOP_VEHICLE_NAME" \
    "$WORKSPACE/run_gen0_3d_slam.sh"

if [[ "$NAV2" == "true" ]]; then
  sleep "$NAV2_DELAY"

  if ! wait_for_topic_once "$NAV2_ODOM_TOPIC" "${GEN0_RELOCALIZATION_ODOM_WAIT_TIMEOUT:-90}" "navigation odometry"; then
    log "Nav2 was not started because $NAV2_ODOM_TOPIC is not available. Check $LOG_DIR/relocalization_3d_slam.log."
  elif ! wait_for_tf map base_link "${GEN0_RELOCALIZATION_MAP_TF_WAIT_TIMEOUT:-60}"; then
    log "Nav2 was not started because TF map -> base_link is not available. Check ICP convergence in $LOG_DIR/relocalization_3d_slam.log."
  else
    nav2_env=(
      GEN0_WORKSPACE="$WORKSPACE"
      GEN0_WORLD="$WORLD"
      GEN0_NAV2_PROFILE="$NAV2_PROFILE"
      GEN0_NAV2_LOCALIZATION_MODE="$NAV2_LOCALIZATION_MODE"
      GEN0_NAV2_ODOM_TOPIC="$NAV2_ODOM_TOPIC"
      GEN0_NAV2_USE_RESPAWN="$NAV2_USE_RESPAWN"
      GEN0_NAV2_RVIZ="$NAV2_RVIZ"
      GEN0_NAV2_RVIZ_RENDER_ENV="$NAV2_RVIZ_RENDER_ENV"
      GEN0_NAV2_COSTMAP_SOURCE="$NAV2_COSTMAP_SOURCE"
      GEN0_NAV2_MAP_SOURCE="$NAV2_MAP_SOURCE"
      GEN0_NAV2_ODOM_HEALTH_GUARD="$NAV2_ODOM_HEALTH_GUARD"
    )
    if [[ -n "$NAV2_REFERENCE_ODOM_TOPIC" ]]; then
      nav2_env+=(GEN0_NAV2_REFERENCE_ODOM_TOPIC="$NAV2_REFERENCE_ODOM_TOPIC")
    fi
    if [[ -n "$NAV2_MAP" ]]; then
      nav2_env+=(GEN0_NAV2_MAP="$NAV2_MAP")
    fi

    start_process nav2_costmap_view \
      env "${nav2_env[@]}" "$WORKSPACE/run_gen0_nav2.sh"
  fi
fi

log "Relocalization stack is running. Press Ctrl+C in this terminal to stop everything launched by this script."

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
