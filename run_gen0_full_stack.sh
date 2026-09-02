#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"

WORLD="${GEN0_WORLD:-my_map}"
DEFAULT_ACTORS_SCENARIO=""
DEFAULT_TRASH_SCENARIO=""
DEFAULT_NAV2_MAP=""
if [[ "$WORLD" == "my_map" ]]; then
  DEFAULT_ACTORS_SCENARIO="walking_actors3"
  DEFAULT_TRASH_SCENARIO="small_trash_dense"
  DEFAULT_NAV2_MAP="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/maps/recovered_projected_20260807_102204.yaml"
fi

ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO-$DEFAULT_ACTORS_SCENARIO}"
TRASH_SCENARIO="${GEN0_TRASH_SCENARIO-$DEFAULT_TRASH_SCENARIO}"

START_BASE_STACK="${GEN0_START_BASE_STACK:-true}"
START_NAV2="${GEN0_START_NAV2:-true}"
START_EPSILON="${GEN0_START_EPSILON:-true}"

ODOM_TOPIC="${GEN0_FULL_STACK_ODOM_TOPIC:-/gen0_mapping/stable_odom}"
COSTMAP_TOPIC="${GEN0_FULL_STACK_COSTMAP_TOPIC:-/projected_costmap}"
PATH_TOPIC="${GEN0_FULL_STACK_PATH_TOPIC:-/plan_smoothed}"

NAV2_PARAMS_FILE="${GEN0_EPSILON_NAV2_PARAMS_FILE:-$WORKSPACE/epsilon_migration/epsilon_planning/config/nav2_gen0_scurm_epsilon_params.yaml}"
NAV2_TO_POSE_BT="${GEN0_EPSILON_NAV2_TO_POSE_BT:-$WORKSPACE/epsilon_migration/epsilon_planning/behavior_tree/ackermann_scurm_epsilon_recovery.xml}"
NAV2_THROUGH_POSES_BT="${GEN0_EPSILON_NAV2_THROUGH_POSES_BT:-$WORKSPACE/epsilon_migration/epsilon_planning/behavior_tree/ackermann_scurm_epsilon_through_poses.xml}"
NAV2_MAP="${GEN0_NAV2_MAP:-$DEFAULT_NAV2_MAP}"
NAV2_COSTMAP_SOURCE="${GEN0_NAV2_COSTMAP_SOURCE:-scurm_terrain}"
NAV2_MAP_SOURCE="${GEN0_NAV2_MAP_SOURCE:-yaml}"
NAV2_USE_RESPAWN="${GEN0_NAV2_USE_RESPAWN:-false}"
NAV2_RVIZ="${GEN0_NAV2_RVIZ:-true}"
NAV2_RVIZ_RENDER_ENV="${GEN0_NAV2_RVIZ_RENDER_ENV:-software}"
NAV2_CONTROLLER_FREQUENCY="${GEN0_NAV2_CONTROLLER_FREQUENCY:-10.0}"
NAV2_SMOOTHING_FREQUENCY="${GEN0_NAV2_SMOOTHING_FREQUENCY:-$NAV2_CONTROLLER_FREQUENCY}"
if [[ -n "${GEN0_NAV2_MODEL_DT:-}" ]]; then
  NAV2_MODEL_DT="$GEN0_NAV2_MODEL_DT"
else
  NAV2_MODEL_DT="$(awk -v frequency="$NAV2_CONTROLLER_FREQUENCY" 'BEGIN { if (frequency > 0.0) printf "%.6f", 1.0 / frequency; else print "0.050000" }')"
fi

NAV2_RAW_CMD_VEL_TOPIC="${GEN0_NAV2_RAW_CMD_VEL_TOPIC:-/control/nav2_cmd_vel_raw}"
if [[ "$START_EPSILON" != "true" && -z "${GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC:-}" ]]; then
  # Without EPSILON there is no mux, so Nav2 must feed the pose guard directly.
  GUARDED_CMD_VEL_TOPIC="$NAV2_RAW_CMD_VEL_TOPIC"
else
  GUARDED_CMD_VEL_TOPIC="${GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC:-/control/cmd_vel_raw}"
fi
FINAL_CMD_VEL_TOPIC="${GEN0_FINAL_CMD_VEL_TOPIC:-/cmd_vel}"
# Full-stack Nav2 already consumes the normalized stable odometry topic.  Do
# not compare it with raw /odom by default: stable_odom intentionally resets
# the simulation origin, so the two streams differ by the Gazebo spawn pose.
NAV2_REFERENCE_ODOM_TOPIC="${GEN0_NAV2_REFERENCE_ODOM_TOPIC:-}"
NAV2_MAX_REFERENCE_ODOM_ERROR="${GEN0_NAV2_MAX_REFERENCE_ODOM_ERROR:-0.0}"
NAV2_MAX_REFERENCE_YAW_ERROR="${GEN0_NAV2_MAX_REFERENCE_YAW_ERROR:-0.0}"
NAV2_REFERENCE_ODOM_TIMEOUT="${GEN0_NAV2_REFERENCE_ODOM_TIMEOUT:-2.0}"

EPSILON_CMD_VEL_TOPIC="${GEN0_EPSILON_CMD_VEL_TOPIC:-/epsilon/cmd_vel_raw}"
EPSILON_CONTROL_SOURCE="${GEN0_EPSILON_CONTROL_SOURCE:-auto}"
EPSILON_CONTROL_MODE_TOPIC="${GEN0_EPSILON_CONTROL_MODE_TOPIC:-/epsilon/control_mode}"
EPSILON_SELECTED_SOURCE_TOPIC="${GEN0_EPSILON_SELECTED_SOURCE_TOPIC:-/epsilon/selected_control_source}"
EPSILON_STATUS_TOPIC="${GEN0_EPSILON_STATUS_TOPIC:-/epsilon/status}"
EPSILON_STATUS_TIMEOUT="${GEN0_EPSILON_STATUS_TIMEOUT:-0.6}"
EPSILON_FALLBACK_TO_NAV2="${GEN0_EPSILON_FALLBACK_TO_NAV2:-true}"
EPSILON_USE_SIM_TIME="${GEN0_EPSILON_USE_SIM_TIME:-true}"
EPSILON_PREDICTION_RATE="${GEN0_EPSILON_PREDICTION_RATE:-5.0}"
EPSILON_DYNAMIC_PUBLISH_PERIOD="${GEN0_EPSILON_DYNAMIC_PUBLISH_PERIOD:-0.1}"
EPSILON_PATH_TIMEOUT="${GEN0_EPSILON_PATH_TIMEOUT:-30.0}"
EPSILON_REQUIRE_PATH="${GEN0_EPSILON_REQUIRE_PATH:-false}"
USE_QCNET_PREDICTION="${GEN0_EPSILON_USE_QCNET_PREDICTION:-true}"
QCNET_BACKEND="${GEN0_QCNET_BACKEND:-qcnet}"
QCNET_ROOT="${GEN0_QCNET_ROOT:-/home/zjxue2007/QCNet}"
QCNET_CKPT_PATH="${GEN0_QCNET_CKPT_PATH:-/home/zjxue2007/QCNet_AV2.ckpt}"
QCNET_DEVICE="${GEN0_QCNET_DEVICE:-cuda}"
ACTOR_SOURCE="${GEN0_EPSILON_ACTOR_SOURCE:-scenario}"
ACTORS_SCENARIO_PATH=""
if [[ -n "$ACTORS_SCENARIO" ]]; then
  ACTORS_SCENARIO_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/scenarios/$WORLD/$ACTORS_SCENARIO.sdf"
fi
ACTOR_WORLD_SDF_PATH="${GEN0_ACTOR_WORLD_SDF_PATH:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.sdf}"
ACTOR_WORLD_VEHICLE_NAME="${GEN0_ACTOR_WORLD_VEHICLE_NAME:-gen0_model}"
ACTOR_WORLD_TO_OUTPUT="${GEN0_ACTOR_WORLD_TO_OUTPUT:-true}"
ACTOR_OUTPUT_ORIGIN_XY="${GEN0_ACTOR_OUTPUT_ORIGIN_XY:-0.0,0.0}"
SCENARIO_ACTOR_TOPIC_PREFIX="${GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_PREFIX:-/epsilon/scenario_actor}"
SCENARIO_ACTOR_PUBLISH_RATE="${GEN0_EPSILON_SCENARIO_ACTOR_PUBLISH_RATE:-10.0}"
BASE_DYNAMIC_ACTOR_TOPICS="${GEN0_FULL_STACK_BASE_DYNAMIC_ACTOR_TOPICS:-${GEN0_DYNAMIC_ACTOR_TOPICS:-}}"
BASE_ACTOR_COSTMAP_POSE_TOPICS="${GEN0_FULL_STACK_ACTOR_COSTMAP_POSE_TOPICS:-${GEN0_ACTOR_COSTMAP_POSE_TOPICS:-}}"
EPSILON_DYNAMIC_ACTOR_TOPICS="${GEN0_DYNAMIC_ACTOR_TOPICS:-}"
EPSILON_ACTOR_POSE_TOPICS="${GEN0_EPSILON_ACTOR_POSE_TOPICS:-}"
if [[ "$ACTOR_SOURCE" == "scenario" && -z "${GEN0_FULL_STACK_BASE_DYNAMIC_ACTOR_TOPICS+x}" ]]; then
  # Do not duplicate actors into the high-volume simulated lidar. Nav2 receives
  # their live poses through the lightweight actor costmap below.
  BASE_DYNAMIC_ACTOR_TOPICS=""
fi
if [[ "$ACTOR_SOURCE" == "scenario" && -z "${GEN0_FULL_STACK_ACTOR_COSTMAP_POSE_TOPICS+x}" ]]; then
  BASE_ACTOR_COSTMAP_POSE_TOPICS=""
  for actor_index in {1..20}; do
    actor_topic="/actor/pedestrian_${actor_index}/pose"
    if [[ -n "$BASE_ACTOR_COSTMAP_POSE_TOPICS" ]]; then
      BASE_ACTOR_COSTMAP_POSE_TOPICS+=","
    fi
    BASE_ACTOR_COSTMAP_POSE_TOPICS+="$actor_topic"
  done
fi
if [[ "$ACTOR_SOURCE" == "scenario" ]]; then
  EPSILON_DYNAMIC_ACTOR_TOPICS=""
  EPSILON_ACTOR_POSE_TOPICS=""
fi

ODOM_TIMEOUT="${GEN0_FULL_STACK_ODOM_TIMEOUT:-240}"
COSTMAP_TIMEOUT="${GEN0_FULL_STACK_COSTMAP_TIMEOUT:-180}"
MAP_TF_TIMEOUT="${GEN0_FULL_STACK_MAP_TF_TIMEOUT:-90}"
PROBE_TIMEOUT="${GEN0_FULL_STACK_PROBE_TIMEOUT:-5}"

PIDS=()
NAMES=()

mkdir -p "$LOG_DIR" "$ROS_LOG_DIR"
export ROS_LOG_DIR

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

require_file() {
  if [[ ! -f "$1" ]]; then
    printf 'Required file not found: %s\n' "$1" >&2
    exit 1
  fi
}

check_qcnet_cuda_ready() {
  if [[ "$START_EPSILON" != "true" || "$USE_QCNET_PREDICTION" != "true" || "$QCNET_BACKEND" != "qcnet" ]]; then
    return 0
  fi
  case "${QCNET_DEVICE,,}" in
    cuda|cuda:*) ;;
    *) return 0 ;;
  esac

  python3 - "$QCNET_DEVICE" <<'PY'
import sys

device = sys.argv[1]
try:
    import torch
except Exception as exc:
    print("PyTorch import failed: %s" % exc, file=sys.stderr)
    sys.exit(1)

print(
    "QCNet CUDA preflight: torch=%s torch_cuda=%s cuda_available=%s device_count=%s requested=%s"
    % (torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.device_count(), device),
    file=sys.stderr,
)
sys.exit(0 if torch.cuda.is_available() else 1)
PY
}

wait_for_topic_once() {
  local topic="$1"
  local wait_timeout="$2"
  local description="$3"
  local deadline=$((SECONDS + wait_timeout))
  local remaining
  local current_timeout

  log "Waiting for $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    remaining=$((deadline - SECONDS))
    current_timeout="$PROBE_TIMEOUT"
    if ((remaining < current_timeout)); then
      current_timeout="$remaining"
    fi
    if ((current_timeout < 1)); then
      current_timeout=1
    fi
    if timeout --kill-after=1s "$current_timeout" ros2 topic echo --no-daemon --once "$topic" --field header >/dev/null 2>&1; then
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
    output="$(timeout "$PROBE_TIMEOUT" ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null || true)"
    if [[ "$output" == *"Translation:"* ]]; then
      return 0
    fi
    sleep 1
  done
  return 1
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

start_script() {
  local name="$1"
  shift
  log "Starting $name"
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("$name")
  log "$name pid=${PIDS[-1]} log=$LOG_DIR/$name.log"
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

require_file /opt/ros/humble/setup.bash
require_file "$WORKSPACE/install/setup.bash"
require_file "$WORKSPACE/run_gen0_relocalization.sh"
require_file "$WORKSPACE/run_gen0_epsilon.sh"
require_file "$NAV2_PARAMS_FILE"
require_file "$NAV2_TO_POSE_BT"
require_file "$NAV2_THROUGH_POSES_BT"
if [[ "$NAV2_MAP_SOURCE" == "yaml" ]]; then
  require_file "$NAV2_MAP"
fi

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"

for package_name in epsilon_planning qcnet_prediction gen0_nav2_path_exporter; do
  if ! ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    printf 'ROS package %s is not in the sourced workspace.\n' "$package_name" >&2
    printf 'Build first: colcon build --symlink-install --packages-select vehicle_msgs epsilon_core qcnet_prediction epsilon_planning gen0_nav2_path_exporter\n' >&2
    exit 1
  fi
done

if [[ "$START_EPSILON" == "true" && "$NAV2_RAW_CMD_VEL_TOPIC" == "$GUARDED_CMD_VEL_TOPIC" ]]; then
  printf 'GEN0_NAV2_RAW_CMD_VEL_TOPIC and GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC must differ.\n' >&2
  exit 1
fi

case "$ACTOR_SOURCE" in
  scenario|live|none) ;;
  *)
    printf 'Invalid GEN0_EPSILON_ACTOR_SOURCE=%s. Use scenario, live, or none.\n' "$ACTOR_SOURCE" >&2
    exit 1
    ;;
esac

if [[ "$ACTOR_SOURCE" == "scenario" && -n "$ACTORS_SCENARIO" ]]; then
  require_file "$ACTORS_SCENARIO_PATH"
  require_file "$ACTOR_WORLD_SDF_PATH"
fi
if [[ "$QCNET_BACKEND" == "qcnet" ]]; then
  require_file "$QCNET_CKPT_PATH"
  if [[ ! -d "$QCNET_ROOT" ]]; then
    printf 'Required QCNet root directory not found: %s\n' "$QCNET_ROOT" >&2
    exit 1
  fi
fi
if ! check_qcnet_cuda_ready; then
  printf 'QCNet is configured as backend=qcnet device=%s, but this Python environment has no CUDA-enabled PyTorch.\n' "$QCNET_DEVICE" >&2
  printf 'Install CUDA-enabled PyTorch for QCNet GPU, or temporarily run with GEN0_QCNET_BACKEND=constant_velocity to validate the rest of the stack.\n' >&2
  exit 1
fi

log "Workspace: $WORKSPACE"
log "World: $WORLD, base=relocalization, actors_scenario=${ACTORS_SCENARIO:-none}, trash_scenario=${TRASH_SCENARIO:-none}"
log "Nav2 sidecar: params=$NAV2_PARAMS_FILE, path_topic=$PATH_TOPIC, nav2_raw=$NAV2_RAW_CMD_VEL_TOPIC, controller_frequency=${NAV2_CONTROLLER_FREQUENCY}Hz, rviz=$NAV2_RVIZ"
log "Nav2 pose guard: reference=$NAV2_REFERENCE_ODOM_TOPIC, max_xy_error=$NAV2_MAX_REFERENCE_ODOM_ERROR, max_yaw_error=$NAV2_MAX_REFERENCE_YAW_ERROR"
log "Actor source: $ACTOR_SOURCE, scenario_path=${ACTORS_SCENARIO_PATH:-none}, base_dynamic_topics=${BASE_DYNAMIC_ACTOR_TOPICS:-none}"
log "EPSILON sidecar: control_source=$EPSILON_CONTROL_SOURCE, epsilon_raw=$EPSILON_CMD_VEL_TOPIC, mux_output=$GUARDED_CMD_VEL_TOPIC, final=$FINAL_CMD_VEL_TOPIC, selected_topic=$EPSILON_SELECTED_SOURCE_TOPIC, qcnet_backend=$QCNET_BACKEND, qcnet_device=$QCNET_DEVICE"
log "Logs: $LOG_DIR"
log "ROS logs: $ROS_LOG_DIR"

if [[ "$START_BASE_STACK" == "true" ]]; then
  start_script \
    relocalization_stack \
    env \
      GEN0_WORKSPACE="$WORKSPACE" \
      GEN0_WORLD="$WORLD" \
      GEN0_ACTORS_SCENARIO="$ACTORS_SCENARIO" \
      GEN0_DYNAMIC_ACTOR_TOPICS="$BASE_DYNAMIC_ACTOR_TOPICS" \
      GEN0_ACTOR_COSTMAP_POSE_TOPICS="$BASE_ACTOR_COSTMAP_POSE_TOPICS" \
      GEN0_ACTOR_WORLD_SDF_PATH="$ACTOR_WORLD_SDF_PATH" \
      GEN0_ACTOR_WORLD_VEHICLE_NAME="$ACTOR_WORLD_VEHICLE_NAME" \
      GEN0_ACTOR_WORLD_TO_OUTPUT="$ACTOR_WORLD_TO_OUTPUT" \
      GEN0_ACTOR_OUTPUT_ORIGIN_XY="$ACTOR_OUTPUT_ORIGIN_XY" \
      GEN0_TRASH_SCENARIO="$TRASH_SCENARIO" \
      GEN0_RELOCALIZATION_NAV2=false \
      "$WORKSPACE/run_gen0_relocalization.sh"
fi

if ! wait_for_topic_once "$ODOM_TOPIC" "$ODOM_TIMEOUT" "stable odometry"; then
  printf 'Timed out waiting for %s from the relocalization base stack.\n' "$ODOM_TOPIC" >&2
  exit 1
fi

if ! wait_for_topic_once "$COSTMAP_TOPIC" "$COSTMAP_TIMEOUT" "projected costmap"; then
  printf 'Timed out waiting for %s from the relocalization base stack.\n' "$COSTMAP_TOPIC" >&2
  exit 1
fi

if ! wait_for_tf map base_link "$MAP_TF_TIMEOUT"; then
  printf 'Timed out waiting for TF map -> base_link. Nav2 will reject goals until relocalization converges.\n' >&2
  exit 1
fi

if [[ "$START_EPSILON" == "true" ]]; then
  start_script \
    epsilon \
    env \
      GEN0_WORKSPACE="$WORKSPACE" \
      GEN0_WORLD="$WORLD" \
      GEN0_ACTORS_SCENARIO="$ACTORS_SCENARIO" \
      GEN0_EPSILON_ACTOR_SOURCE="$ACTOR_SOURCE" \
      GEN0_EPSILON_ACTOR_POSE_TOPICS="$EPSILON_ACTOR_POSE_TOPICS" \
      GEN0_DYNAMIC_ACTOR_TOPICS="$EPSILON_DYNAMIC_ACTOR_TOPICS" \
      GEN0_EPSILON_ACTORS_SCENARIO_PATH="$ACTORS_SCENARIO_PATH" \
      GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_PREFIX="$SCENARIO_ACTOR_TOPIC_PREFIX" \
      GEN0_EPSILON_SCENARIO_ACTOR_PUBLISH_RATE="$SCENARIO_ACTOR_PUBLISH_RATE" \
      GEN0_EPSILON_ACTOR_WORLD_SDF_PATH="$ACTOR_WORLD_SDF_PATH" \
      GEN0_EPSILON_ACTOR_WORLD_VEHICLE_NAME="$ACTOR_WORLD_VEHICLE_NAME" \
      GEN0_EPSILON_ACTOR_WORLD_TO_OUTPUT="$ACTOR_WORLD_TO_OUTPUT" \
      GEN0_EPSILON_ACTOR_OUTPUT_ORIGIN_XY="$ACTOR_OUTPUT_ORIGIN_XY" \
      GEN0_EPSILON_ODOM_TOPIC="$ODOM_TOPIC" \
      GEN0_EPSILON_COSTMAP_TOPIC="$COSTMAP_TOPIC" \
      GEN0_EPSILON_PATH_TOPIC="$PATH_TOPIC" \
      GEN0_EPSILON_REQUIRE_PATH="$EPSILON_REQUIRE_PATH" \
      GEN0_EPSILON_USE_SIM_TIME="$EPSILON_USE_SIM_TIME" \
      GEN0_EPSILON_PREDICTION_RATE="$EPSILON_PREDICTION_RATE" \
      GEN0_EPSILON_DYNAMIC_PUBLISH_PERIOD="$EPSILON_DYNAMIC_PUBLISH_PERIOD" \
      GEN0_EPSILON_PATH_TIMEOUT="$EPSILON_PATH_TIMEOUT" \
      GEN0_EPSILON_CMD_VEL_TOPIC="$EPSILON_CMD_VEL_TOPIC" \
      GEN0_EPSILON_USE_CMD_VEL_MUX=true \
      GEN0_EPSILON_CONTROL_SOURCE="$EPSILON_CONTROL_SOURCE" \
      GEN0_NAV2_RAW_CMD_VEL_TOPIC="$NAV2_RAW_CMD_VEL_TOPIC" \
      GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC="$GUARDED_CMD_VEL_TOPIC" \
      GEN0_EPSILON_CONTROL_MODE_TOPIC="$EPSILON_CONTROL_MODE_TOPIC" \
      GEN0_EPSILON_SELECTED_SOURCE_TOPIC="$EPSILON_SELECTED_SOURCE_TOPIC" \
      GEN0_EPSILON_STATUS_TOPIC="$EPSILON_STATUS_TOPIC" \
      GEN0_EPSILON_STATUS_TIMEOUT="$EPSILON_STATUS_TIMEOUT" \
      GEN0_EPSILON_FALLBACK_TO_NAV2="$EPSILON_FALLBACK_TO_NAV2" \
      GEN0_QCNET_BACKEND="$QCNET_BACKEND" \
      GEN0_QCNET_ROOT="$QCNET_ROOT" \
      GEN0_QCNET_CKPT_PATH="$QCNET_CKPT_PATH" \
      GEN0_QCNET_DEVICE="$QCNET_DEVICE" \
      "$WORKSPACE/run_gen0_epsilon.sh"
fi

if [[ "$START_NAV2" == "true" ]]; then
  start_script \
    nav2_epsilon \
    ros2 launch epsilon_planning gen0_navigation_epsilon.launch.py \
      use_sim_time:=true \
      params_file:="$NAV2_PARAMS_FILE" \
      use_respawn:="$NAV2_USE_RESPAWN" \
      nav2_controller_frequency:="$NAV2_CONTROLLER_FREQUENCY" \
      nav2_model_dt:="$NAV2_MODEL_DT" \
      nav2_smoothing_frequency:="$NAV2_SMOOTHING_FREQUENCY" \
      nav2_cmd_vel_topic:="$NAV2_RAW_CMD_VEL_TOPIC" \
      guarded_cmd_vel_topic:="$GUARDED_CMD_VEL_TOPIC" \
      final_cmd_vel_topic:="$FINAL_CMD_VEL_TOPIC" \
      actor_obstacle_topic:="/gen0_mapping/actor_obstacles" \
      publish_identity_map_to_odom:=false \
      costmap_source:="$NAV2_COSTMAP_SOURCE" \
      map_source:="$NAV2_MAP_SOURCE" \
      map_server_topic:=map \
      odom_topic:="$ODOM_TOPIC" \
      reference_odom_topic:="$NAV2_REFERENCE_ODOM_TOPIC" \
      max_reference_odom_error:="$NAV2_MAX_REFERENCE_ODOM_ERROR" \
      max_reference_yaw_error:="$NAV2_MAX_REFERENCE_YAW_ERROR" \
      reference_odom_timeout:="$NAV2_REFERENCE_ODOM_TIMEOUT" \
      rviz:="$NAV2_RVIZ" \
      rviz_render_env:="$NAV2_RVIZ_RENDER_ENV" \
      default_nav_to_pose_bt_xml:="$NAV2_TO_POSE_BT" \
      default_nav_through_poses_bt_xml:="$NAV2_THROUGH_POSES_BT" \
      map:="$NAV2_MAP"
fi

log "Full stack is running. Send a Nav2 goal; EPSILON receives /plan_smoothed and muxes final control through $GUARDED_CMD_VEL_TOPIC -> $FINAL_CMD_VEL_TOPIC."

while true; do
  for i in "${!PIDS[@]}"; do
    pid="${PIDS[$i]}"
    name="${NAMES[$i]}"
    [[ -z "$pid" ]] && continue
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
