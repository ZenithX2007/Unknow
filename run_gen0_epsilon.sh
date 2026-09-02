#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
WORLD="${GEN0_WORLD:-my_map}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"

ODOM_TOPIC="${GEN0_EPSILON_ODOM_TOPIC:-/gen0_mapping/stable_odom}"
PATH_TOPIC="${GEN0_EPSILON_PATH_TOPIC:-/plan_smoothed}"
COSTMAP_TOPIC="${GEN0_EPSILON_COSTMAP_TOPIC:-/projected_costmap}"
OBJECT_POSE_TOPIC="${GEN0_EPSILON_OBJECT_POSE_TOPIC:-/gen0_perception/trash_poses}"
ACTOR_POSE_TOPICS="${GEN0_EPSILON_ACTOR_POSE_TOPICS:-${GEN0_DYNAMIC_ACTOR_TOPICS:-}}"
CMD_VEL_TOPIC="${GEN0_EPSILON_CMD_VEL_TOPIC:-/epsilon/cmd_vel_raw}"
PREDICTED_TRAJECTORIES_TOPIC="${GEN0_EPSILON_PREDICTED_TRAJECTORIES_TOPIC:-/epsilon/predicted_trajectories}"
USE_SIM_TIME="${GEN0_EPSILON_USE_SIM_TIME:-true}"
PREDICTION_RATE="${GEN0_EPSILON_PREDICTION_RATE:-5.0}"
DYNAMIC_PUBLISH_PERIOD="${GEN0_EPSILON_DYNAMIC_PUBLISH_PERIOD:-0.1}"
PATH_TIMEOUT="${GEN0_EPSILON_PATH_TIMEOUT:-120.0}"
USE_CMD_VEL_MUX="${GEN0_EPSILON_USE_CMD_VEL_MUX:-false}"
CONTROL_SOURCE="${GEN0_EPSILON_CONTROL_SOURCE:-auto}"
NAV2_CMD_VEL_TOPIC="${GEN0_NAV2_RAW_CMD_VEL_TOPIC:-/control/nav2_cmd_vel_raw}"
MUX_OUTPUT_CMD_VEL_TOPIC="${GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC:-/control/cmd_vel_raw}"
CONTROL_MODE_TOPIC="${GEN0_EPSILON_CONTROL_MODE_TOPIC:-/epsilon/control_mode}"
SELECTED_SOURCE_TOPIC="${GEN0_EPSILON_SELECTED_SOURCE_TOPIC:-/epsilon/selected_control_source}"
EPSILON_STATUS_TOPIC="${GEN0_EPSILON_STATUS_TOPIC:-/epsilon/status}"
EPSILON_STATUS_TIMEOUT="${GEN0_EPSILON_STATUS_TIMEOUT:-1.5}"
FALLBACK_TO_NAV2="${GEN0_EPSILON_FALLBACK_TO_NAV2:-true}"
ACTOR_SOURCE="${GEN0_EPSILON_ACTOR_SOURCE:-scenario}"
ACTORS_SCENARIO_PATH="${GEN0_EPSILON_ACTORS_SCENARIO_PATH:-}"
SCENARIO_ACTOR_TOPIC_PREFIX="${GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_PREFIX:-/epsilon/scenario_actor}"
SCENARIO_ACTOR_PUBLISH_RATE="${GEN0_EPSILON_SCENARIO_ACTOR_PUBLISH_RATE:-10.0}"
SCENARIO_ACTOR_FRAME_ID="${GEN0_EPSILON_SCENARIO_ACTOR_FRAME_ID:-map}"
SCENARIO_ACTOR_TOPIC_COUNT="${GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_COUNT:-9}"
USE_SCENARIO_ACTOR_PUBLISHER="${GEN0_EPSILON_USE_SCENARIO_ACTOR_PUBLISHER:-false}"
SCENARIO_ACTOR_WORLD_SDF_PATH="${GEN0_EPSILON_ACTOR_WORLD_SDF_PATH:-${GEN0_ACTOR_WORLD_SDF_PATH:-}}"
SCENARIO_ACTOR_WORLD_VEHICLE_NAME="${GEN0_EPSILON_ACTOR_WORLD_VEHICLE_NAME:-${GEN0_ACTOR_WORLD_VEHICLE_NAME:-gen0_model}}"
SCENARIO_ACTOR_WORLD_TO_OUTPUT="${GEN0_EPSILON_ACTOR_WORLD_TO_OUTPUT:-${GEN0_ACTOR_WORLD_TO_OUTPUT:-true}}"
SCENARIO_ACTOR_OUTPUT_ORIGIN_XY="${GEN0_EPSILON_ACTOR_OUTPUT_ORIGIN_XY:-${GEN0_ACTOR_OUTPUT_ORIGIN_XY:-0.0,0.0}}"
ACTOR_DISCOVERY_TIMEOUT="${GEN0_EPSILON_ACTOR_DISCOVERY_TIMEOUT:-20}"

DESIRED_VELOCITY="${GEN0_EPSILON_DESIRED_VELOCITY:-1.5}"
USE_SCENE_BRIDGE="${GEN0_EPSILON_USE_SCENE_BRIDGE:-true}"
USE_QCNET_PREDICTION="${GEN0_EPSILON_USE_QCNET_PREDICTION:-true}"
QCNET_BACKEND="${GEN0_QCNET_BACKEND:-qcnet}"
QCNET_ROOT="${GEN0_QCNET_ROOT:-/home/zjxue2007/QCNet}"
QCNET_CKPT_PATH="${GEN0_QCNET_CKPT_PATH:-/home/zjxue2007/QCNet_AV2.ckpt}"
QCNET_DEVICE="${GEN0_QCNET_DEVICE:-cuda}"

PARAMS_FILE="${GEN0_EPSILON_PARAMS_FILE:-}"
QCNET_PARAMS_FILE="${GEN0_QCNET_PARAMS_FILE:-}"

ODOM_WAIT_TIMEOUT="${GEN0_EPSILON_ODOM_WAIT_TIMEOUT:-20}"
COSTMAP_WAIT_TIMEOUT="${GEN0_EPSILON_COSTMAP_WAIT_TIMEOUT:-30}"
PATH_WAIT_TIMEOUT="${GEN0_EPSILON_PATH_WAIT_TIMEOUT:-90}"
REQUIRE_PATH="${GEN0_EPSILON_REQUIRE_PATH:-true}"
ALLOW_EXISTING="${GEN0_EPSILON_ALLOW_EXISTING_NODES:-false}"

mkdir -p "$ROS_LOG_DIR"
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

warn_missing_file() {
  if [[ ! -f "$1" ]]; then
    printf 'Warning: file not found: %s\n' "$1" >&2
  fi
}

warn_missing_dir() {
  if [[ ! -d "$1" ]]; then
    printf 'Warning: directory not found: %s\n' "$1" >&2
  fi
}

check_qcnet_cuda_ready() {
  if [[ "$USE_QCNET_PREDICTION" != "true" || "$QCNET_BACKEND" != "qcnet" ]]; then
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
  local probe_timeout="${GEN0_EPSILON_TOPIC_PROBE_TIMEOUT:-5}"
  local deadline=$((SECONDS + wait_timeout))
  local remaining
  local current_probe_timeout

  log "Waiting for $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    remaining=$((deadline - SECONDS))
    current_probe_timeout="$probe_timeout"
    if ((remaining < probe_timeout)); then
      current_probe_timeout="$remaining"
    fi

    if timeout --kill-after=1s "$current_probe_timeout" ros2 topic echo --no-daemon --once "$topic" --field header >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done

  return 1
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble setup not found at /opt/ros/humble/setup.bash\n' >&2
  exit 1
fi

require_file "$WORKSPACE/install/setup.bash"
if [[ -n "$PARAMS_FILE" ]]; then
  require_file "$PARAMS_FILE"
fi
if [[ -n "$QCNET_PARAMS_FILE" ]]; then
  require_file "$QCNET_PARAMS_FILE"
fi

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"

if ! ros2 pkg prefix epsilon_planning >/dev/null 2>&1; then
  printf 'ROS package epsilon_planning is not in the sourced workspace.\n\n' >&2
  printf 'Build the migrated packages first:\n\n' >&2
  printf '  colcon build --packages-select vehicle_msgs epsilon_core qcnet_prediction epsilon_planning\n\n' >&2
  exit 1
fi
if [[ "$USE_QCNET_PREDICTION" == "true" ]] && ! ros2 pkg prefix qcnet_prediction >/dev/null 2>&1; then
  printf 'ROS package qcnet_prediction is not in the sourced workspace.\n\n' >&2
  printf 'Build it with epsilon_planning before launching this stack.\n' >&2
  exit 1
fi

if [[ "$ALLOW_EXISTING" != "true" ]]; then
  existing="$(
    timeout 5s ros2 node list --no-daemon 2>/dev/null \
      | grep -E '(^/epsilon_scene_bridge_node$|^/epsilon_integrated_planner_node$|^/qcnet_prediction_node$|^/epsilon_cmd_vel_mux_node$)' \
      || true
  )"
  if [[ -n "$existing" ]]; then
    printf 'Existing EPSILON/QCNet nodes are already running:\n%s\n\n' "$existing" >&2
    printf 'Run ./stop_gen0_epsilon.sh first, or set GEN0_EPSILON_ALLOW_EXISTING_NODES=true to bypass this check.\n' >&2
    exit 1
  fi
fi

case "$QCNET_BACKEND" in
  auto|qcnet|constant_velocity) ;;
  *)
    printf 'Invalid GEN0_QCNET_BACKEND=%s. Use auto, qcnet, or constant_velocity.\n' "$QCNET_BACKEND" >&2
    exit 1
    ;;
esac

case "$ACTOR_SOURCE" in
  scenario|live|none) ;;
  *)
    printf 'Invalid GEN0_EPSILON_ACTOR_SOURCE=%s. Use scenario, live, or none.\n' "$ACTOR_SOURCE" >&2
    exit 1
    ;;
esac

if [[ "$QCNET_BACKEND" == "qcnet" ]]; then
  require_file "$QCNET_CKPT_PATH"
  if [[ ! -d "$QCNET_ROOT" ]]; then
    printf 'Required QCNet root directory not found: %s\n' "$QCNET_ROOT" >&2
    exit 1
  fi
elif [[ "$USE_QCNET_PREDICTION" == "true" && "$QCNET_BACKEND" == "auto" ]]; then
  warn_missing_dir "$QCNET_ROOT"
  warn_missing_file "$QCNET_CKPT_PATH"
fi
if ! check_qcnet_cuda_ready; then
  printf 'QCNet is configured as backend=qcnet device=%s, but this Python environment has no CUDA-enabled PyTorch.\n' "$QCNET_DEVICE" >&2
  printf 'Install CUDA-enabled PyTorch for QCNet GPU, or temporarily run with GEN0_QCNET_BACKEND=constant_velocity to validate the rest of the stack.\n' >&2
  exit 1
fi

if [[ -z "$ACTORS_SCENARIO_PATH" && -n "${GEN0_ACTORS_SCENARIO:-}" ]]; then
  ACTORS_SCENARIO_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/scenarios/$WORLD/${GEN0_ACTORS_SCENARIO}.sdf"
fi
if [[ -z "$SCENARIO_ACTOR_WORLD_SDF_PATH" ]]; then
  SCENARIO_ACTOR_WORLD_SDF_PATH="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.sdf"
fi

build_scenario_actor_pose_topics() {
  local prefix="${SCENARIO_ACTOR_TOPIC_PREFIX%/}"
  local topic=""
  local actor_index
  ACTOR_POSE_TOPICS=""
  for ((actor_index = 1; actor_index <= SCENARIO_ACTOR_TOPIC_COUNT; actor_index++)); do
    topic="$prefix/pedestrian_${actor_index}/pose"
    if [[ -n "$ACTOR_POSE_TOPICS" ]]; then
      ACTOR_POSE_TOPICS+=","
    fi
    ACTOR_POSE_TOPICS+="$topic"
  done
}

configure_actor_source() {
  if [[ "$ACTOR_SOURCE" == "none" ]]; then
    ACTOR_POSE_TOPICS=""
    USE_SCENARIO_ACTOR_PUBLISHER=false
    return
  fi

  if [[ "$ACTOR_SOURCE" == "live" ]]; then
    USE_SCENARIO_ACTOR_PUBLISHER=false
    return
  fi

  if [[ -z "$ACTORS_SCENARIO_PATH" ]]; then
    log "No actor scenario configured; scenario actor input is empty."
    ACTOR_POSE_TOPICS=""
    USE_SCENARIO_ACTOR_PUBLISHER=false
    return
  fi

  require_file "$ACTORS_SCENARIO_PATH"
  require_file "$SCENARIO_ACTOR_WORLD_SDF_PATH"
  if [[ -z "${ACTOR_POSE_TOPICS//[[:space:]]/}" ]]; then
    build_scenario_actor_pose_topics
  fi
  USE_SCENARIO_ACTOR_PUBLISHER=true
}

discover_actor_pose_topics() {
  if [[ -n "${ACTOR_POSE_TOPICS//[[:space:]]/}" || -z "${GEN0_ACTORS_SCENARIO:-}" ]]; then
    return
  fi

  local deadline=$((SECONDS + ACTOR_DISCOVERY_TIMEOUT))
  local candidates topic info type
  while ((SECONDS < deadline)); do
    candidates="$(
      timeout 5s ros2 topic list --no-daemon 2>/dev/null \
        | grep -E '^/actor/[^/]+/pose$' \
        | sort -V \
        || true
    )"
    ACTOR_POSE_TOPICS=""
    while IFS= read -r topic; do
      [[ -z "$topic" ]] && continue
      info="$(timeout 3s ros2 topic info "$topic" 2>/dev/null || true)"
      if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$info"; then
        continue
      fi
      type="$(timeout 3s ros2 topic type "$topic" 2>/dev/null || true)"
      if [[ "$type" == "geometry_msgs/msg/PoseStamped" ]]; then
        if [[ -z "$ACTOR_POSE_TOPICS" ]]; then
          ACTOR_POSE_TOPICS="$topic"
        else
          ACTOR_POSE_TOPICS+=",$topic"
        fi
      fi
    done <<< "$candidates"

    if [[ -n "$ACTOR_POSE_TOPICS" ]]; then
      log "Discovered actor pose topics with live publishers: $ACTOR_POSE_TOPICS"
      return
    fi
    sleep 1
  done

  log "No actor PoseStamped topics discovered within ${ACTOR_DISCOVERY_TIMEOUT}s; dynamic actor input is empty."
}

if ! wait_for_topic_once "$ODOM_TOPIC" "$ODOM_WAIT_TIMEOUT" "EPSILON odometry"; then
  printf 'Timed out waiting for %s.\n' "$ODOM_TOPIC" >&2
  printf 'Start ./run_gen0_3d_slam.sh first and wait until stable odometry is publishing.\n' >&2
  exit 1
fi

if ! wait_for_topic_once "$COSTMAP_TOPIC" "$COSTMAP_WAIT_TIMEOUT" "projected costmap"; then
  printf 'Timed out waiting for %s.\n' "$COSTMAP_TOPIC" >&2
  printf 'Keep the 3D SLAM/projected-map stack running until the projected costmap is populated.\n' >&2
  exit 1
fi

if [[ "$REQUIRE_PATH" == "true" ]]; then
  if ! wait_for_topic_once "$PATH_TOPIC" "$PATH_WAIT_TIMEOUT" "Nav2 smoothed path"; then
    printf 'Timed out waiting for %s.\n' "$PATH_TOPIC" >&2
    printf 'Start Nav2, send a navigation goal, or set GEN0_EPSILON_REQUIRE_PATH=false to launch EPSILON before a path exists.\n' >&2
    exit 1
  fi
else
  log "Skipping path preflight; EPSILON will wait internally for $PATH_TOPIC."
fi

configure_actor_source
if [[ "$ACTOR_SOURCE" == "live" ]]; then
  discover_actor_pose_topics
fi

if [[ "$USE_CMD_VEL_MUX" == "true" && "$NAV2_CMD_VEL_TOPIC" == "$MUX_OUTPUT_CMD_VEL_TOPIC" ]]; then
  printf 'GEN0_NAV2_RAW_CMD_VEL_TOPIC and GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC must differ when command muxing is enabled.\n' >&2
  exit 1
fi

epsilon_launch=(
  ros2 launch epsilon_planning epsilon_planning.launch.py
  use_sim_time:="$USE_SIM_TIME"
  cmd_vel_topic:="$CMD_VEL_TOPIC"
  odom_topic:="$ODOM_TOPIC"
  desired_velocity:="$DESIRED_VELOCITY"
  use_scene_bridge:="$USE_SCENE_BRIDGE"
  path_topic:="$PATH_TOPIC"
  costmap_topic:="$COSTMAP_TOPIC"
  object_pose_topic:="$OBJECT_POSE_TOPIC"
  use_scenario_actor_publisher:="$USE_SCENARIO_ACTOR_PUBLISHER"
  scenario_actor_pose_topic_prefix:="$SCENARIO_ACTOR_TOPIC_PREFIX"
  scenario_actor_publish_rate:="$SCENARIO_ACTOR_PUBLISH_RATE"
  scenario_actor_frame_id:="$SCENARIO_ACTOR_FRAME_ID"
  scenario_actor_world_vehicle_name:="$SCENARIO_ACTOR_WORLD_VEHICLE_NAME"
  scenario_actor_world_to_output:="$SCENARIO_ACTOR_WORLD_TO_OUTPUT"
  scenario_actor_output_origin_xy:="$SCENARIO_ACTOR_OUTPUT_ORIGIN_XY"
  predicted_trajectories_topic:="$PREDICTED_TRAJECTORIES_TOPIC"
  use_qcnet_prediction:="$USE_QCNET_PREDICTION"
  qcnet_backend:="$QCNET_BACKEND"
  qcnet_root:="$QCNET_ROOT"
  qcnet_ckpt_path:="$QCNET_CKPT_PATH"
  qcnet_device:="$QCNET_DEVICE"
  prediction_rate:="$PREDICTION_RATE"
  dynamic_publish_period:="$DYNAMIC_PUBLISH_PERIOD"
  path_timeout:="$PATH_TIMEOUT"
  use_cmd_vel_mux:="$USE_CMD_VEL_MUX"
  control_source:="$CONTROL_SOURCE"
  nav2_cmd_vel_topic:="$NAV2_CMD_VEL_TOPIC"
  mux_output_cmd_vel_topic:="$MUX_OUTPUT_CMD_VEL_TOPIC"
  control_mode_topic:="$CONTROL_MODE_TOPIC"
  selected_source_topic:="$SELECTED_SOURCE_TOPIC"
  epsilon_status_topic:="$EPSILON_STATUS_TOPIC"
  epsilon_status_timeout:="$EPSILON_STATUS_TIMEOUT"
  fallback_to_nav2:="$FALLBACK_TO_NAV2"
)
if [[ -n "$ACTOR_POSE_TOPICS" ]]; then
  epsilon_launch+=(actor_pose_topics:="$ACTOR_POSE_TOPICS")
fi
if [[ -n "$ACTORS_SCENARIO_PATH" ]]; then
  epsilon_launch+=(actor_scenario_path:="$ACTORS_SCENARIO_PATH")
fi
if [[ -n "$SCENARIO_ACTOR_WORLD_SDF_PATH" ]]; then
  epsilon_launch+=(scenario_actor_world_sdf_path:="$SCENARIO_ACTOR_WORLD_SDF_PATH")
fi
if [[ -n "$PARAMS_FILE" ]]; then
  epsilon_launch+=(params_file:="$PARAMS_FILE")
fi
if [[ -n "$QCNET_PARAMS_FILE" ]]; then
  epsilon_launch+=(qcnet_params_file:="$QCNET_PARAMS_FILE")
fi

log "Starting EPSILON/QCNet: odom=$ODOM_TOPIC path=$PATH_TOPIC path_timeout=${PATH_TIMEOUT}s costmap=$COSTMAP_TOPIC cmd_vel=$CMD_VEL_TOPIC backend=$QCNET_BACKEND device=$QCNET_DEVICE prediction_rate=${PREDICTION_RATE}Hz mux=$USE_CMD_VEL_MUX source=$CONTROL_SOURCE selected_topic=$SELECTED_SOURCE_TOPIC actor_source=$ACTOR_SOURCE scenario_actor_publisher=$USE_SCENARIO_ACTOR_PUBLISHER actor_scenario=${ACTORS_SCENARIO_PATH:-none} actors=${ACTOR_POSE_TOPICS:-none}"
exec "${epsilon_launch[@]}"
