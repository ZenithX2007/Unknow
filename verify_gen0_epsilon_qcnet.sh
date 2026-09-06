#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
WORLD="${GEN0_WORLD:-my_map}"
DEFAULT_ACTORS_SCENARIO=""
if [[ "$WORLD" == "my_map" ]]; then
  DEFAULT_ACTORS_SCENARIO="walking_actors3"
fi
ACTORS_SCENARIO="${GEN0_ACTORS_SCENARIO-$DEFAULT_ACTORS_SCENARIO}"
MODE="${1:---runtime}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
INSTALL_PREFIX="${GEN0_INSTALL_PREFIX:-$WORKSPACE/install}"

ODOM_TOPIC="${GEN0_EPSILON_ODOM_TOPIC:-/gen0_mapping/stable_odom}"
COSTMAP_TOPIC="${GEN0_EPSILON_COSTMAP_TOPIC:-/projected_costmap}"
PATH_TOPIC="${GEN0_EPSILON_PATH_TOPIC:-/plan_smoothed}"
OBJECT_POSE_TOPIC="${GEN0_EPSILON_OBJECT_POSE_TOPIC:-/gen0_perception/trash_poses}"
TRASH_POSE_TOPIC="${GEN0_VERIFY_TRASH_POSE_TOPIC:-/gen0_perception/trash_poses}"
CMD_VEL_TOPIC="${GEN0_EPSILON_CMD_VEL_TOPIC:-/epsilon/cmd_vel_raw}"
NAV2_RAW_CMD_VEL_TOPIC="${GEN0_NAV2_RAW_CMD_VEL_TOPIC:-/control/nav2_cmd_vel_raw}"
MUX_OUTPUT_CMD_VEL_TOPIC="${GEN0_EPSILON_MUX_OUTPUT_CMD_VEL_TOPIC:-/control/cmd_vel_raw}"
EXPECT_CMD_MUX="${GEN0_VERIFY_EXPECT_CMD_MUX:-auto}"
ARENA_STATIC_TOPIC="${GEN0_EPSILON_ARENA_STATIC_TOPIC:-/epsilon/arena_info_static}"
ARENA_DYNAMIC_TOPIC="${GEN0_EPSILON_ARENA_DYNAMIC_TOPIC:-/epsilon/arena_info_dynamic}"
PREDICTED_TRAJECTORIES_TOPIC="${GEN0_EPSILON_PREDICTED_TRAJECTORIES_TOPIC:-/epsilon/predicted_trajectories}"
EPSILON_STATUS_TOPIC="${GEN0_EPSILON_STATUS_TOPIC:-/epsilon/status}"
EPSILON_SELECTED_SOURCE_TOPIC="${GEN0_EPSILON_SELECTED_SOURCE_TOPIC:-/epsilon/selected_control_source}"

QCNET_BACKEND="${GEN0_QCNET_BACKEND:-qcnet}"
QCNET_ROOT="${GEN0_QCNET_ROOT:-/home/zjxue2007/QCNet}"
QCNET_CKPT_PATH="${GEN0_QCNET_CKPT_PATH:-/home/zjxue2007/QCNet_AV2.ckpt}"
QCNET_DEVICE="${GEN0_QCNET_DEVICE:-cuda}"

ACTOR_POSE_TOPICS="${GEN0_EPSILON_ACTOR_POSE_TOPICS:-${GEN0_DYNAMIC_ACTOR_TOPICS:-}}"
ACTOR_SOURCE="${GEN0_EPSILON_ACTOR_SOURCE:-scenario}"
SCENARIO_ACTOR_TOPIC_PREFIX="${GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_PREFIX:-/epsilon/scenario_actor}"
SCENARIO_ACTOR_TOPIC_COUNT="${GEN0_EPSILON_SCENARIO_ACTOR_TOPIC_COUNT:-9}"

TOPIC_TIMEOUT="${GEN0_VERIFY_TOPIC_TIMEOUT:-20}"
PLANNER_TIMEOUT="${GEN0_VERIFY_PLANNER_TIMEOUT:-40}"
PROBE_TIMEOUT="${GEN0_VERIFY_PROBE_TIMEOUT:-5}"
EXPECT_ACTORS="${GEN0_VERIFY_EXPECT_ACTORS:-auto}"
EXPECT_TRASH_STATIC="${GEN0_VERIFY_EXPECT_TRASH_STATIC:-false}"
EXPECT_PLANNER_OK="${GEN0_VERIFY_EXPECT_PLANNER_OK:-true}"
EXPECT_PATH="${GEN0_VERIFY_EXPECT_PATH:-false}"
RUN_QCNET_LOAD_CHECK="${GEN0_VERIFY_QCNET_LOAD_CHECK:-true}"

PASS_COUNT=0
FAIL_COUNT=0
SKIP_COUNT=0
TMP_FILES=()

usage() {
  cat <<'EOF'
Usage:
  ./verify_gen0_epsilon_qcnet.sh [--static|--runtime]

Modes:
  --static   Check sourced packages, launch files, QCNet root, and checkpoint loading.
  --runtime  Run static checks and sample live ROS topics after the stack is running.
EOF
}

cleanup() {
  for path in "${TMP_FILES[@]}"; do
    rm -f "$path"
  done
}
trap cleanup EXIT

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$1"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$1" >&2
}

skip() {
  SKIP_COUNT=$((SKIP_COUNT + 1))
  printf '[SKIP] %s\n' "$1"
}

check() {
  local label="$1"
  shift
  if "$@"; then
    pass "$label"
  else
    fail "$label"
  fi
}

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

string_list_to_array() {
  local raw="$1"
  local current=""
  local ch
  ACTOR_TOPICS=()
  for ((i = 0; i < ${#raw}; i++)); do
    ch="${raw:i:1}"
    if [[ "$ch" == "," ]]; then
      current="${current#"${current%%[![:space:]]*}"}"
      current="${current%"${current##*[![:space:]]}"}"
      if [[ -n "$current" ]]; then
        ACTOR_TOPICS+=("$current")
      fi
      current=""
    else
      current+="$ch"
    fi
  done
  current="${current#"${current%%[![:space:]]*}"}"
  current="${current%"${current##*[![:space:]]}"}"
  if [[ -n "$current" ]]; then
    ACTOR_TOPICS+=("$current")
  fi
}

new_tmp_file() {
  local path
  path="$(mktemp /tmp/gen0_verify.XXXXXX)"
  TMP_FILES+=("$path")
  printf '%s\n' "$path"
}

has_package() {
  ros2 pkg prefix "$1" >/dev/null 2>&1
}

topic_has_type() {
  local topic="$1"
  local expected_type="$2"
  local output
  output="$(timeout "$PROBE_TIMEOUT" ros2 topic type "$topic" 2>/dev/null || true)"
  [[ "$output" == *"$expected_type"* ]]
}

node_exists() {
  local node_name="$1"
  local deadline=$((SECONDS + PROBE_TIMEOUT))
  while ((SECONDS < deadline)); do
    if timeout 3s ros2 node list --no-daemon 2>/dev/null | grep -Fxq "$node_name"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

topic_has_publishers() {
  local topic="$1"
  local output
  output="$(timeout "$PROBE_TIMEOUT" ros2 topic info "$topic" 2>/dev/null || true)"
  grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$output"
}

topic_has_subscribers() {
  local topic="$1"
  local output
  output="$(timeout "$PROBE_TIMEOUT" ros2 topic info "$topic" 2>/dev/null || true)"
  grep -Eq 'Subscription count: [1-9][0-9]*' <<<"$output"
}

wait_for_topic_message() {
  local topic="$1"
  local field="$2"
  local wait_timeout="$3"
  local pattern="${4:-}"
  local label="${5:-$topic}"
  local deadline=$((SECONDS + wait_timeout))
  local path
  local remaining
  local current_timeout

  path="$(new_tmp_file)"
  while ((SECONDS < deadline)); do
    remaining=$((deadline - SECONDS))
    current_timeout="$PROBE_TIMEOUT"
    if ((remaining < current_timeout)); then
      current_timeout="$remaining"
    fi
    if ((current_timeout < 1)); then
      current_timeout=1
    fi

    if [[ -n "$field" ]]; then
      if timeout --kill-after=1s "$current_timeout" ros2 topic echo --no-daemon --once "$topic" --field "$field" >"$path" 2>&1; then
        if [[ -z "$pattern" ]] || grep -Eq "$pattern" "$path"; then
          return 0
        fi
      fi
    else
      if timeout --kill-after=1s "$current_timeout" ros2 topic echo --no-daemon --once "$topic" >"$path" 2>&1; then
        if [[ -z "$pattern" ]] || grep -Eq "$pattern" "$path"; then
          return 0
        fi
      fi
    fi
    sleep 1
  done

  printf 'Last sample for %s:\n' "$label" >&2
  sed -n '1,80p' "$path" >&2 || true
  return 1
}

check_any_actor_topic() {
  local topic
  for topic in "${ACTOR_TOPICS[@]}"; do
    if topic_has_type "$topic" "geometry_msgs/msg/PoseStamped" &&
      wait_for_topic_message "$topic" "header" "$PROBE_TIMEOUT" "" "$topic"; then
      printf 'Sampled actor topic: %s\n' "$topic"
      return 0
    fi
  done
  return 1
}

discover_actor_pose_topics() {
  local candidates topic info type
  candidates="$(
    timeout "$PROBE_TIMEOUT" ros2 topic list -t 2>/dev/null \
      | awk '$1 ~ /^\/actor\/.+\/pose$/ && $2 == "[geometry_msgs/msg/PoseStamped]" {print $1}' \
      || true
  )"
  while IFS= read -r topic; do
    [[ -z "$topic" ]] && continue
    info="$(timeout "$PROBE_TIMEOUT" ros2 topic info "$topic" 2>/dev/null || true)"
    if ! grep -Eq 'Publisher count: [1-9][0-9]*' <<< "$info"; then
      continue
    fi
    type="$(timeout "$PROBE_TIMEOUT" ros2 topic type "$topic" 2>/dev/null || true)"
    [[ "$type" == "geometry_msgs/msg/PoseStamped" ]] && ACTOR_TOPICS+=("$topic")
  done <<< "$candidates"
}

build_scenario_actor_pose_topics() {
  local prefix="${SCENARIO_ACTOR_TOPIC_PREFIX%/}"
  local topic
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

check_qcnet_checkpoint_loads() {
  if [[ "$QCNET_BACKEND" == "constant_velocity" ]]; then
    skip "QCNet checkpoint load check skipped because GEN0_QCNET_BACKEND=constant_velocity"
    return 0
  fi
  if [[ "$RUN_QCNET_LOAD_CHECK" != "true" ]]; then
    skip "QCNet checkpoint load check disabled"
    return 0
  fi

  PYTHONPATH="$QCNET_ROOT:${PYTHONPATH:-}" python3 - "$QCNET_CKPT_PATH" "$QCNET_DEVICE" <<'PY'
import sys

checkpoint_path, device = sys.argv[1:3]

import torch
if device == "auto":
    device = "cuda" if torch.cuda.is_available() else "cpu"
if device.startswith("cuda") and not torch.cuda.is_available():
    print(
        "QCNet CUDA requested but unavailable: "
        f"torch={torch.__version__} torch_cuda={torch.version.cuda} "
        f"cuda_available={torch.cuda.is_available()} device_count={torch.cuda.device_count()}"
    )
    sys.exit(1)

from predictors.qcnet import QCNet

model = QCNet.load_from_checkpoint(checkpoint_path=checkpoint_path, map_location=device)
print(
    "QCNet checkpoint loaded: "
    f"history={model.num_historical_steps} future={model.num_future_steps}"
)
PY
}

check_epsilon_launch_args_parse() {
  ros2 launch epsilon_planning epsilon_planning.launch.py --show-args >/dev/null
}

check_epsilon_nav2_launch_args_parse() {
  ros2 launch epsilon_planning gen0_navigation_epsilon.launch.py --show-args >/dev/null
}

check_publish_path_bt_plugin_exports() {
  local library_path
  library_path="$(
    ros2 pkg prefix gen0_nav2_path_exporter
  )/lib/libgen0_nav2_path_exporter_bt_node.so"
  [[ -f "$library_path" ]] || return 1
  readelf -Ws "$library_path" 2>/dev/null \
    | grep -E 'GLOBAL.*DEFAULT.*BT_RegisterNodesFromPlugin' >/dev/null
}

static_checks() {
  log "Static checks: workspace=$WORKSPACE qcnet_root=$QCNET_ROOT ckpt=$QCNET_CKPT_PATH backend=$QCNET_BACKEND device=$QCNET_DEVICE"
  check "ROS 2 Humble setup exists" test -f /opt/ros/humble/setup.bash
  check "Workspace install setup exists" test -f "$INSTALL_PREFIX/setup.bash"

  source_ros_setup /opt/ros/humble/setup.bash
  source_ros_setup "$INSTALL_PREFIX/setup.bash"

  if [[ "$INSTALL_PREFIX" == "$WORKSPACE/install" ]]; then
    check "ROS package gen0_main is discoverable" has_package gen0_main
  elif has_package gen0_main; then
    pass "ROS package gen0_main is discoverable"
  else
    skip "ROS package gen0_main not present in non-workspace install prefix"
  fi
  check "ROS package vehicle_msgs is discoverable" has_package vehicle_msgs
  check "ROS package epsilon_core is discoverable" has_package epsilon_core
  check "ROS package epsilon_planning is discoverable" has_package epsilon_planning
  check "ROS package qcnet_prediction is discoverable" has_package qcnet_prediction
  check "ROS package gen0_nav2_path_exporter is discoverable" has_package gen0_nav2_path_exporter
  check "QCNet root exists" test -d "$QCNET_ROOT"
  check "QCNet checkpoint exists" test -f "$QCNET_CKPT_PATH"
  check "EPSILON launch arguments parse" check_epsilon_launch_args_parse
  check "EPSILON Nav2 launch arguments parse" check_epsilon_nav2_launch_args_parse
  check "PublishPath BT plugin exports registration symbol" check_publish_path_bt_plugin_exports
  check "QCNet checkpoint loads" check_qcnet_checkpoint_loads
}

runtime_checks() {
  if [[ -z "$ACTOR_POSE_TOPICS" && "$ACTOR_SOURCE" == "scenario" && -n "$ACTORS_SCENARIO" ]]; then
    build_scenario_actor_pose_topics
  fi
  string_list_to_array "$ACTOR_POSE_TOPICS"
  if [[ ${#ACTOR_TOPICS[@]} -eq 0 && "$ACTOR_SOURCE" == "live" ]]; then
    discover_actor_pose_topics
  fi

  log "Runtime checks: odom=$ODOM_TOPIC path=$PATH_TOPIC costmap=$COSTMAP_TOPIC actor_source=$ACTOR_SOURCE actors=${#ACTOR_TOPICS[@]}"
  if node_exists /epsilon_scene_bridge_node; then
    pass "Node /epsilon_scene_bridge_node exists"
  else
    fail "Node /epsilon_scene_bridge_node exists"
    return 1
  fi
  check "Node /epsilon_integrated_planner_node exists" node_exists /epsilon_integrated_planner_node
  check "Node /qcnet_prediction_node exists" node_exists /qcnet_prediction_node

  check "Odom topic type is nav_msgs/msg/Odometry" topic_has_type "$ODOM_TOPIC" nav_msgs/msg/Odometry
  check "Projected costmap topic type is nav_msgs/msg/OccupancyGrid" topic_has_type "$COSTMAP_TOPIC" nav_msgs/msg/OccupancyGrid
  if [[ "$EXPECT_PATH" == "true" ]]; then
    check "Nav2 smoothed path topic type is nav_msgs/msg/Path" topic_has_type "$PATH_TOPIC" nav_msgs/msg/Path
  else
    skip "Nav2 smoothed path topic type check skipped until a navigation goal is sent"
  fi
  check "EPSILON static scene topic type is vehicle_msgs/msg/ArenaInfoStatic" topic_has_type "$ARENA_STATIC_TOPIC" vehicle_msgs/msg/ArenaInfoStatic
  check "EPSILON dynamic scene topic type is vehicle_msgs/msg/ArenaInfoDynamic" topic_has_type "$ARENA_DYNAMIC_TOPIC" vehicle_msgs/msg/ArenaInfoDynamic
  check "QCNet prediction topic type is vehicle_msgs/msg/PredictedTrajectoryArray" topic_has_type "$PREDICTED_TRAJECTORIES_TOPIC" vehicle_msgs/msg/PredictedTrajectoryArray
  check "EPSILON status topic type is std_msgs/msg/String" topic_has_type "$EPSILON_STATUS_TOPIC" std_msgs/msg/String
  check "EPSILON cmd_vel topic type is geometry_msgs/msg/Twist" topic_has_type "$CMD_VEL_TOPIC" geometry_msgs/msg/Twist

  local mux_enabled=false
  if [[ "$EXPECT_CMD_MUX" == "true" ]] ||
    ([[ "$EXPECT_CMD_MUX" == "auto" ]] && node_exists /epsilon_cmd_vel_mux_node); then
    mux_enabled=true
    check "Node /epsilon_cmd_vel_mux_node exists" node_exists /epsilon_cmd_vel_mux_node
    check "Nav2 raw cmd_vel topic type is geometry_msgs/msg/Twist" topic_has_type "$NAV2_RAW_CMD_VEL_TOPIC" geometry_msgs/msg/Twist
    check "Mux output cmd_vel topic type is geometry_msgs/msg/Twist" topic_has_type "$MUX_OUTPUT_CMD_VEL_TOPIC" geometry_msgs/msg/Twist
    check "Selected control source topic type is std_msgs/msg/String" topic_has_type "$EPSILON_SELECTED_SOURCE_TOPIC" std_msgs/msg/String
    check "EPSILON cmd_vel has downstream subscriber" topic_has_subscribers "$CMD_VEL_TOPIC"
    check "Mux output has downstream subscriber" topic_has_subscribers "$MUX_OUTPUT_CMD_VEL_TOPIC"
  else
    skip "EPSILON/Nav2 command mux checks skipped"
  fi

  check "Odom publishes samples" wait_for_topic_message "$ODOM_TOPIC" header "$TOPIC_TIMEOUT"
  check "Projected costmap publishes samples" wait_for_topic_message "$COSTMAP_TOPIC" header "$TOPIC_TIMEOUT"
  if [[ "$EXPECT_PATH" == "true" ]]; then
    check "Nav2 smoothed path publishes samples" wait_for_topic_message "$PATH_TOPIC" header "$TOPIC_TIMEOUT"
    check "EPSILON static scene publishes lane/static obstacle scene" wait_for_topic_message "$ARENA_STATIC_TOPIC" lane_net.lanes "$TOPIC_TIMEOUT" "id:"
  else
    skip "Nav2 path and static-scene content checks skipped until a navigation goal is sent"
  fi
  check "EPSILON dynamic scene publishes ego vehicle" wait_for_topic_message "$ARENA_DYNAMIC_TOPIC" vehicle_set.vehicles "$TOPIC_TIMEOUT" "data: 0"

  if [[ "$EXPECT_TRASH_STATIC" == "true" ]]; then
    check "Trash pose topic type is geometry_msgs/msg/PoseArray" topic_has_type "$TRASH_POSE_TOPIC" geometry_msgs/msg/PoseArray
    check "Trash pose topic publishes samples" wait_for_topic_message "$TRASH_POSE_TOPIC" header "$TOPIC_TIMEOUT"
    check "Static obstacle scene includes circular obstacles" wait_for_topic_message "$ARENA_STATIC_TOPIC" obstacle_set.obs_circle "$TOPIC_TIMEOUT" "id:"
  else
    skip "Explicit static obstacle content check skipped"
  fi

  if [[ "$EXPECT_ACTORS" == "true" || ( "$EXPECT_ACTORS" == "auto" && ${#ACTOR_TOPICS[@]} -gt 0 ) ]]; then
    check "At least one actor PoseStamped topic publishes" check_any_actor_topic
    check "EPSILON dynamic scene includes actor vehicles" wait_for_topic_message "$ARENA_DYNAMIC_TOPIC" vehicle_set.vehicles "$TOPIC_TIMEOUT" "data: 10[0-9]+"
    check "QCNet prediction publishes non-ego trajectories" wait_for_topic_message "$PREDICTED_TRAJECTORIES_TOPIC" trajectories "$PLANNER_TIMEOUT" "data: 10[0-9]+"
  else
    skip "Actor and non-ego prediction checks skipped; no actor topics configured"
    check "QCNet prediction topic publishes samples" wait_for_topic_message "$PREDICTED_TRAJECTORIES_TOPIC" header "$TOPIC_TIMEOUT"
  fi

  if [[ "$EXPECT_PLANNER_OK" == "true" ]]; then
    if [[ "$EXPECT_PATH" == "true" ]]; then
      check "EPSILON planner status reaches ok" wait_for_topic_message "$EPSILON_STATUS_TOPIC" data "$PLANNER_TIMEOUT" "^ok "
    else
      skip "EPSILON planner ok check skipped until a navigation goal is sent"
    fi
  else
    check "EPSILON planner status publishes" wait_for_topic_message "$EPSILON_STATUS_TOPIC" data "$TOPIC_TIMEOUT"
  fi

  if [[ "$mux_enabled" == "true" ]]; then
    check "Selected control source publishes current owner" wait_for_topic_message "$EPSILON_SELECTED_SOURCE_TOPIC" data "$TOPIC_TIMEOUT" "selected="
  fi
}

case "$MODE" in
  --help|-h)
    usage
    exit 0
    ;;
  --static|static)
    mkdir -p "$ROS_LOG_DIR"
    export ROS_LOG_DIR
    static_checks
    ;;
  --runtime|runtime|"")
    mkdir -p "$ROS_LOG_DIR"
    export ROS_LOG_DIR
    static_checks
    runtime_checks
    ;;
  *)
    usage >&2
    exit 2
    ;;
esac

printf '\nVerification summary: pass=%d fail=%d skip=%d\n' "$PASS_COUNT" "$FAIL_COUNT" "$SKIP_COUNT"
if ((FAIL_COUNT > 0)); then
  exit 1
fi
