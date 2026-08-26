#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
WORLD="${GEN0_WORLD:-my_map}"
MAPPING_DRIVE="${GEN0_MAPPING_DRIVE:-false}"
TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-false}"
RVIZ="${GEN0_RVIZ:-true}"
PROJECTED_MAP_BACKEND="${GEN0_PROJECTED_MAP_BACKEND:-python}"
NAV2_PROFILE="${GEN0_NAV2_PROFILE:-scurm_gen0}"
NAV2_USE_RESPAWN="${GEN0_NAV2_USE_RESPAWN:-false}"
NAV2_CONTROLLER_FREQUENCY="${GEN0_NAV2_CONTROLLER_FREQUENCY:-10.0}"
NAV2_SMOOTHING_FREQUENCY="${GEN0_NAV2_SMOOTHING_FREQUENCY:-$NAV2_CONTROLLER_FREQUENCY}"
NAV2_ODOM_WAIT_TIMEOUT="${GEN0_NAV2_ODOM_WAIT_TIMEOUT:-120}"
NAV2_TERRAIN_WAIT_TIMEOUT="${GEN0_NAV2_TERRAIN_WAIT_TIMEOUT:-120}"
NAV2_TERRAIN_TOPIC="${GEN0_NAV2_TERRAIN_TOPIC:-/gen0_mapping/terrain_map_ext}"
NAV2_PROJECTED_MAP_WAIT_TIMEOUT="${GEN0_NAV2_PROJECTED_MAP_WAIT_TIMEOUT:-120}"
NAV2_TOPIC_PROBE_TIMEOUT="${GEN0_NAV2_TOPIC_PROBE_TIMEOUT:-5}"
NAV2_PROJECTED_MAP_TOPIC="${GEN0_NAV2_PROJECTED_MAP_TOPIC:-/projected_map}"

if [[ -n "${GEN0_NAV2_ODOM_TOPIC:-}" ]]; then
  NAV2_ODOM_TOPIC="$GEN0_NAV2_ODOM_TOPIC"
elif [[ "$NAV2_PROFILE" == "scurm_gen0" ]]; then
  NAV2_ODOM_TOPIC="/gen0_mapping/stable_odom"
else
  NAV2_ODOM_TOPIC="${GEN0_FAST_LIO_ODOM_TOPIC:-/gen0_mapping/fast_lio/odom}"
fi

if [[ -n "${GEN0_NAV2_COSTMAP_SOURCE:-}" ]]; then
  NAV2_COSTMAP_SOURCE="$GEN0_NAV2_COSTMAP_SOURCE"
elif [[ "$NAV2_PROFILE" == "scurm_gen0" ]]; then
  NAV2_COSTMAP_SOURCE="scurm_terrain"
else
  NAV2_COSTMAP_SOURCE="laser_scan"
fi

if [[ -n "${GEN0_NAV2_MAP_SOURCE:-}" ]]; then
  NAV2_MAP_SOURCE="$GEN0_NAV2_MAP_SOURCE"
elif [[ "$NAV2_PROFILE" == "scurm_gen0" ]]; then
  NAV2_MAP_SOURCE="projected_map"
else
  NAV2_MAP_SOURCE="yaml"
fi

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

slam_stack_alive() {
  kill -0 "$SLAM_PID" 2>/dev/null
}

wait_for_topic_once() {
  local topic="$1"
  local wait_timeout="$2"
  local description="$3"
  local field="${4:-header}"
  local deadline=$((SECONDS + wait_timeout))
  local remaining
  local current_probe_timeout

  log "Waiting for $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    if ! slam_stack_alive; then
      log "3D SLAM stack exited before $description became available."
      return 1
    fi

    remaining=$((deadline - SECONDS))
    current_probe_timeout="$NAV2_TOPIC_PROBE_TIMEOUT"
    if ((remaining < NAV2_TOPIC_PROBE_TIMEOUT)); then
      current_probe_timeout="$remaining"
    fi

    if timeout "$current_probe_timeout" ros2 topic echo --no-daemon --once "$topic" --field "$field" >/dev/null 2>&1; then
      log "$description is available."
      return 0
    fi
    sleep 1
  done

  return 1
}

wait_for_occupancy_grid_nonempty() {
  local topic="$1"
  local wait_timeout="$2"
  local description="$3"
  local info_file
  local deadline=$((SECONDS + wait_timeout))
  local remaining
  local current_probe_timeout
  local width
  local height

  info_file="$(mktemp /tmp/gen0_mapping_grid_info.XXXXXX)"
  log "Waiting for non-empty $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    if ! slam_stack_alive; then
      log "3D SLAM stack exited before $description became available."
      rm -f "$info_file"
      return 1
    fi

    remaining=$((deadline - SECONDS))
    current_probe_timeout="$NAV2_TOPIC_PROBE_TIMEOUT"
    if ((remaining < NAV2_TOPIC_PROBE_TIMEOUT)); then
      current_probe_timeout="$remaining"
    fi

    if timeout "$current_probe_timeout" ros2 topic echo --no-daemon --once "$topic" --field info >"$info_file" 2>/dev/null; then
      width="$(awk '/^[[:space:]]*width:/ {print $2; exit}' "$info_file")"
      height="$(awk '/^[[:space:]]*height:/ {print $2; exit}' "$info_file")"
      if [[ "$width" =~ ^[0-9]+$ && "$height" =~ ^[0-9]+$ ]] && ((width > 1 && height > 1)); then
        rm -f "$info_file"
        log "$description is available."
        return 0
      fi
    fi
    sleep 1
  done

  rm -f "$info_file"
  return 1
}

cd "$WORKSPACE"
mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

./stop_gen0_nav2.sh
./stop_gen0_3d_slam.sh

GEN0_WORKSPACE="$PWD" \
GEN0_WORLD="$WORLD" \
GEN0_MAPPING_DRIVE="$MAPPING_DRIVE" \
GEN0_TRASH_CLEANUP="$TRASH_CLEANUP" \
GEN0_RVIZ="$RVIZ" \
GEN0_PROJECTED_MAP_BACKEND="$PROJECTED_MAP_BACKEND" \
./run_gen0_3d_slam.sh &
SLAM_PID=$!

cleanup() {
  trap - EXIT INT TERM

  if kill -0 "$SLAM_PID" 2>/dev/null; then
    kill -INT "$SLAM_PID" 2>/dev/null || true
  fi
  if [[ -n "${NAV2_PID:-}" ]] && kill -0 "$NAV2_PID" 2>/dev/null; then
    kill -INT "$NAV2_PID" 2>/dev/null || true
  fi

  wait "$SLAM_PID" 2>/dev/null || true
  if [[ -n "${NAV2_PID:-}" ]]; then
    wait "$NAV2_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

if ! wait_for_topic_once "$NAV2_ODOM_TOPIC" "$NAV2_ODOM_WAIT_TIMEOUT" "Nav2 odometry"; then
  printf 'Timed out waiting for %s. Check %s/fast_lio_3d_slam.log.\n' "$NAV2_ODOM_TOPIC" "$LOG_DIR" >&2
  exit 1
fi

if [[ "$NAV2_COSTMAP_SOURCE" == "scurm_terrain" ]]; then
  if ! wait_for_topic_once "$NAV2_TERRAIN_TOPIC" "$NAV2_TERRAIN_WAIT_TIMEOUT" "SCURM local terrain map"; then
    printf 'Timed out waiting for %s. Check %s/fast_lio_3d_slam.log.\n' "$NAV2_TERRAIN_TOPIC" "$LOG_DIR" >&2
    exit 1
  fi
fi

if [[ "$NAV2_MAP_SOURCE" == "projected_map" ]]; then
  if ! wait_for_occupancy_grid_nonempty "$NAV2_PROJECTED_MAP_TOPIC" "$NAV2_PROJECTED_MAP_WAIT_TIMEOUT" "online projected occupancy map"; then
    printf 'Timed out waiting for non-empty %s. Check %s/fast_lio_3d_slam.log.\n' "$NAV2_PROJECTED_MAP_TOPIC" "$LOG_DIR" >&2
    exit 1
  fi
fi

GEN0_WORKSPACE="$PWD" \
GEN0_NAV2_PROFILE="$NAV2_PROFILE" \
GEN0_WORLD="$WORLD" \
GEN0_NAV2_USE_RESPAWN="$NAV2_USE_RESPAWN" \
GEN0_NAV2_CONTROLLER_FREQUENCY="$NAV2_CONTROLLER_FREQUENCY" \
GEN0_NAV2_SMOOTHING_FREQUENCY="$NAV2_SMOOTHING_FREQUENCY" \
GEN0_NAV2_ODOM_WAIT_TIMEOUT="$NAV2_ODOM_WAIT_TIMEOUT" \
GEN0_NAV2_TERRAIN_WAIT_TIMEOUT="$NAV2_TERRAIN_WAIT_TIMEOUT" \
GEN0_NAV2_TERRAIN_TOPIC="$NAV2_TERRAIN_TOPIC" \
GEN0_NAV2_PROJECTED_MAP_WAIT_TIMEOUT="$NAV2_PROJECTED_MAP_WAIT_TIMEOUT" \
./run_gen0_nav2.sh &
NAV2_PID=$!

wait "$SLAM_PID"
SLAM_STATUS=$?

if kill -0 "$NAV2_PID" 2>/dev/null; then
  wait "$NAV2_PID"
  NAV2_STATUS=$?
else
  NAV2_STATUS=0
fi

if ((SLAM_STATUS != 0)); then
  exit "$SLAM_STATUS"
fi
exit "$NAV2_STATUS"
