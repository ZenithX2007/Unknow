#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE="${GEN0_WORKSPACE:-$PWD}"
LOG_DIR="${GEN0_LOG_DIR:-$WORKSPACE/runtime_logs}"
ROS_LOG_DIR="${ROS_LOG_DIR:-$LOG_DIR/ros}"
PROFILE="${GEN0_NAV2_PROFILE:-scurm_gen0}"
WORLD="${GEN0_WORLD:-my_map}"
FAST_LIO_ODOM_TOPIC="${GEN0_FAST_LIO_ODOM_TOPIC:-/gen0_mapping/fast_lio/odom}"
NAV2_ODOM_TOPIC_REQUESTED="${GEN0_NAV2_ODOM_TOPIC:-}"
USE_RESPAWN="${GEN0_NAV2_USE_RESPAWN:-false}"
NAV2_CONTROLLER_FREQUENCY="${GEN0_NAV2_CONTROLLER_FREQUENCY:-20.0}"
NAV2_SMOOTHING_FREQUENCY="${GEN0_NAV2_SMOOTHING_FREQUENCY:-$NAV2_CONTROLLER_FREQUENCY}"
if [[ -n "${GEN0_NAV2_MODEL_DT:-}" ]]; then
  NAV2_MODEL_DT="$GEN0_NAV2_MODEL_DT"
else
  NAV2_MODEL_DT="$(awk -v frequency="$NAV2_CONTROLLER_FREQUENCY" 'BEGIN { if (frequency > 0.0) printf "%.6f", 1.0 / frequency; else print "0.050000" }')"
fi
ALLOW_GROUND_TRUTH_LOCALIZATION="${GEN0_NAV2_ALLOW_GROUND_TRUTH_LOCALIZATION:-false}"
ODOM_WAIT_TIMEOUT="${GEN0_NAV2_ODOM_WAIT_TIMEOUT:-20}"
TF_WAIT_TIMEOUT="${GEN0_NAV2_TF_WAIT_TIMEOUT:-10}"
TF_PROBE_TIMEOUT="${GEN0_NAV2_TF_PROBE_TIMEOUT:-5}"
TF_TARGET_FRAME="${GEN0_NAV2_TF_TARGET_FRAME:-odom}"
TF_SOURCE_FRAME="${GEN0_NAV2_TF_SOURCE_FRAME:-base_link}"
TERRAIN_WAIT_TIMEOUT="${GEN0_NAV2_TERRAIN_WAIT_TIMEOUT:-20}"
PROJECTED_MAP_WAIT_TIMEOUT="${GEN0_NAV2_PROJECTED_MAP_WAIT_TIMEOUT:-90}"
PROJECTED_MAP_TOPIC="${GEN0_NAV2_PROJECTED_MAP_TOPIC:-/projected_map}"
PROJECTED_MAP_BACKEND="${GEN0_NAV2_PROJECTED_MAP_BACKEND:-python}"
PROJECTED_MAP_FIXED_GEOMETRY="${GEN0_NAV2_PROJECTED_MAP_FIXED_GEOMETRY:-true}"
PROJECTED_MAP_AUTO_WORLD_BOUNDS="${GEN0_NAV2_PROJECTED_MAP_AUTO_WORLD_BOUNDS:-true}"
PROJECTED_MAP_WORLD_BOUNDS_MARGIN="${GEN0_NAV2_PROJECTED_MAP_WORLD_BOUNDS_MARGIN:-15.0}"
PROJECTED_MAP_FIXED_ORIGIN_X="${GEN0_NAV2_PROJECTED_MAP_FIXED_ORIGIN_X:-}"
PROJECTED_MAP_FIXED_ORIGIN_Y="${GEN0_NAV2_PROJECTED_MAP_FIXED_ORIGIN_Y:-}"
PROJECTED_MAP_FIXED_WIDTH="${GEN0_NAV2_PROJECTED_MAP_FIXED_WIDTH:-}"
PROJECTED_MAP_FIXED_HEIGHT="${GEN0_NAV2_PROJECTED_MAP_FIXED_HEIGHT:-}"
PROJECTED_MAP_FIXED_RESOLUTION="${GEN0_NAV2_PROJECTED_MAP_FIXED_RESOLUTION:-0.10}"
PROJECTED_MAP_BOUNDS_MARGIN="${GEN0_NAV2_PROJECTED_MAP_BOUNDS_MARGIN:-5.0}"
NAV2_RVIZ="${GEN0_NAV2_RVIZ:-true}"
NAV2_RVIZ_CONFIG="${GEN0_NAV2_RVIZ_CONFIG:-$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/gen0_nav2_default_view.rviz}"
NAV2_RVIZ_RENDER_ENV="${GEN0_NAV2_RVIZ_RENDER_ENV:-passthrough}"
MAP_TF_WAIT_TIMEOUT="${GEN0_NAV2_MAP_TF_WAIT_TIMEOUT:-10}"
POSE_SANITY_MAX_ABS_XY="${GEN0_NAV2_MAX_ABS_XY:-500.0}"
POSE_SANITY_MAX_ABS_Z="${GEN0_NAV2_MAX_ABS_Z:-20.0}"
ODOM_HEALTH_GUARD="${GEN0_NAV2_ODOM_HEALTH_GUARD:-true}"
REFERENCE_ODOM_TOPIC="${GEN0_NAV2_REFERENCE_ODOM_TOPIC:-/odom}"
MAX_REFERENCE_ODOM_ERROR="${GEN0_NAV2_MAX_REFERENCE_ODOM_ERROR:-3.0}"
MAX_REFERENCE_YAW_ERROR="${GEN0_NAV2_MAX_REFERENCE_YAW_ERROR:-0.75}"
REFERENCE_ODOM_TIMEOUT="${GEN0_NAV2_REFERENCE_ODOM_TIMEOUT:-2.0}"

mkdir -p "$ROS_LOG_DIR"
export ROS_LOG_DIR

case "$PROFILE" in
  legacy)
    DEFAULT_PARAMS_FILE="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/nav2_gen0_params.yaml"
    DEFAULT_COSTMAP_SOURCE="laser_scan"
    DEFAULT_LOCALIZATION_MODE="relocalized"
    DEFAULT_MAP_SOURCE="yaml"
    DEFAULT_MAP_YAML="/tmp/gen0_my_map_nav.yaml"
    DEFAULT_PROJECTED_MAP_UNKNOWN_AS_FREE="false"
    DEFAULT_NAV_TO_POSE_BT="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/behavior_tree/ackermann_forward_recovery.xml"
    DEFAULT_NAV_THROUGH_POSES_BT="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/behavior_tree/navigate_through_poses_ackermann_forward_recovery.xml"
    ;;
  scurm_gen0)
    DEFAULT_PARAMS_FILE="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/config/nav2_gen0_scurm_params.yaml"
    DEFAULT_COSTMAP_SOURCE="scurm_terrain"
    DEFAULT_LOCALIZATION_MODE="odom_only"
    DEFAULT_MAP_SOURCE="projected_map"
    DEFAULT_PROJECTED_MAP_UNKNOWN_AS_FREE="true"
    DEFAULT_NAV_TO_POSE_BT="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/behavior_tree/ackermann_scurm_recovery.xml"
    DEFAULT_NAV_THROUGH_POSES_BT="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/behavior_tree/ackermann_scurm_through_poses.xml"
    case "$WORLD" in
      san_roundabout)
        DEFAULT_MAP_YAML="/home/zjxue2007/gen0_maps/san_roundabout_front3d_demo.yaml"
        ;;
      my_map)
        DEFAULT_MAP_YAML="/home/zjxue2007/maps/my_map.yaml"
        ;;
      *)
        DEFAULT_MAP_YAML=""
        ;;
    esac
    ;;
  *)
    printf 'Invalid GEN0_NAV2_PROFILE=%s. Use legacy or scurm_gen0.\n' "$PROFILE" >&2
    exit 1
    ;;
esac

PARAMS_FILE="${GEN0_NAV2_PARAMS_FILE:-$DEFAULT_PARAMS_FILE}"
COSTMAP_SOURCE="${GEN0_NAV2_COSTMAP_SOURCE:-$DEFAULT_COSTMAP_SOURCE}"
LOCALIZATION_MODE="${GEN0_NAV2_LOCALIZATION_MODE:-$DEFAULT_LOCALIZATION_MODE}"
MAP_SOURCE="${GEN0_NAV2_MAP_SOURCE:-$DEFAULT_MAP_SOURCE}"
MAP_YAML="${GEN0_NAV2_MAP:-$DEFAULT_MAP_YAML}"
PROJECTED_MAP_UNKNOWN_AS_FREE="${GEN0_NAV2_PROJECTED_MAP_UNKNOWN_AS_FREE:-$DEFAULT_PROJECTED_MAP_UNKNOWN_AS_FREE}"
NAV_TO_POSE_BT="${GEN0_NAV2_TO_POSE_BT:-$DEFAULT_NAV_TO_POSE_BT}"
NAV_THROUGH_POSES_BT="${GEN0_NAV2_THROUGH_POSES_BT:-$DEFAULT_NAV_THROUGH_POSES_BT}"

if [[ -n "$NAV2_ODOM_TOPIC_REQUESTED" ]]; then
  NAV2_ODOM_TOPIC="$NAV2_ODOM_TOPIC_REQUESTED"
elif [[ "$PROFILE" == "scurm_gen0" && "$LOCALIZATION_MODE" == "odom_only" ]]; then
  NAV2_ODOM_TOPIC="/gen0_mapping/stable_odom"
else
  NAV2_ODOM_TOPIC="$FAST_LIO_ODOM_TOPIC"
fi

if [[ "$PROFILE" == "scurm_gen0" && "$MAP_SOURCE" == "projected_map" && -z "${GEN0_NAV2_MAP:-}" ]]; then
  MAP_YAML="/tmp/gen0_my_map_nav.yaml"
fi

if [[ "$MAP_SOURCE" == "projected_map" ]]; then
  MAP_SERVER_TOPIC="${GEN0_NAV2_MAP_SERVER_TOPIC:-/map_yaml_unused}"
else
  MAP_SERVER_TOPIC="${GEN0_NAV2_MAP_SERVER_TOPIC:-map}"
fi

log() {
  printf '[%(%F %T)T] %s\n' -1 "$*"
}

source_ros_setup() {
  set +u
  source "$1"
  set -u
}

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    printf 'Required file not found: %s\n' "$path" >&2
    exit 1
  fi
}

wait_for_tf() {
  local target_frame="$1"
  local source_frame="$2"
  local wait_timeout="$3"
  local probe_timeout="$4"
  local deadline=$((SECONDS + wait_timeout))
  local output

  while ((SECONDS < deadline)); do
    output="$(timeout "$probe_timeout" ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null || true)"
    if [[ "$output" == *"Translation:"* ]]; then
      return 0
    fi
    sleep 1
  done

  return 1
}

wait_for_topic_once() {
  local topic="$1"
  local wait_timeout="$2"
  local description="$3"
  local probe_timeout="${GEN0_NAV2_TOPIC_PROBE_TIMEOUT:-5}"
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

    if timeout "$current_probe_timeout" ros2 topic echo --no-daemon --once "$topic" --field header >/dev/null 2>&1; then
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
  local probe_timeout="${GEN0_NAV2_TOPIC_PROBE_TIMEOUT:-5}"
  local deadline=$((SECONDS + wait_timeout))
  local info_file
  local width
  local height
  local remaining
  local current_probe_timeout

  info_file="$(mktemp /tmp/gen0_nav2_grid_info.XXXXXX)"
  log "Waiting for non-empty $description on $topic (timeout=${wait_timeout}s)"
  while ((SECONDS < deadline)); do
    remaining=$((deadline - SECONDS))
    current_probe_timeout="$probe_timeout"
    if ((remaining < probe_timeout)); then
      current_probe_timeout="$remaining"
    fi

    if timeout "$current_probe_timeout" ros2 topic echo --no-daemon --once "$topic" --field info >"$info_file" 2>/dev/null; then
      width="$(awk '/^[[:space:]]*width:/ {print $2; exit}' "$info_file")"
      height="$(awk '/^[[:space:]]*height:/ {print $2; exit}' "$info_file")"
      if [[ "$width" =~ ^[0-9]+$ && "$height" =~ ^[0-9]+$ ]] && ((width > 1 && height > 1)); then
        rm -f "$info_file"
        return 0
      fi
    fi
    sleep 1
  done

  rm -f "$info_file"
  return 1
}

print_3d_slam_start_hint() {
  if [[ "$PROFILE" == "scurm_gen0" && "$LOCALIZATION_MODE" == "odom_only" ]]; then
    printf 'Start the my_map 3D SLAM/Gazebo stack first and wait for projected_map + terrain_map + stable odometry:\n\n' >&2
    printf '  ./run_gen0_3d_slam.sh\n\n' >&2
    printf 'Then start Nav2 from a second terminal.\n' >&2
  else
    printf 'Start relocalized 3D SLAM first and wait for odometry:\n\n' >&2
    printf '  GEN0_WORKSPACE=%q \\\n' "$WORKSPACE" >&2
    printf '  GEN0_RELOCALIZATION=true \\\n' >&2
    printf '  GEN0_PRIOR_MAP_PATH=/tmp/my_map_prior.pcd \\\n' >&2
    printf '  GEN0_MAPPING_DRIVE=false \\\n' >&2
    printf '  ./run_gen0_3d_slam.sh\n\n' >&2
  fi
}

check_nav_pose_sane() {
  local target_frame="$1"
  local source_frame="$2"
  local max_abs_xy="$3"
  local max_abs_z="$4"
  local probe_timeout="$5"
  local output
  local translation
  local x
  local y
  local z

  output="$(timeout "$probe_timeout" ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null || true)"
  translation="$(printf '%s\n' "$output" | awk -F'[][]' '/Translation:/ {print $2; exit}')"
  if [[ -z "$translation" ]]; then
    printf 'Could not sample TF %s -> %s for Nav2 pose sanity check.\n' "$target_frame" "$source_frame" >&2
    return 1
  fi

  IFS=',' read -r x y z <<< "$translation"
  python3 - "$x" "$y" "$z" "$max_abs_xy" "$max_abs_z" <<'PY'
import math
import sys

x, y, z, max_abs_xy, max_abs_z = (float(value.strip()) for value in sys.argv[1:6])
if not all(math.isfinite(value) for value in (x, y, z)):
    print(f'Pose contains non-finite values: x={x}, y={y}, z={z}', file=sys.stderr)
    sys.exit(1)
if abs(x) > max_abs_xy or abs(y) > max_abs_xy or abs(z) > max_abs_z:
    print(
        'Pose is outside sane Nav2 bounds: '
        f'x={x:.2f}, y={y:.2f}, z={z:.2f}, '
        f'max_abs_xy={max_abs_xy:.2f}, max_abs_z={max_abs_z:.2f}',
        file=sys.stderr,
    )
    sys.exit(1)
print(f'Nav2 pose sanity OK: x={x:.2f}, y={y:.2f}, z={z:.2f}')
PY
}

check_projected_map_pose_bounds() {
  local target_frame="$1"
  local source_frame="$2"
  local origin_x="$3"
  local origin_y="$4"
  local width="$5"
  local height="$6"
  local resolution="$7"
  local margin="$8"
  local probe_timeout="$9"
  local output
  local translation
  local x
  local y
  local z

  output="$(timeout "$probe_timeout" ros2 run tf2_ros tf2_echo "$target_frame" "$source_frame" 2>/dev/null || true)"
  translation="$(printf '%s\n' "$output" | awk -F'[][]' '/Translation:/ {print $2; exit}')"
  if [[ -z "$translation" ]]; then
    printf 'Warning: could not sample TF %s -> %s for projected-map bounds check; skipping this guard.\n' "$target_frame" "$source_frame" >&2
    return 0
  fi

  IFS=',' read -r x y z <<< "$translation"
  python3 - "$x" "$y" "$z" "$origin_x" "$origin_y" "$width" "$height" "$resolution" "$margin" <<'PY'
import math
import sys

x, y, z, origin_x, origin_y, width, height, resolution, margin = (
    float(value.strip()) for value in sys.argv[1:10]
)
if not all(math.isfinite(value) for value in (x, y, z)):
    print(f'Pose contains non-finite values: x={x}, y={y}, z={z}', file=sys.stderr)
    sys.exit(1)

min_x = origin_x
max_x = origin_x + width * resolution
min_y = origin_y
max_y = origin_y + height * resolution
if not (min_x - margin <= x <= max_x + margin):
    print(
        'Initial pose is outside the fixed projected-map X bounds: '
        f'x={x:.2f}, bounds=[{min_x:.2f}, {max_x:.2f}], margin={margin:.2f}',
        file=sys.stderr,
    )
    sys.exit(1)
if not (min_y - margin <= y <= max_y + margin):
    print(
        'Initial pose is outside the fixed projected-map Y bounds: '
        f'y={y:.2f}, bounds=[{min_y:.2f}, {max_y:.2f}], margin={margin:.2f}',
        file=sys.stderr,
    )
    sys.exit(1)

print(
    'Projected-map pose bounds OK: '
    f'x={x:.2f}, y={y:.2f}, bounds='
    f'x[{min_x:.2f},{max_x:.2f}] y[{min_y:.2f},{max_y:.2f}]'
)
PY
}

configure_projected_map_fixed_geometry() {
  if [[ "$PROJECTED_MAP_FIXED_GEOMETRY" != "true" ]]; then
    return
  fi

  local have_explicit_geometry="true"
  if [[ -z "$PROJECTED_MAP_FIXED_ORIGIN_X" || -z "$PROJECTED_MAP_FIXED_ORIGIN_Y" || \
        -z "$PROJECTED_MAP_FIXED_WIDTH" || -z "$PROJECTED_MAP_FIXED_HEIGHT" ]]; then
    have_explicit_geometry="false"
  fi

  if [[ "$PROJECTED_MAP_AUTO_WORLD_BOUNDS" == "true" && "$have_explicit_geometry" == "false" ]]; then
    local world_obj="$WORKSPACE/gen0_gz_sim_ros2/gen0_main/worlds/$WORLD/$WORLD.obj"
    if [[ -f "$world_obj" ]]; then
      local geometry
      geometry="$(
        python3 - "$world_obj" "$PROJECTED_MAP_FIXED_RESOLUTION" "$PROJECTED_MAP_WORLD_BOUNDS_MARGIN" <<'PY'
import math
import sys

obj_path, resolution_text, margin_text = sys.argv[1:4]
resolution = float(resolution_text)
margin = float(margin_text)
if resolution <= 0.0:
    raise SystemExit("resolution must be positive")

mins = [math.inf, math.inf, math.inf]
maxs = [-math.inf, -math.inf, -math.inf]
with open(obj_path, "r", encoding="utf-8", errors="ignore") as obj_file:
    for line in obj_file:
        if not line.startswith("v "):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        values = [float(parts[1]), float(parts[2]), float(parts[3])]
        for idx, value in enumerate(values):
            mins[idx] = min(mins[idx], value)
            maxs[idx] = max(maxs[idx], value)

if not all(math.isfinite(value) for value in mins + maxs):
    raise SystemExit("world mesh has no vertices")

spans = [maxs[idx] - mins[idx] for idx in range(3)]
vertical_axis = min(range(3), key=lambda idx: spans[idx])
horizontal_axes = [idx for idx in range(3) if idx != vertical_axis]
map_x_axis = 0 if 0 in horizontal_axes else horizontal_axes[0]
map_y_axis = next(idx for idx in horizontal_axes if idx != map_x_axis)

origin_x = math.floor((mins[map_x_axis] - margin) / resolution) * resolution
origin_y = math.floor((mins[map_y_axis] - margin) / resolution) * resolution
max_x = math.ceil((maxs[map_x_axis] + margin) / resolution) * resolution
max_y = math.ceil((maxs[map_y_axis] + margin) / resolution) * resolution
width = max(1, int(math.ceil((max_x - origin_x) / resolution)))
height = max(1, int(math.ceil((max_y - origin_y) / resolution)))

axis_names = "xyz"
print(
    f"{origin_x:.3f} {origin_y:.3f} {width} {height} "
    f"{axis_names[map_x_axis]} {axis_names[map_y_axis]}"
)
PY
      )"
      read -r PROJECTED_MAP_FIXED_ORIGIN_X PROJECTED_MAP_FIXED_ORIGIN_Y \
        PROJECTED_MAP_FIXED_WIDTH PROJECTED_MAP_FIXED_HEIGHT \
        PROJECTED_MAP_WORLD_AXIS_X PROJECTED_MAP_WORLD_AXIS_Y <<< "$geometry"
      log "Projected-map fixed canvas from world mesh: origin=($PROJECTED_MAP_FIXED_ORIGIN_X,$PROJECTED_MAP_FIXED_ORIGIN_Y), size=${PROJECTED_MAP_FIXED_WIDTH}x${PROJECTED_MAP_FIXED_HEIGHT}@${PROJECTED_MAP_FIXED_RESOLUTION}, axes=${PROJECTED_MAP_WORLD_AXIS_X}/${PROJECTED_MAP_WORLD_AXIS_Y}, margin=${PROJECTED_MAP_WORLD_BOUNDS_MARGIN}m"
      return
    fi
  fi

  PROJECTED_MAP_FIXED_ORIGIN_X="${PROJECTED_MAP_FIXED_ORIGIN_X:--40.0}"
  PROJECTED_MAP_FIXED_ORIGIN_Y="${PROJECTED_MAP_FIXED_ORIGIN_Y:--45.0}"
  PROJECTED_MAP_FIXED_WIDTH="${PROJECTED_MAP_FIXED_WIDTH:-1200}"
  PROJECTED_MAP_FIXED_HEIGHT="${PROJECTED_MAP_FIXED_HEIGHT:-1100}"
}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  printf 'ROS 2 Humble setup not found at /opt/ros/humble/setup.bash\n' >&2
  exit 1
fi

require_file "$WORKSPACE/install/setup.bash"
require_file "$PARAMS_FILE"
require_file "$NAV_TO_POSE_BT"
require_file "$NAV_THROUGH_POSES_BT"
if [[ -z "$MAP_YAML" ]]; then
  printf 'GEN0_NAV2_PROFILE=%s does not know a default map for GEN0_WORLD=%s.\n' "$PROFILE" "$WORLD" >&2
  printf 'Set GEN0_NAV2_MAP=/path/to/aligned_map.yaml.\n' >&2
  exit 1
fi
if [[ "$MAP_YAML" != "/tmp/gen0_my_map_nav.yaml" ]]; then
  require_file "$MAP_YAML"
fi

case "$COSTMAP_SOURCE" in
  laser_scan|scurm_terrain) ;;
  *)
    printf 'Invalid GEN0_NAV2_COSTMAP_SOURCE=%s. Use laser_scan or scurm_terrain.\n' "$COSTMAP_SOURCE" >&2
    exit 1
    ;;
esac

case "$MAP_SOURCE" in
  yaml|projected_map) ;;
  *)
    printf 'Invalid GEN0_NAV2_MAP_SOURCE=%s. Use yaml or projected_map.\n' "$MAP_SOURCE" >&2
    exit 1
    ;;
esac

case "$PROJECTED_MAP_BACKEND" in
  octomap|python) ;;
  *)
    printf 'Invalid GEN0_NAV2_PROJECTED_MAP_BACKEND=%s. Use octomap or python.\n' "$PROJECTED_MAP_BACKEND" >&2
    exit 1
    ;;
esac

case "$LOCALIZATION_MODE" in
  relocalized|odom_only) ;;
  *)
    printf 'Invalid GEN0_NAV2_LOCALIZATION_MODE=%s. Use relocalized or odom_only.\n' "$LOCALIZATION_MODE" >&2
    exit 1
    ;;
esac

source_ros_setup /opt/ros/humble/setup.bash
source_ros_setup "$WORKSPACE/install/setup.bash"
configure_projected_map_fixed_geometry

log "Preflight: checking for existing Nav2 and ground-truth localization nodes"
existing_nav2="$(
  timeout 5s ros2 node list --no-daemon 2>/dev/null \
    | grep -E '(^/map_server$|^/controller_server$|^/planner_server$|^/bt_navigator$|^/lifecycle_manager_navigation$)' \
    || true
)"
if [[ -n "$existing_nav2" ]]; then
  printf 'Existing Nav2 nodes are already running:\n%s\n\n' "$existing_nav2" >&2
  printf 'Stop the existing gen0_navigation launch before starting another one.\n' >&2
  exit 1
fi

existing_ground_truth="$(
  timeout 5s ros2 node list --no-daemon 2>/dev/null \
    | grep -E '^/pose_publisher$' \
    || true
)"
if [[ -n "$existing_ground_truth" && "$ALLOW_GROUND_TRUTH_LOCALIZATION" != "true" ]]; then
  printf 'Ground-truth localization node is running:\n%s\n\n' "$existing_ground_truth" >&2
  printf '/pose_publisher publishes Gazebo-derived map -> odom and conflicts with ICP/FAST-LIO relocalized Nav2.\n' >&2
  printf 'Stop the 3D SLAM stack and restart it with GEN0_GROUND_TRUTH_LOCALIZATION=false, or set GEN0_NAV2_ALLOW_GROUND_TRUTH_LOCALIZATION=true only for explicit ground-truth tests.\n' >&2
  exit 1
fi

if ! wait_for_topic_once "$NAV2_ODOM_TOPIC" "$ODOM_WAIT_TIMEOUT" "Nav2 odometry"; then
  printf 'Timed out waiting for %s.\n\n' "$NAV2_ODOM_TOPIC" >&2
  print_3d_slam_start_hint
  exit 1
fi

if [[ "$PROFILE" == "scurm_gen0" ]]; then
  log "Using SCURM-aligned Gen0 Nav2 profile: params=$PARAMS_FILE, map_source=$MAP_SOURCE, map=$MAP_YAML, bt=$NAV_TO_POSE_BT"
fi

log "Waiting for TF $TF_TARGET_FRAME -> $TF_SOURCE_FRAME (timeout=${TF_WAIT_TIMEOUT}s)"
if ! wait_for_tf "$TF_TARGET_FRAME" "$TF_SOURCE_FRAME" "$TF_WAIT_TIMEOUT" "$TF_PROBE_TIMEOUT"; then
  printf 'Timed out waiting for TF %s -> %s. Nav2 local costmap cannot activate without this TF.\n' "$TF_TARGET_FRAME" "$TF_SOURCE_FRAME" >&2
  exit 1
fi
log "TF $TF_TARGET_FRAME -> $TF_SOURCE_FRAME is available."

PUBLISH_IDENTITY_MAP_TO_ODOM=false
if [[ "$LOCALIZATION_MODE" == "odom_only" ]]; then
  if [[ "$MAP_SOURCE" == "projected_map" && "$PROJECTED_MAP_BACKEND" == "octomap" ]]; then
    log "Using odom-only Nav2 mode: projected-map backend already publishes map -> odom; Nav2 will not duplicate the identity TF."
  else
    PUBLISH_IDENTITY_MAP_TO_ODOM=true
    log "Using odom-only Nav2 mode: gen0_navigation will publish identity map -> odom; no prior map or ICP relocalization is required."
  fi
else
  log "Waiting for TF map -> $TF_SOURCE_FRAME (timeout=${MAP_TF_WAIT_TIMEOUT}s)"
  if ! wait_for_tf map "$TF_SOURCE_FRAME" "$MAP_TF_WAIT_TIMEOUT" "$TF_PROBE_TIMEOUT"; then
    printf 'Timed out waiting for TF map -> %s. Nav2 global costmaps require relocalized map -> odom before launch.\n' "$TF_SOURCE_FRAME" >&2
    printf 'If you do not have a clean prior map, restart SLAM with GEN0_RELOCALIZATION=false and start Nav2 with GEN0_NAV2_LOCALIZATION_MODE=odom_only.\n' >&2
    exit 1
  fi
  log "TF map -> $TF_SOURCE_FRAME is available."

  if ! check_nav_pose_sane map "$TF_SOURCE_FRAME" "$POSE_SANITY_MAX_ABS_XY" "$POSE_SANITY_MAX_ABS_Z" "$TF_PROBE_TIMEOUT"; then
    printf '\nFAST-LIO/ICP localization is not sane enough for Nav2.\n' >&2
    printf 'Stop Nav2/3D SLAM, restart relocalized SLAM, and wait for a stable map -> %s pose before sending goals.\n' "$TF_SOURCE_FRAME" >&2
    printf 'If your prior maps are unusable, use GEN0_NAV2_LOCALIZATION_MODE=odom_only for short-distance validation.\n' >&2
    printf 'Override GEN0_NAV2_MAX_ABS_XY or GEN0_NAV2_MAX_ABS_Z only for a deliberately larger test map.\n' >&2
    exit 1
  fi
fi

if [[ "$COSTMAP_SOURCE" == "scurm_terrain" ]]; then
  if ! wait_for_topic_once /gen0_mapping/terrain_map "$TERRAIN_WAIT_TIMEOUT" "SCURM local terrain map"; then
    printf 'Timed out waiting for /gen0_mapping/terrain_map. Keep the relocalized 3D SLAM stack running until terrain_analysis publishes.\n' >&2
    exit 1
  fi
fi

if [[ "$MAP_SOURCE" == "projected_map" ]]; then
  if ! wait_for_occupancy_grid_nonempty "$PROJECTED_MAP_TOPIC" "$PROJECTED_MAP_WAIT_TIMEOUT" "online projected occupancy map"; then
    printf 'Timed out waiting for non-empty %s. Keep 3D SLAM and the projected map backend running until terrain projection is populated.\n' "$PROJECTED_MAP_TOPIC" >&2
    exit 1
  fi
fi

if [[ "$PROFILE" == "scurm_gen0" && "$MAP_SOURCE" == "projected_map" && "$PROJECTED_MAP_BACKEND" == "octomap" ]]; then
  log "Waiting for SCURM map -> odom TF from the projected-map backend (timeout=${MAP_TF_WAIT_TIMEOUT}s)"
  if ! wait_for_tf map odom "$MAP_TF_WAIT_TIMEOUT" "$TF_PROBE_TIMEOUT"; then
    printf 'Timed out waiting for TF map -> odom from the projected-map backend.\n' >&2
    printf 'Start the SCURM-style mapping stack first so octomap_server can publish the map frame.\n' >&2
    exit 1
  fi
  log "TF map -> odom is available from the projected-map backend."
fi

if [[ "$PROFILE" == "scurm_gen0" && "$MAP_SOURCE" == "projected_map" && "$PROJECTED_MAP_FIXED_GEOMETRY" == "true" ]]; then
  log "Checking initial pose against fixed projected-map bounds."
  if ! check_projected_map_pose_bounds \
      "$TF_TARGET_FRAME" \
      "$TF_SOURCE_FRAME" \
      "$PROJECTED_MAP_FIXED_ORIGIN_X" \
      "$PROJECTED_MAP_FIXED_ORIGIN_Y" \
      "$PROJECTED_MAP_FIXED_WIDTH" \
      "$PROJECTED_MAP_FIXED_HEIGHT" \
      "$PROJECTED_MAP_FIXED_RESOLUTION" \
      "$PROJECTED_MAP_BOUNDS_MARGIN" \
      "$TF_PROBE_TIMEOUT"; then
    printf '\nThe current vehicle pose is outside the fixed projected-map canvas.\n' >&2
    printf 'Stop and restart Gazebo/3D SLAM to reset the vehicle, or enlarge the projected-map canvas deliberately.\n' >&2
    printf 'Current canvas: origin=(%s,%s), size=%sx%s cells at %sm/cell, margin=%sm.\n' \
      "$PROJECTED_MAP_FIXED_ORIGIN_X" \
      "$PROJECTED_MAP_FIXED_ORIGIN_Y" \
      "$PROJECTED_MAP_FIXED_WIDTH" \
      "$PROJECTED_MAP_FIXED_HEIGHT" \
      "$PROJECTED_MAP_FIXED_RESOLUTION" \
      "$PROJECTED_MAP_BOUNDS_MARGIN" >&2
    exit 1
  fi
fi

if [[ "$ODOM_HEALTH_GUARD" == "true" ]]; then
  log "Nav2 odom health guard: reference=$REFERENCE_ODOM_TOPIC, max_xy_error=${MAX_REFERENCE_ODOM_ERROR}m, max_yaw_error=${MAX_REFERENCE_YAW_ERROR}rad"
else
  log "Nav2 odom health guard: disabled"
  REFERENCE_ODOM_TOPIC=""
  MAX_REFERENCE_ODOM_ERROR="0.0"
  MAX_REFERENCE_YAW_ERROR="0.0"
fi

nav2_launch=(
  ros2 launch gen0_main gen0_navigation.launch.py
  params_file:="$PARAMS_FILE" \
  use_respawn:="$USE_RESPAWN" \
  nav2_controller_frequency:="$NAV2_CONTROLLER_FREQUENCY" \
  nav2_model_dt:="$NAV2_MODEL_DT" \
  nav2_smoothing_frequency:="$NAV2_SMOOTHING_FREQUENCY" \
  publish_identity_map_to_odom:="$PUBLISH_IDENTITY_MAP_TO_ODOM" \
  costmap_source:="$COSTMAP_SOURCE" \
  map_source:="$MAP_SOURCE" \
  map_server_topic:="$MAP_SERVER_TOPIC" \
  projected_map_topic:="$PROJECTED_MAP_TOPIC" \
  projected_map_unknown_as_free:="$PROJECTED_MAP_UNKNOWN_AS_FREE" \
  projected_map_fixed_geometry:="$PROJECTED_MAP_FIXED_GEOMETRY" \
  projected_map_fixed_origin_x:="$PROJECTED_MAP_FIXED_ORIGIN_X" \
  projected_map_fixed_origin_y:="$PROJECTED_MAP_FIXED_ORIGIN_Y" \
  projected_map_fixed_width:="$PROJECTED_MAP_FIXED_WIDTH" \
  projected_map_fixed_height:="$PROJECTED_MAP_FIXED_HEIGHT" \
  projected_map_fixed_resolution:="$PROJECTED_MAP_FIXED_RESOLUTION" \
  odom_topic:="$NAV2_ODOM_TOPIC" \
  max_reference_odom_error:="$MAX_REFERENCE_ODOM_ERROR" \
  max_reference_yaw_error:="$MAX_REFERENCE_YAW_ERROR" \
  reference_odom_timeout:="$REFERENCE_ODOM_TIMEOUT" \
  rviz:="$NAV2_RVIZ" \
  rviz_config:="$NAV2_RVIZ_CONFIG" \
  rviz_render_env:="$NAV2_RVIZ_RENDER_ENV" \
  default_nav_to_pose_bt_xml:="$NAV_TO_POSE_BT" \
  default_nav_through_poses_bt_xml:="$NAV_THROUGH_POSES_BT" \
  map:="$MAP_YAML"
)
if [[ -n "$REFERENCE_ODOM_TOPIC" ]]; then
  nav2_launch+=(reference_odom_topic:="$REFERENCE_ODOM_TOPIC")
fi

log "Starting Gen0 Nav2: profile=$PROFILE, map_source=$MAP_SOURCE, map=$MAP_YAML, params=$PARAMS_FILE, odom_topic=$NAV2_ODOM_TOPIC, localization_mode=$LOCALIZATION_MODE, costmap_source=$COSTMAP_SOURCE, controller_frequency=$NAV2_CONTROLLER_FREQUENCY, model_dt=$NAV2_MODEL_DT, smoothing_frequency=$NAV2_SMOOTHING_FREQUENCY, projected_map_unknown_as_free=$PROJECTED_MAP_UNKNOWN_AS_FREE, projected_map_fixed=${PROJECTED_MAP_FIXED_GEOMETRY}:${PROJECTED_MAP_FIXED_WIDTH}x${PROJECTED_MAP_FIXED_HEIGHT}@${PROJECTED_MAP_FIXED_RESOLUTION}, projected_map_origin=(${PROJECTED_MAP_FIXED_ORIGIN_X},${PROJECTED_MAP_FIXED_ORIGIN_Y}), rviz=$NAV2_RVIZ, rviz_config=$NAV2_RVIZ_CONFIG, command_topic=/cmd_vel"
log "RViz render env: $NAV2_RVIZ_RENDER_ENV"
exec "${nav2_launch[@]}"
