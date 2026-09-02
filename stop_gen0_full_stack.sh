#!/usr/bin/env bash
set -Eeuo pipefail

QUIET=false
if [[ "${1:-}" == "--quiet" ]]; then
  QUIET=true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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
  "run_gen0_full_stack.sh"
  "ros2 launch epsilon_planning gen0_navigation_epsilon.launch.py"
)

for pattern in "${patterns[@]}"; do
  stop_pattern "$pattern" "$pattern"
done

sleep 2

"$SCRIPT_DIR/stop_gen0_epsilon.sh" --quiet || true
"$SCRIPT_DIR/stop_gen0_nav2.sh" --quiet || true
"$SCRIPT_DIR/stop_gen0_3d_slam.sh" --quiet || true

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
