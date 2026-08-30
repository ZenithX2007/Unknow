#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="${GEN0_WORKSPACE:-$SCRIPT_DIR}"

cd "$WORKSPACE"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

./stop_gen0_3d_slam.sh

# Keep mapping_drive disabled here so Nav2 owns vehicle control.
GEN0_WORKSPACE="$PWD" \
GEN0_WORLD=my_map \
GEN0_MAPPING_DRIVE=false \
GEN0_TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-false}" \
GEN0_GAZEBO_GUI="${GEN0_GAZEBO_GUI:-true}" \
GEN0_GAZEBO_RENDER_ENV="${GEN0_GAZEBO_RENDER_ENV:-unset}" \
GEN0_GPU_ADAPTER="${GEN0_GPU_ADAPTER:-NVIDIA}" \
GEN0_RVIZ=true \
GEN0_RVIZ_RENDER_ENV="${GEN0_RVIZ_RENDER_ENV:-passthrough}" \
GEN0_PROJECTED_MAP_BACKEND=python \
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

GEN0_WORKSPACE="$PWD" \
GEN0_NAV2_PROFILE=scurm_gen0 \
GEN0_WORLD=my_map \
GEN0_NAV2_USE_RESPAWN=false \
GEN0_NAV2_RVIZ_RENDER_ENV="${GEN0_NAV2_RVIZ_RENDER_ENV:-passthrough}" \
GEN0_NAV2_ODOM_WAIT_TIMEOUT="${GEN0_NAV2_ODOM_WAIT_TIMEOUT:-90}" \
GEN0_NAV2_PROJECTED_MAP_WAIT_TIMEOUT="${GEN0_NAV2_PROJECTED_MAP_WAIT_TIMEOUT:-120}" \
GEN0_NAV2_TF_WAIT_TIMEOUT="${GEN0_NAV2_TF_WAIT_TIMEOUT:-30}" \
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
