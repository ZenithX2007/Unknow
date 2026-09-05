#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Fixed integrated validation profile: Nav2 plus pedestrian avoidance and
# YOLO/point-cloud trash perception, without the EPSILON control sidecar.
export GEN0_WORLD="${GEN0_WORLD:-my_map}"
export GEN0_ACTOR_SOFT_STOP="${GEN0_ACTOR_SOFT_STOP:-true}"
export GEN0_ACTOR_SOFT_STOP_MARGIN="${GEN0_ACTOR_SOFT_STOP_MARGIN:-1.5}"
export GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN="${GEN0_ACTOR_SOFT_STOP_RELEASE_MARGIN:-1.9}"
export GEN0_TRASH_CLEANUP="${GEN0_TRASH_CLEANUP:-false}"
export GEN0_TRASH_FUSION_DETECTION="${GEN0_TRASH_FUSION_DETECTION:-true}"
export GEN0_CAMERA_VIEW="${GEN0_CAMERA_VIEW:-true}"
export GEN0_START_EPSILON="${GEN0_START_EPSILON:-false}"
export GEN0_START_NAV2="${GEN0_START_NAV2:-true}"
export GEN0_NAV2_REFERENCE_ODOM_TOPIC="${GEN0_NAV2_REFERENCE_ODOM_TOPIC:-/gen0_mapping/stable_odom}"

"$SCRIPT_DIR/stop_gen0_full_stack.sh"
exec "$SCRIPT_DIR/run_gen0_full_stack.sh"
