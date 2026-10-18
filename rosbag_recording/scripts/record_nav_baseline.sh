#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/nav_baseline"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/nav_baseline_${STAMP}"

mkdir -p "$OUT_ROOT"

TOPICS=(
  /tf
  /tf_static
  /aft_mapped_to_init
  /path
  /amcl_pose
  /command/cmd_twist
  /cmd_vel
  /goal_pose
  /plan
  /transformed_global_plan
  /trajectories
)

echo "=========================================="
echo " NAV BASELINE ROSBAG"
echo "=========================================="
echo "Output: $OUT_DIR"
echo
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Press Ctrl-C once to stop safely."
echo "=========================================="

ros2 bag record --output "$OUT_DIR" "${TOPICS[@]}"
