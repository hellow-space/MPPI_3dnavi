#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/nav_baseline"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/nav_baseline_${STAMP}"

mkdir -p "$OUT_ROOT"

TOPICS=(
  /map
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
  /local_costmap/costmap
  /global_costmap/costmap
)

# Optional action-status topic: useful if this Nav2 exposes it.
if ros2 topic list | grep -Fxq "/navigate_to_pose/_action/status"; then
  TOPICS+=(/navigate_to_pose/_action/status)
fi

echo "=========================================="
echo " NAVIGATION EXPERIMENT RECORDING"
echo "=========================================="
echo "Output: $OUT_DIR"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Start one navigation task per bag."
echo "After arrival press Ctrl-C once."
echo "=========================================="

ros2 bag record --output "$OUT_DIR" "${TOPICS[@]}"
