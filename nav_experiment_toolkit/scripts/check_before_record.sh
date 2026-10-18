#!/usr/bin/env bash
set -u

PROFILE="${1:-baseline}"
if [[ $# -gt 1 || ( "$PROFILE" != baseline && "$PROFILE" != elevation ) ]]; then
  echo "Usage: $0 [baseline|elevation]" >&2
  exit 2
fi

REQ=(
 /map /tf /tf_static /aft_mapped_to_init /path /amcl_pose
 /command/cmd_twist /cmd_vel /goal_pose /plan
 /transformed_global_plan /trajectories
 /local_costmap/costmap /global_costmap/costmap
)

if [[ "$PROFILE" == elevation ]]; then
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  source "${SCRIPT_DIR}/../config/elevation_recording_topics.sh"
  REQ=("${ELEVATION_REQUIRED_TOPICS[@]}")
fi

topics="$(ros2 topic list --include-hidden-topics)" || exit 1
bad=0
echo "========== Required topics =========="
for t in "${REQ[@]}"; do
  if grep -Fxq "$t" <<< "$topics"; then
    echo "[OK]      $t"
  else
    echo "[MISSING] $t"
    bad=1
  fi
done

if [[ "$PROFILE" == elevation ]]; then
  echo "========== Task topics (may appear after sending a goal) =========="
  for t in "${ELEVATION_TASK_TOPICS[@]}"; do
    if grep -Fxq "$t" <<< "$topics"; then
      echo "[OK]      $t"
    else
      echo "[WAITING] $t"
    fi
  done
  echo "Topic presence alone does not prove messages are being received."
  echo "Check: ros2 topic hz /cloud_registered"
  echo "Check: ros2 topic hz $ELEVATION_TOPIC"
fi

echo
echo "========== Key message check =========="
echo "Run these if needed:"
echo "  ros2 topic echo /map --once"
echo "  ros2 topic hz /aft_mapped_to_init"
echo "  ros2 topic hz /cmd_vel"
echo
echo "Known expected rates:"
echo "  /aft_mapped_to_init  ~10 Hz"
echo "  /cmd_vel             ~20 Hz"
echo "  /local_costmap/costmap  ~10 Hz"
echo "  /global_costmap/costmap  ~1 Hz"

exit "$bad"
