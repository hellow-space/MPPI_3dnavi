#!/usr/bin/env bash
set -u

echo "=========================================="
echo " ROS2 recording topic check"
echo "=========================================="

required_topics=(
  "/tf"
  "/tf_static"
  "/aft_mapped_to_init"
  "/path"
  "/amcl_pose"
  "/command/cmd_twist"
  "/cmd_vel"
  "/goal_pose"
  "/plan"
  "/transformed_global_plan"
  "/trajectories"
  "/cloud_registered"
)

all_topics="$(ros2 topic list 2>/dev/null || true)"
missing=0

for topic in "${required_topics[@]}"; do
  if grep -Fxq "$topic" <<< "$all_topics"; then
    printf "[OK]      %s\n" "$topic"
  else
    printf "[MISSING] %s\n" "$topic"
    missing=1
  fi
done

echo
echo "Expected rates:"
echo "  /cloud_registered      ~10 Hz"
echo "  /aft_mapped_to_init    ~10 Hz"
echo "  /cmd_vel               ~20 Hz"
echo
echo "Manual checks:"
echo "  ros2 topic hz /cloud_registered"
echo "  ros2 topic hz /aft_mapped_to_init"
echo "  ros2 topic hz /cmd_vel"

if [ "$missing" -eq 0 ]; then
  echo
  echo "[PASS] All planned topics exist."
else
  echo
  echo "[WARN] Some topics are missing. Check whether their nodes are running."
fi
