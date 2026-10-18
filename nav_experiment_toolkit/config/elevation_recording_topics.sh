# Shared by the elevation recorder and preflight check. Source with bash.
ELEVATION_TOPIC="${ELEVATION_TOPIC:-/elevation_mapping_node/elevation_map}"

# These streams should exist before starting a height-obstacle navigation run.
ELEVATION_REQUIRED_TOPICS=(
  /cloud_registered
  "$ELEVATION_TOPIC"
  /height_obstacle_layer/height_difference_map
  /tf
  /tf_static
  /aft_mapped_to_init
  /map
  /local_costmap/costmap
  /global_costmap/costmap
)

# Task-dependent topics may first appear after a goal is sent. Keep them in the
# explicit recording list so rosbag discovery can subscribe when they appear.
ELEVATION_TASK_TOPICS=(
  /command/cmd_twist
  /cmd_vel
  /goal_pose
  /plan
  /transformed_global_plan
  /navigate_to_pose/_action/status
  /follow_path/_action/status
  /custom_path/raw_path
  /custom_path/preview_path
  /custom_path/status
)
