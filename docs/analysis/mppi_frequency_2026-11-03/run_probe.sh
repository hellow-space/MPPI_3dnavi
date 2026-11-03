#!/usr/bin/env bash
# Offline diagnostic. Never launches navigation or publishes motion commands.
set -eo pipefail
probe_source_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
probe_root_dir=$(cd "$probe_source_dir/../../.." && pwd)
probe_build_dir=${MPPI_PROBE_BUILD_DIR:-/tmp/mppi_frequency_probe_build}
source /opt/ros/humble/setup.bash
source "$probe_root_dir/install/setup.bash"
export ROS_DOMAIN_ID=142
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cmake -S "$probe_source_dir" -B "$probe_build_dir" -DCMAKE_BUILD_TYPE=Release
cmake --build "$probe_build_dir" -j2
probe_binary="$probe_build_dir/probe"
probe_params="$probe_source_dir/baseline_controller.yaml"
"$probe_binary" --ros-args --params-file "$probe_params" --log-level warn > "$probe_source_dir/current_visualization.log" 2>&1
"$probe_binary" --ros-args --params-file "$probe_params" -p FollowPath.TrajectoryVisualizer.trajectory_step:=20 -p FollowPath.TrajectoryVisualizer.time_step:=5 --log-level warn > "$probe_source_dir/sparse_visualization.log" 2>&1
"$probe_binary" --ros-args --params-file "$probe_params" -p FollowPath.visualize:=false --log-level warn > "$probe_source_dir/no_visualization.log" 2>&1
# Ablation only in this synthetic diagnostic; production terrain critic stays enabled.
"$probe_binary" --ros-args --params-file "$probe_params" -p FollowPath.visualize:=false -p FollowPath.HeightJumpCritic.enabled:=false --log-level warn > "$probe_source_dir/no_height_diagnostic.log" 2>&1
"$probe_binary" --ros-args --params-file "$probe_params" -p FollowPath.TrajectoryVisualizer.trajectory_step:=20 -p FollowPath.TrajectoryVisualizer.time_step:=5 -p frequency_probe.periodic:=true --log-level warn > "$probe_source_dir/sparse_periodic_20hz.log" 2>&1
/usr/bin/python3 "$probe_source_dir/summarize.py"
