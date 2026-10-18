#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 BAG_DIR"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RVIZ="${ROOT_DIR}/config/nav_replay.rviz"
ROBOT_URDF="${ROOT_DIR}/../src/fishbot_navigation2/urdf/navigation_footprint.urdf"

if [ ! -f "$1/metadata.yaml" ]; then
  echo "[ERROR] metadata.yaml not found in $1" >&2
  exit 1
fi
BAG="$(realpath "$1")"

for required_file in "$RVIZ" "$ROBOT_URDF"; do
  if [ ! -f "$required_file" ]; then
    echo "[ERROR] required file not found: $required_file" >&2
    exit 1
  fi
done

ROBOT_STATE_PUBLISHER_PREFIX="$(ros2 pkg prefix robot_state_publisher)"
ROBOT_STATE_PUBLISHER="${ROBOT_STATE_PUBLISHER_PREFIX}/lib/robot_state_publisher/robot_state_publisher"
if [ ! -x "$ROBOT_STATE_PUBLISHER" ]; then
  echo "[ERROR] robot_state_publisher executable not found: $ROBOT_STATE_PUBLISHER" >&2
  exit 1
fi

stop_process() {
  local pid=${1:-}
  if [ -z "$pid" ]; then
    return
  fi
  if kill -0 "$pid" 2>/dev/null; then
    kill "$pid" >/dev/null 2>&1 || true
  fi
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  stop_process "${ROBOT_STATE_PUBLISHER_PID:-}"
  stop_process "${RVIZ_PID:-}"
}
trap cleanup EXIT INT TERM

echo "Starting RViz standard replay..."
rviz2 -d "$RVIZ" --ros-args -p use_sim_time:=true &
RVIZ_PID=$!
"$ROBOT_STATE_PUBLISHER" --ros-args \
  -r robot_description:=/navigation_footprint/robot_description \
  -p use_sim_time:=true \
  -p robot_description:="$(<"$ROBOT_URDF")" &
ROBOT_STATE_PUBLISHER_PID=$!
sleep 3

if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
  wait "$RVIZ_PID" 2>/dev/null || true
  echo "[ERROR] RViz exited during startup" >&2
  exit 1
fi
if ! kill -0 "$ROBOT_STATE_PUBLISHER_PID" 2>/dev/null; then
  wait "$ROBOT_STATE_PUBLISHER_PID" 2>/dev/null || true
  echo "[ERROR] robot_state_publisher exited during startup" >&2
  exit 1
fi

ros2 bag play "$BAG" --clock
