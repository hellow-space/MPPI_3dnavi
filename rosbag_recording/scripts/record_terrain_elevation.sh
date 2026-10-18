#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/terrain_elevation"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/terrain_elevation_${STAMP}"

mkdir -p "$OUT_ROOT"

ELEVATION_TOPIC="${ELEVATION_TOPIC:-/elevation_mapping_node/elevation_map}"

if ! ros2 topic list | grep -Fxq "$ELEVATION_TOPIC"; then
  echo "[ERROR] Elevation topic not found: $ELEVATION_TOPIC"
  echo "Use:"
  echo "  ELEVATION_TOPIC=/your/topic ./record_terrain_elevation.sh"
  exit 1
fi

TOPICS=(
  /cloud_registered
  "$ELEVATION_TOPIC"
  /aft_mapped_to_init
  /tf
  /tf_static
)

echo "=========================================="
echo " TERRAIN + ELEVATION ROSBAG"
echo "=========================================="
echo "Output: $OUT_DIR"
echo
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Press Ctrl-C once to stop safely."
echo "=========================================="

ros2 bag record --output "$OUT_DIR" "${TOPICS[@]}"
