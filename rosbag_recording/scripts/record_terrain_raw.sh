#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/terrain_raw"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/terrain_raw_${STAMP}"

mkdir -p "$OUT_ROOT"

TOPICS=(
  /cloud_registered
  /aft_mapped_to_init
  /tf
  /tf_static
)

echo "=========================================="
echo " TERRAIN RAW ROSBAG"
echo "=========================================="
echo "Output: $OUT_DIR"
echo
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Use this bag for offline Elevation Map replay."
echo "Press Ctrl-C once to stop safely."
echo "=========================================="

ros2 bag record --output "$OUT_DIR" "${TOPICS[@]}"
