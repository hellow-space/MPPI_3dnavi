#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/nav_elevation"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/nav_elevation_${STAMP}"
source "${ROOT_DIR}/config/elevation_recording_topics.sh"

AVAILABLE_TOPICS="$(ros2 topic list --include-hidden-topics)"
MISSING=0
for topic in "${ELEVATION_REQUIRED_TOPICS[@]}"; do
  if ! grep -Fxq "$topic" <<< "$AVAILABLE_TOPICS"; then
    echo "[ERROR] required topic not found: $topic" >&2
    MISSING=1
  fi
done
if (( MISSING )); then exit 1; fi

mkdir -p "$OUT_ROOT"

TOPICS=(
  "${ELEVATION_REQUIRED_TOPICS[@]}"
  "${ELEVATION_TASK_TOPICS[@]}"
)

echo "=========================================="
echo " NAVIGATION ELEVATION RECORDING"
echo "=========================================="
echo "Output: $OUT_DIR"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Wait for recorder subscriptions, then start one navigation task per bag."
echo "Task topics not yet present will be discovered automatically."
echo "Keep 5-10 seconds before/after the task, then press Ctrl-C once."
echo "=========================================="

ros2 bag record --include-hidden-topics --output "$OUT_DIR" "${TOPICS[@]}"
