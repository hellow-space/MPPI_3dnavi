#!/usr/bin/env bash
# HeightJump / HeightObstacle 通用录包；在已配置 ROS 环境的宿主机终端执行。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUT_ROOT="${ROOT_DIR}/bags/nav_elevation"
STAMP="$(date +'%Y-%m-%d_%H-%M-%S')"
OUT_DIR="${OUT_ROOT}/nav_elevation_${STAMP}"
source "${ROOT_DIR}/config/elevation_recording_topics.sh"

mkdir -p "$OUT_ROOT"

TOPICS=(
  "${ELEVATION_REQUIRED_TOPICS[@]}"
  "${ELEVATION_TASK_TOPICS[@]}"
)

echo "录包保存到：$OUT_DIR"
echo '等待录制器建立订阅后再发导航目标，按 Ctrl+C 结束录包。'
# 未发布的话题由 rosbag 持续发现；HeightJump 缺少高度差图也可录制。

exec ros2 bag record --include-hidden-topics --output "$OUT_DIR" "${TOPICS[@]}"
