#!/usr/bin/env bash

set -euo pipefail

SESSION="mppi_nav_height_jump"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "$SESSION 当前没有运行"
    exit 0
fi

echo "正在关闭 HeightJump Navigation、keep_zero 和 initial_pose_tf..."

while read -r pane; do
    tmux send-keys -t "$pane" C-c 2>/dev/null || true
done < <(tmux list-panes -s -t "$SESSION" -F '#{pane_id}' 2>/dev/null || true)

sleep 2
if ! tmux kill-session -t "$SESSION" 2>/dev/null && \
    tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "关闭失败：$SESSION 仍在运行" >&2
    exit 1
fi

echo "已关闭"
