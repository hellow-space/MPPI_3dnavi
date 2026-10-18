#!/usr/bin/env bash

set -euo pipefail

SESSION="mppi_nav_height_obstacle"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
printf -v PROJECT_ROOT_Q '%q' "$PROJECT_ROOT"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "发现旧的 $SESSION，正在关闭..."
    while read -r pane; do
        tmux send-keys -t "$pane" C-c 2>/dev/null || true
    done < <(tmux list-panes -s -t "$SESSION" -F '#{pane_id}' 2>/dev/null || true)
    sleep 2
    if ! tmux kill-session -t "$SESSION" 2>/dev/null && \
        tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "关闭失败：$SESSION 仍在运行，未启动新会话" >&2
        exit 1
    fi
fi

tmux new-session -d -s "$SESSION" -n main
tmux set-option -t "$SESSION" mouse on
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format ' #{pane_index} | #{pane_title} '

tmux split-window -h -t "$SESSION:0.0"
tmux split-window -v -t "$SESSION:0.1"
tmux select-layout -t "$SESSION:0" tiled

tmux select-pane -t "$SESSION:0.0" -T "Navigation HeightObstacle"
tmux send-keys -t "$SESSION:0.0" "cd $PROJECT_ROOT_Q" C-m
tmux send-keys -t "$SESSION:0.0" 'source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.0" \
    "ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=$PROJECT_ROOT_Q/src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml" C-m

tmux select-pane -t "$SESSION:0.1" -T "keep_zero"
tmux send-keys -t "$SESSION:0.1" "cd $PROJECT_ROOT_Q/src" C-m
tmux send-keys -t "$SESSION:0.1" 'python3 keep_zero.py' C-m

tmux select-pane -t "$SESSION:0.2" -T "initial_pose_tf"
tmux send-keys -t "$SESSION:0.2" "cd $PROJECT_ROOT_Q" C-m
tmux send-keys -t "$SESSION:0.2" 'source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.2" \
    "python3 $PROJECT_ROOT_Q/scripts/initial_pose_tf.py" C-m

tmux select-pane -t "$SESSION:0.0"
gnome-terminal --window \
    --title="MPPI Navigation HeightObstacle" \
    -- bash -lc "tmux attach-session -t $SESSION"
