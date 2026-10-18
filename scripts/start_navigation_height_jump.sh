#!/usr/bin/env bash

set -euo pipefail

SESSION="mppi_nav_height_jump"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
printf -v PROJECT_ROOT_Q '%q' "$PROJECT_ROOT"

# 避免重复启动同名会话，并先让 ROS/Python 进程正常处理中断。
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
tmux set-option -t "$SESSION" pane-border-format \
    ' #{pane_index} | #{pane_title} '

# 创建三个窗格。
tmux split-window -h -t "$SESSION:0.0"
tmux split-window -v -t "$SESSION:0.1"
tmux select-layout -t "$SESSION:0" tiled

# pane 0：Navigation + HeightJumpCritic
tmux select-pane -t "$SESSION:0.0" -T "Navigation HeightJump"
tmux send-keys -t "$SESSION:0.0" "cd $PROJECT_ROOT_Q" C-m
tmux send-keys -t "$SESSION:0.0" 'source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.0" \
    "ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=$PROJECT_ROOT_Q/src/fishbot_navigation2/config/nav2_params_height_jump.yaml" C-m

# pane 1：keep_zero
tmux select-pane -t "$SESSION:0.1" -T "keep_zero"
tmux send-keys -t "$SESSION:0.1" "cd $PROJECT_ROOT_Q/src" C-m
tmux send-keys -t "$SESSION:0.1" 'python3 keep_zero.py' C-m

# pane 2：初始位姿 TF
tmux select-pane -t "$SESSION:0.2" -T "initial_pose_tf"
tmux send-keys -t "$SESSION:0.2" "cd $PROJECT_ROOT_Q" C-m
tmux send-keys -t "$SESSION:0.2" 'source install/setup.bash' C-m
tmux send-keys -t "$SESSION:0.2" \
    "python3 $PROJECT_ROOT_Q/scripts/initial_pose_tf.py" C-m

tmux select-pane -t "$SESSION:0.0"

gnome-terminal --window \
    --title="MPPI Navigation HeightJump" \
    -- bash -lc "tmux attach-session -t $SESSION"
