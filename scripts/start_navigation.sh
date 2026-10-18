#!/bin/bash

SESSION="mppi_nav"

# ------------------------------------------------------------
# 1. 如果旧 session 存在，先关闭
# ------------------------------------------------------------
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "发现旧的 $SESSION，正在关闭..."
    tmux kill-session -t "$SESSION"
    sleep 1
fi

# ------------------------------------------------------------
# 2. 创建 tmux session
# pane 0 先只是一个普通 bash
# ------------------------------------------------------------
tmux new-session -d -s "$SESSION" -n main

# 开启鼠标
tmux set-option -t "$SESSION" mouse on

# 显示 pane 名字
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format \
' #{pane_index} | #{pane_title} '

# ------------------------------------------------------------
# 3. 左边：Navigation
# ------------------------------------------------------------
tmux select-pane -t "$SESSION:0.0" -T "Navigation"

tmux send-keys -t "$SESSION:0.0" \
'cd /home/nvidia/MPPI_3dnavi' C-m

tmux send-keys -t "$SESSION:0.0" \
'source install/setup.bash' C-m

tmux send-keys -t "$SESSION:0.0" \
'ros2 launch fishbot_navigation2 navigation2.launch.py params_file:=/home/nvidia/MPPI_3dnavi/src/fishbot_navigation2/config/2d_params.yaml' C-m

# ------------------------------------------------------------
# 4. 创建右边 pane
# ------------------------------------------------------------
tmux split-window -h -t "$SESSION:0.0"

tmux select-pane -t "$SESSION:0.1" -T "keep_zero"

tmux send-keys -t "$SESSION:0.1" \
'cd /home/nvidia/MPPI_3dnavi/src' C-m

tmux send-keys -t "$SESSION:0.1" \
'python3 keep_zero.py' C-m

# ------------------------------------------------------------
# 5. 左右均分
# ------------------------------------------------------------
tmux select-layout -t "$SESSION:0" even-horizontal

# 默认选中左边
tmux select-pane -t "$SESSION:0.0"

# ------------------------------------------------------------
# 6. 强制打开一个新的 GNOME Terminal 窗口
# ------------------------------------------------------------
gnome-terminal --window \
    --title="MPPI Navigation" \
    -- bash -lc "tmux attach-session -t $SESSION"

exit 0