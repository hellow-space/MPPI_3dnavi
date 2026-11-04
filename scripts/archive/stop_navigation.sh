#!/bin/bash

SESSION="mppi_nav"

if ! tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "$SESSION 当前没有运行"
    exit 0
fi

echo "正在关闭 Navigation 和 keep_zero..."

# 给每个 pane 单独发送 Ctrl+C
tmux list-panes -t "$SESSION" -F '#{pane_id}' |
while read pane
do
    tmux send-keys -t "$pane" C-c
done

sleep 2

# 删除整个 tmux session
tmux kill-session -t "$SESSION"

echo "已关闭"