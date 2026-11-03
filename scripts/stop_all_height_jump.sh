#!/usr/bin/env bash

# 配套 start_all_height_jump.sh：导航 -> 高程图容器 -> 雷达与定位。
# 使用：./scripts/stop_all_height_jump.sh [--dry-run]
# 高程图终端保留日志；elevation_env 容器会停止，下一次启动时自动开启。
set -uo pipefail

PROJECT_ROOT=/home/nvidia/MPPI_3dnavi
LIVO_DIR='/home/nvidia/wsnew/start&close_script'
DRY_RUN=false
FAILED=0

case "${1:-}" in
    '') ;;
    --dry-run) DRY_RUN=true ;;
    *) echo "用法：$0 [--dry-run]" >&2; exit 1 ;;
esac
if (( $# > 1 )); then
    echo "用法：$0 [--dry-run]" >&2
    exit 1
fi

run_stop() {
    if "$DRY_RUN"; then
        printf '[预览] '
        printf '%q ' "$@"
        printf '\n'
    elif ! "$@"; then
        echo '本步骤关闭失败，继续关闭其余组件。' >&2
        FAILED=1
    fi
}

echo '[1/3] 关闭 HeightJump 导航、keep_zero 和 initial_pose_tf'
run_stop bash "$PROJECT_ROOT/scripts/stop_navigation_height_jump.sh"

echo '[2/3] 关闭高程图及 elevation_env 容器（最多等待 10 秒后强制结束）'
run_stop docker stop --timeout 10 elevation_env

echo '[3/3] 关闭雷达、FAST-LIVO2 和 PCL'
run_stop bash "$LIVO_DIR/stop_livo.sh"

# 现有雷达停止脚本最后一行是 echo，额外确认目标会话确实消失。
if ! "$DRY_RUN"; then
    if ! command -v tmux >/dev/null; then
        echo '缺少 tmux，无法确认导航和雷达会话是否关闭。' >&2
        FAILED=1
    else
        for session in mppi_nav_height_jump livo_system; do
            if tmux has-session -t "=$session" 2>/dev/null; then
                echo "关闭失败：$session 仍在运行。" >&2
                FAILED=1
            fi
        done
    fi
fi

if "$DRY_RUN"; then
    echo '预览结束：未关闭任何进程或容器。'
elif (( FAILED )); then
    echo '部分组件关闭失败，请检查上方错误。' >&2
else
    echo '导航、高程图容器、雷达与定位已关闭。高程图终端保留日志，可手动关闭。'
fi
exit "$FAILED"
