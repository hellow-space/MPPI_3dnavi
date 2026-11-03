#!/usr/bin/env bash

# startup.md 的一键入口：雷达与定位 -> Docker 高程图 -> HeightJump 导航。
# 使用：./scripts/start_all_height_jump.sh [--dry-run]
# 调整等待时间：LIVO_WAIT_SECONDS=10 ELEVATION_WAIT_SECONDS=10 ./scripts/start_all_height_jump.sh
# 等待时间仅用于错开启动，不代表节点已经就绪。
set -eo pipefail

PROJECT_ROOT=/home/nvidia/MPPI_3dnavi
LIVO_DIR='/home/nvidia/wsnew/start&close_script'
LIVO_WAIT_SECONDS=${LIVO_WAIT_SECONDS:-5}
ELEVATION_WAIT_SECONDS=${ELEVATION_WAIT_SECONDS:-5}
DRY_RUN=false

case "${1:-}" in
    '') ;;
    --dry-run) DRY_RUN=true ;;
    *) echo "用法：$0 [--dry-run]" >&2; exit 1 ;;
esac
if (( $# > 1 )); then
    echo "用法：$0 [--dry-run]" >&2
    exit 1
fi
if [[ ! $LIVO_WAIT_SECONDS =~ ^[0-9]+$ || ! $ELEVATION_WAIT_SECONDS =~ ^[0-9]+$ ]]; then
    echo '等待时间必须是非负整数（秒）。' >&2
    exit 1
fi

for dependency in gnome-terminal tmux docker; do
    command -v "$dependency" >/dev/null || { echo "缺少命令：$dependency" >&2; exit 1; }
done
for file in /opt/ros/humble/setup.bash "$PROJECT_ROOT/install/setup.bash" \
    "$LIVO_DIR/start_livo.sh" "$PROJECT_ROOT/scripts/start_navigation_height_jump.sh"; do
    [[ -r $file ]] || { echo "找不到或无法读取：$file" >&2; exit 1; }
done

run() {
    if "$DRY_RUN"; then
        printf '[预览] '
        printf '%q ' "$@"
        printf '\n'
    else
        "$@"
    fi
}

cd "$PROJECT_ROOT"
# ROS 的 setup 脚本可能引用未定义变量，因此本脚本不启用 set -u。
run source /opt/ros/humble/setup.bash
run source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

# tmux 已运行时，新会话使用服务端环境，需要同步这两个 ROS 设置。
# 没有服务端时，start_livo.sh 创建的服务端会直接继承本脚本环境。
if ! "$DRY_RUN" && tmux list-sessions >/dev/null 2>&1; then
    tmux set-environment -g ROS_DOMAIN_ID "$ROS_DOMAIN_ID"
    tmux set-environment -g RMW_IMPLEMENTATION "$RMW_IMPLEMENTATION"
fi

echo '[1/3] 启动雷达与定位（独立 LIVO SYSTEM 窗口）'
run bash -c 'cd "/home/nvidia/wsnew/start&close_script" && ./start_livo.sh'
run sleep "$LIVO_WAIT_SECONDS"

echo '[2/3] 启动高程图（独立 Elevation Map 窗口）'
run bash -c 'docker start elevation_env >/dev/null'
ELEVATION_COMMAND=$(cat <<'COMMAND'
docker exec -it elevation_env bash -lc '
cd /root/elevation_map &&
source /opt/ros/humble/setup.bash &&
source install/setup.bash &&
export ROS_DOMAIN_ID=42 &&
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp &&
exec ros2 launch elevation_mapping_cupy raw_elevation.launch.py
'
status=$?
printf '\n高程图进程已退出，退出码：%s。窗口保留供查看日志。\n' "$status"
exec bash
COMMAND
)
run gnome-terminal --window --title='Elevation Map' -- bash -c "$ELEVATION_COMMAND"
run sleep "$ELEVATION_WAIT_SECONDS"

echo '[3/3] 启动 HeightJump 导航（独立导航窗口）'
run bash -c 'cd /home/nvidia/MPPI_3dnavi && ./scripts/start_navigation_height_jump.sh'

if "$DRY_RUN"; then
    echo '预览结束：未启动雷达、容器或导航。'
else
    echo '三阶段启动命令已执行，请在各自终端查看节点日志。'
fi
