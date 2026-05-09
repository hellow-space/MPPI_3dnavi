#!/bin/bash

###############################################################################
# CPU 峰值诊断工具
# 用途: 找出导致 CPU 突然满载的具体进程和原因
###############################################################################

echo "========================================="
echo "  CPU 峰值诊断工具"
echo "========================================="
echo ""

# 1. 查看当前 CPU 使用率最高的进程
echo "📊 当前 CPU 使用率 TOP 10 进程:"
echo "-----------------------------------------"
ps aux --sort=-%cpu | head -11
echo ""

# 2. 检查 ROS2 相关进程
echo "🤖 ROS2 相关进程 CPU 使用:"
echo "-----------------------------------------"
ps aux | grep -E "(amcl|controller|planner|costmap|mppi)" | grep -v grep | awk '{printf "%-50s CPU: %s%%\n", $11, $3}'
echo ""

# 3. 检查是否有僵尸进程或异常进程
echo "⚠️  异常进程检查:"
echo "-----------------------------------------"
ZOMBIE_COUNT=$(ps aux | awk '$8 == "Z" {count++} END {print count+0}')
if [ "$ZOMBIE_COUNT" -gt 0 ]; then
    echo "发现 $ZOMBIE_COUNT 个僵尸进程!"
    ps aux | awk '$8 == "Z" {print $0}'
else
    echo "✓ 没有僵尸进程"
fi
echo ""

# 4. 检查系统负载
echo "📈 系统负载 (1/5/15分钟):"
echo "-----------------------------------------"
uptime
echo ""

# 5. 检查 CPU 频率和温度
echo "🌡️  CPU 状态:"
echo "-----------------------------------------"
if command -v nvidia-smi &> /dev/null; then
    echo "GPU 信息:"
    nvidia-smi --query-gpu=name,utilization.gpu,temperature.gpu,power.draw --format=csv
    echo ""
fi

# 6. 检查内存压力
echo "💾 内存压力:"
echo "-----------------------------------------"
free -h
echo ""

# 7. 检查 IO 等待
echo "💿 IO 等待状态:"
echo "-----------------------------------------"
iostat -x 1 1 2>/dev/null || echo "需要安装 sysstat: sudo apt install sysstat"
echo ""

# 8. 实时监控建议
echo "========================================="
echo "🔧 实时监控命令:"
echo "========================================="
echo ""
echo "# 方法1: 使用 top 实时查看 (按 P 按 CPU 排序)"
echo "top -p \$(pgrep -f 'amcl|controller|planner' | tr '\n' ',' | sed 's/,$//')"
echo ""
echo "# 方法2: 使用 htop (更友好)"
echo "htop"
echo ""
echo "# 方法3: 记录 CPU 峰值日志"
echo "while true; do echo \"\$(date): \$(ps aux --sort=-%cpu | head -3)\" >> cpu_log.txt; sleep 1; done"
echo ""
echo "# 方法4: 监控特定 ROS2 节点"
echo "ros2 topic hz /amcl/particle_cloud --window 5"
echo ""

echo "========================================="
echo "💡 优化建议:"
echo "========================================="
echo ""
echo "1. AMCL 优化:"
echo "   - 减少 max_particles (当前: 1000, 可尝试 500)"
echo "   - 减少 max_beams (当前: 36, 可尝试 24)"
echo "   - 降低 save_pose_rate (当前: 1.0, 可尝试 0.5)"
echo ""
echo "2. MPPI 控制器优化:"
echo "   - 减少 batch_size (当前: 1000, 可尝试 500)"
echo "   - 减少 time_steps (当前: 56, 可尝试 40)"
echo "   - 降低 controller_frequency (当前: 20.0, 可尝试 10.0)"
echo ""
echo "3. 点云处理优化:"
echo "   - 检查 pointcloud.py 是否有不必要的处理"
echo "   - 增加点云降采样率"
echo "   - 减少点云发布频率"
echo ""
echo "4. 系统级优化:"
echo "   - 关闭不必要的后台服务"
echo "   - 使用 nice/renice 调整进程优先级"
echo "   - 考虑使用 CPU 隔离 (isolcpus)"
echo ""