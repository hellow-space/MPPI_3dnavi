#!/bin/bash

###############################################################################
# Navigation2 系统性能监控脚本
# 用途: 实时监控 CPU、内存、GPU 和 ROS2 节点状态
###############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 清屏
clear

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Navigation2 系统性能监控${NC}"
echo -e "${BLUE}  按 Ctrl+C 退出${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 主循环
while true; do
    # 移动到屏幕顶部
    tput cup 0 0
    
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  系统性能监控 - $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""
    
    # 1. CPU 使用率
    echo -e "${YELLOW}📊 CPU 使用率:${NC}"
    CPU_IDLE=$(top -bn1 | grep "Cpu(s)" | awk '{print $8}' | cut -d'%' -f1)
    if [ -z "$CPU_IDLE" ]; then
        CPU_IDLE=$(top -bn1 | grep "%Cpu" | awk '{print $8}' | cut -d'%' -f1)
    fi
    CPU_USAGE=$(echo "100 - $CPU_IDLE" | bc 2>/dev/null || echo "N/A")
    
    if [ "$CPU_USAGE" != "N/A" ]; then
        CPU_INT=${CPU_USAGE%.*}
        if [ "$CPU_INT" -gt 80 ]; then
            echo -e "  ${RED}● CPU: ${CPU_USAGE}% (高负载!)${NC}"
        elif [ "$CPU_INT" -gt 50 ]; then
            echo -e "  ${YELLOW}● CPU: ${CPU_USAGE}% (中等)${NC}"
        else
            echo -e "  ${GREEN}● CPU: ${CPU_USAGE}% (正常)${NC}"
        fi
    else
        echo -e "  ${YELLOW}● CPU: 无法获取${NC}"
    fi
    echo ""
    
    # 2. 内存使用
    echo -e "${YELLOW}💾 内存使用:${NC}"
    MEM_INFO=$(free -h | grep Mem)
    MEM_TOTAL=$(echo $MEM_INFO | awk '{print $2}')
    MEM_USED=$(echo $MEM_INFO | awk '{print $3}')
    MEM_AVAIL=$(echo $MEM_INFO | awk '{print $7}')
    MEM_PERCENT=$(free | grep Mem | awk '{printf "%.1f", $3/$2 * 100}')
    
    MEM_INT=${MEM_PERCENT%.*}
    if [ "$MEM_INT" -gt 80 ]; then
        echo -e "  ${RED}● 总计: ${MEM_TOTAL} | 已用: ${MEM_USED} | 可用: ${MEM_AVAIL} (${MEM_PERCENT}%)${NC}"
    elif [ "$MEM_INT" -gt 50 ]; then
        echo -e "  ${YELLOW}● 总计: ${MEM_TOTAL} | 已用: ${MEM_USED} | 可用: ${MEM_AVAIL} (${MEM_PERCENT}%)${NC}"
    else
        echo -e "  ${GREEN}● 总计: ${MEM_TOTAL} | 已用: ${MEM_USED} | 可用: ${MEM_AVAIL} (${MEM_PERCENT}%)${NC}"
    fi
    echo ""
    
    # 3. GPU 使用率 (如果有 NVIDIA GPU)
    if command -v nvidia-smi &> /dev/null; then
        echo -e "${YELLOW}🎮 GPU 使用率:${NC}"
        GPU_UTIL=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits | head -1)
        GPU_MEM=$(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader,nounits | head -1)
        
        if [ -n "$GPU_UTIL" ]; then
            GPU_INT=${GPU_UTIL%.*}
            if [ "$GPU_INT" -gt 80 ]; then
                echo -e "  ${RED}● GPU: ${GPU_UTIL}% | 显存: ${GPU_MEM} MB${NC}"
            elif [ "$GPU_INT" -gt 50 ]; then
                echo -e "  ${YELLOW}● GPU: ${GPU_UTIL}% | 显存: ${GPU_MEM} MB${NC}"
            else
                echo -e "  ${GREEN}● GPU: ${GPU_UTIL}% | 显存: ${GPU_MEM} MB${NC}"
            fi
        fi
        echo ""
    fi
    
    # 4. 磁盘使用
    echo -e "${YELLOW}💿 磁盘使用:${NC}"
    DISK_USAGE=$(df -h / | tail -1 | awk '{print $5}')
    DISK_AVAIL=$(df -h / | tail -1 | awk '{print $4}')
    echo -e "  ● 根分区: 已用 ${DISK_USAGE} | 可用 ${DISK_AVAIL}"
    echo ""
    
    # 5. ROS2 节点状态
    echo -e "${YELLOW}🤖 ROS2 节点:${NC}"
    if command -v ros2 &> /dev/null; then
        NODE_COUNT=$(ros2 node list 2>/dev/null | wc -l)
        echo -e "  ● 活跃节点数: ${NODE_COUNT}"
        
        # 检查关键节点
        AMCL_RUNNING=$(ros2 node list 2>/dev/null | grep -c "amcl" || echo "0")
        CONTROLLER_RUNNING=$(ros2 node list 2>/dev/null | grep -c "controller_server" || echo "0")
        PLANNER_RUNNING=$(ros2 node list 2>/dev/null | grep -c "planner_server" || echo "0")
        
        if [ "$AMCL_RUNNING" -gt 0 ]; then
            echo -e "  ${GREEN}✓ AMCL 运行中${NC}"
        else
            echo -e "  ${RED}✗ AMCL 未运行${NC}"
        fi
        
        if [ "$CONTROLLER_RUNNING" -gt 0 ]; then
            echo -e "  ${GREEN}✓ Controller Server 运行中${NC}"
        else
            echo -e "  ${YELLOW}○ Controller Server 未运行${NC}"
        fi
        
        if [ "$PLANNER_RUNNING" -gt 0 ]; then
            echo -e "  ${GREEN}✓ Planner Server 运行中${NC}"
        else
            echo -e "  ${YELLOW}○ Planner Server 未运行${NC}"
        fi
    else
        echo -e "  ${YELLOW}ROS2 未安装或未 sourced${NC}"
    fi
    echo ""
    
    # 6. 关键话题频率
    echo -e "${YELLOW}📡 关键话题状态:${NC}"
    if command -v ros2 &> /dev/null; then
        # 检查激光雷达
        SCAN_HZ=$(ros2 topic hz /scan --window 1 2>&1 | grep "average rate" | awk '{print $4}' || echo "N/A")
        if [ "$SCAN_HZ" != "N/A" ] && [ -n "$SCAN_HZ" ]; then
            echo -e "  ● /scan: ${SCAN_HZ} Hz"
        else
            echo -e "  ${YELLOW}○ /scan: 无数据${NC}"
        fi
        
        # 检查 TF
        TF_HZ=$(ros2 topic hz /tf --window 1 2>&1 | grep "average rate" | awk '{print $4}' || echo "N/A")
        if [ "$TF_HZ" != "N/A" ] && [ -n "$TF_HZ" ]; then
            echo -e "  ● /tf: ${TF_HZ} Hz"
        else
            echo -e "  ${YELLOW}○ /tf: 无数据${NC}"
        fi
        
        # 检查局部代价地图
        LOCAL_COSTMAP_HZ=$(ros2 topic hz /local_costmap/costmap_raw --window 1 2>&1 | grep "average rate" | awk '{print $4}' || echo "N/A")
        if [ "$LOCAL_COSTMAP_HZ" != "N/A" ] && [ -n "$LOCAL_COSTMAP_HZ" ]; then
            echo -e "  ● /local_costmap: ${LOCAL_COSTMAP_HZ} Hz"
        else
            echo -e "  ${YELLOW}○ /local_costmap: 无数据${NC}"
        fi
    fi
    echo ""
    
    # 7. 温度监控 (如果可用)
    if command -v nvidia-smi &> /dev/null; then
        echo -e "${YELLOW}🌡️  GPU 温度:${NC}"
        GPU_TEMP=$(nvidia-smi --query-gpu=temperature.gpu --format=csv,noheader,nounits | head -1)
        if [ -n "$GPU_TEMP" ]; then
            if [ "$GPU_TEMP" -gt 80 ]; then
                echo -e "  ${RED}● GPU 温度: ${GPU_TEMP}°C (过热!)${NC}"
            elif [ "$GPU_TEMP" -gt 60 ]; then
                echo -e "  ${YELLOW}● GPU 温度: ${GPU_TEMP}°C (较高)${NC}"
            else
                echo -e "  ${GREEN}● GPU 温度: ${GPU_TEMP}°C (正常)${NC}"
            fi
        fi
        echo ""
    fi
    
    echo -e "${BLUE}----------------------------------------${NC}"
    echo -e "${BLUE}提示: 按 Ctrl+C 退出监控${NC}"
    
    # 等待1秒后刷新
    sleep 1
done