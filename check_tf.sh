#!/bin/bash

echo "========================================="
echo "Navigation2 TF 树诊断工具"
echo "========================================="
echo ""

# 检查关键节点是否运行
echo "1. 检查关键 ROS2 节点:"
echo "-----------------------------------------"
ros2 node list | grep -E "(amcl|controller_server|planner_server)" || echo "❌ 未找到关键节点"
echo ""

# 检查 TF 变换
echo "2. 检查关键 TF 变换:"
echo "-----------------------------------------"
echo "检查 map -> camera_init 变换 (AMCL发布):"
ros2 run tf2_ros tf2_echo map camera_init &
TF_PID=$!
sleep 2
kill $TF_PID 2>/dev/null
wait $TF_PID 2>/dev/null
echo ""

echo "检查 camera_init -> base_link 变换 (SLAM发布):"
ros2 run tf2_ros tf2_echo camera_init base_link &
TF_PID=$!
sleep 2
kill $TF_PID 2>/dev/null
wait $TF_PID 2>/dev/null
echo ""

# 检查 TF 发布频率
echo "3. 检查 TF 发布频率:"
echo "-----------------------------------------"
echo "AMCL TF 发布频率 (map -> camera_init):"
ros2 topic hz /tf_static --window 5 2>&1 | grep "average rate" || echo "无法测量"
echo ""

# 生成 TF 树图
echo "4. 生成 TF 树图:"
echo "-----------------------------------------"
ros2 run tf2_tools view_frames.py
if [ -f "frames.pdf" ]; then
    echo "✅ TF 树图已保存到 frames.pdf"
    echo "   使用以下命令查看: evince frames.pdf"
else
    echo "❌ 未能生成 TF 树图"
fi
echo ""

# 检查 AMCL 状态
echo "5. 检查 AMCL 粒子数:"
echo "-----------------------------------------"
ros2 topic info /particle_cloud || echo "❌ /particle_cloud 话题不存在"
echo ""

# 检查代价地图
echo "6. 检查代价地图话题:"
echo "-----------------------------------------"
echo "Local Costmap 频率:"
ros2 topic hz /local_costmap/costmap_raw --window 3 2>&1 | grep "average rate" || echo "无数据"
echo "Global Costmap 频率:"
ros2 topic hz /global_costmap/costmap_raw --window 3 2>&1 | grep "average rate" || echo "无数据"
echo ""

echo "========================================="
echo "诊断完成!"
echo "========================================="
echo ""
echo "📋 配置说明:"
echo "  - local_costmap.global_frame: camera_init (odom坐标系)"
echo "  - global_costmap.global_frame: map (全局坐标系)"
echo "  - amcl.save_pose_rate: 10.0 Hz (TF发布频率)"
echo "========================================="