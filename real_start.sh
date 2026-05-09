#!/bin/bash

# ================================
# ROS2 多终端一键启动脚本
# gnome-terminal 替换为 xterm，其余完全不变
# ================================

#终端3：启动导航
xterm -title "navigation" -e "
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
exec bash
" &

