#!/bin/bash

# ================================
# ROS2 多终端一键启动脚本
# gnome-terminal 替换为 xterm，其余完全不变
# ================================

# 终端1：启动 ros2 launch
xterm -title "fake_odom" -e "
source install/setup.bash
cd src
python3 fake_odom.py
exec bash
" &

# #终端2：启动fake_laser
# xterm -title "fake_laser" -e "
# source install/setup.bash
# cd src
# python3 fake_laser.py
# exec bash
# " &

#终端2
xterm -title "pcl-scan" -e "
source install/setup.bash
ros2 launch fishbot_navigation2 pcl_to_scan.launch.py
exec bash
" &

#终端3：启动导航
xterm -title "navigation" -e "
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
exec bash
" &

#终端4：给定静态变换
xterm -title "tf_static" -e "
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher   --x 0.0   --y 0.0   --z 0.2   --yaw 0.0   --pitch 0.0   --roll 0.0   --frame-id base_link   --child-frame-id livox_frame
exec bash
" &
