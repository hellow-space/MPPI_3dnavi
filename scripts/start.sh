#!/bin/bash

# 终端1：启动导航
gnome-terminal --title="navigation" -- bash -c '
cd /home/nvidia/MPPI_3dnavi
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
exec bash
' &

sleep 2

# 终端2：启动 keep_zero
gnome-terminal --title="keep_zero" -- bash -c '
cd /home/nvidia/MPPI_3dnavi/src
python3 keep_zero.py
exec bash
' &