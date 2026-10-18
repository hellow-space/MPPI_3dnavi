

## 脚本录包
# 录制
cd ~/MPPI_3dnavi/nav_experiment_toolkit
./scripts/record_nav_baseline.sh
# 检查
 ./scripts/inspect_bag.sh bags/nav_baseline/包名
# 回放
./scripts/replay_nav_rviz.sh bags/nav_baseline/nav_baseline_2026-09-19_17-31-49
 
 
## 高程图导航录包
# 录制
cd ~/MPPI_3dnavi/nav_experiment_toolkit
./scripts/record_nav_elevation.sh
# 检查
./scripts/inspect_bag.sh bags/nav_elevation/包名
# 回放
./scripts/replay_nav_elevation_rviz.sh bags/nav_elevation/包名


## 基础使用
# 录
ros2 bag record -o test_bag /map /tf /tf_static /plan /cmd_vel

# 查
ros2 bag info test_bag

# 放
ros2 bag play test_bag --clock

#暂停

终端空格

