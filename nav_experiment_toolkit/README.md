# Navigation Experiment Toolkit

这套工具把一次导航实验完整闭环成：

```text
检查 Topic
   ↓
录制自包含 rosbag（含 /map）
   ↓
标准 RViz 重播
   ↓
自动解析 bag
   ↓
轨迹 / 控制 / 跟踪误差 / 指标
```

## 目录

```text
nav_experiment_toolkit/
├── analysis/
│   └── analyze_nav_bag.py
├── config/
│   ├── nav_replay.rviz
│   └── nav_elevation_replay.rviz
├── scripts/
│   ├── check_before_record.sh
│   ├── record_nav_baseline.sh
│   ├── record_nav_elevation.sh
│   ├── inspect_bag.sh
│   ├── replay_nav_rviz.sh
│   ├── replay_nav_elevation_rviz.sh
│   └── analyze_nav_bag.sh
└── bags/
    ├── nav_baseline/
    └── nav_elevation/
```

## 0. 放到 Jetson

推荐：

```bash
cd ~/MPPI_3dnavi
unzip nav_experiment_toolkit.zip
cd nav_experiment_toolkit
chmod +x scripts/*.sh analysis/*.py
```

如果缺 matplotlib / numpy：

```bash
sudo apt install python3-numpy python3-matplotlib
```

ROS2 Humble 自带分析所需的 `rosbag2_py`、`rclpy` 等 ROS Python 包时，
请确保运行前已经：

```bash
source /opt/ros/humble/setup.bash
source ~/你的工作空间/install/setup.bash
```

## 1. 录制前检查

```bash
./scripts/check_before_record.sh
```

## 2. 一次实验只录一个导航任务

```bash
./scripts/record_nav_baseline.sh
```

包含：

```text
/map
/tf
/tf_static
/aft_mapped_to_init
/path
/amcl_pose
/command/cmd_twist
/cmd_vel
/goal_pose
/plan
/transformed_global_plan
/trajectories
/local_costmap/costmap
/global_costmap/costmap
```

如果存在：

```text
/navigate_to_pose/_action/status
```

脚本也会自动加入。

## 3. 检查 bag

```bash
./scripts/inspect_bag.sh bags/nav_baseline/nav_baseline_XXXX
```

重点确认 `/map`、`/tf`、`/plan`、`/transformed_global_plan`、`/cmd_vel`
的 Message Count > 0。

## 4. RViz 标准重播

```bash
./scripts/replay_nav_rviz.sh bags/nav_baseline/nav_baseline_XXXX
```

自动：

```text
RViz
Fixed Frame = map
use_sim_time = true

显示：
/map
/plan
/path
/transformed_global_plan
/trajectories
/amcl_pose
TF
/local_costmap/costmap
/global_costmap/costmap

然后：
ros2 bag play BAG --clock
```

## 高程导航独立录制与回放

高程模板记录核心实验链路：点云 → 高程/方差 → 高度差 → 代价地图 → 路径/控制。
适用于已启动 HeightObstacleLayer 的高程导航实验。

```bash
./scripts/check_before_record.sh elevation
./scripts/record_nav_elevation.sh
./scripts/inspect_bag.sh bags/nav_elevation/nav_elevation_XXXX
./scripts/replay_nav_elevation_rviz.sh bags/nav_elevation/nav_elevation_XXXX
```

核心话题清单统一放在 `config/elevation_recording_topics.sh`：

- 建图：`/cloud_registered`、`/elevation_mapping_node/elevation_map`（完整 GridMap，含已发布的 elevation/variance）。
- 判定：`/height_obstacle_layer/height_difference_map`、`/local_costmap/costmap`、`/global_costmap/costmap`。
- 定位：`/tf`、`/tf_static`、`/aft_mapped_to_init`、`/map`。
- 导航：`/goal_pose`、`/plan`、`/transformed_global_plan`、`/command/cmd_twist`、`/cmd_vel`。
- 任务：NavigateToPose 和 FollowPath 的 `/_action/status`；手绘路径的 `/custom_path/raw_path`、`/custom_path/preview_path`、`/custom_path/status`。

任务话题即使录制启动时不存在，也会等待发现；只使用其中一种任务入口时，另一组没有消息正常。
已启用隐藏话题录制；action status 提供状态，不包含完整 result 服务响应或全部错误细节。
默认不录 `/trajectories`、`/path`、`/amcl_pose`，减少体积和重复信息。
当前 costmap 配置为全量发布，因此不增加 updates 话题。

录制脚本会检查建图、判定、定位的核心话题，缺失时退出。
检查只确认话题存在，不代表有有效消息；录制后用 `inspect_bag.sh` 确认消息数。
先录制并等待订阅，再发目标；任务前后各保留5–10秒，最后 Ctrl-C 停止。
若更改高程话题名，在检查和录制命令前都设置 `ELEVATION_TOPIC=/实际话题`。
仅启动高程建图、没有导航时，使用 `../rosbag_recording/scripts/record_terrain_elevation.sh`。

高程 RViz 使用倾斜三维视角显示：

```text
/elevation_mapping_node/elevation_map（elevation 层）
Robot Footprint Box
/plan
/path
/transformed_global_plan
/trajectories
TF
```

## 5. 自动定量分析

```bash
./scripts/analyze_nav_bag.sh bags/nav_baseline/nav_baseline_XXXX
```

默认分析：

```text
map frame  = map
base frame = base_link
goal xy tolerance  = 0.30 m
goal yaw tolerance = 0.35 rad
```

如果机器人 base frame 不同：

```bash
./scripts/analyze_nav_bag.sh BAG --base-frame base_footprint
```

输出：

```text
BAG/analysis/
├── report.md
├── metrics.csv
├── trajectory.csv
├── active_global_error.csv
├── controller_reference_error.csv
├── trajectory_map.png
├── active_global_error.png
├── controller_reference_error.png
├── cmd_linear.png
└── cmd_angular.png
```

## 全局路径会刷新，误差怎么定义？

不能只定义一个误差。

### A. Controller reference error —— 主指标

```text
实际 pose(t)
    ↓
取 t 时刻之前最新的一条 /transformed_global_plan
    ↓
计算实际位置到该 Path 最近点的距离
```

这是判断 **MPPI 本身跟踪能力** 最合适的指标。

### B. Active global-plan error

```text
实际 pose(t)
    ↓
取 t 时刻之前最新的一条 /plan
    ↓
最近点距离
```

这允许 `/plan` 正常刷新，不会把新旧 Path 混在一起。

### C. Initial-plan deviation

```text
第一次 /plan
    ↓
冻结
    ↓
整段实际轨迹都与它比较
```

用途不是评价 controller，而是回答：

> 后面误差变小，是机器人真的跟得更好了，还是全局规划不断重新贴着机器人规划？

如果：

```text
Active global error 很小
Initial plan error 很大
```

说明 replanning 改变了参考路线，不能把所有改善都归因于 MPPI。

## 最终建议看这些指标

```text
1. controller_reference_error_rmse_m   ← MPPI 跟踪主指标
2. controller_reference_error_p95_m
3. active_global_error_rmse_m
4. initial_plan_error_rmse_m
5. actual_path_length_m
6. navigation_time_s
7. plan_publish_rate_hz
8. final_cmd_rms_w                     ← 转向是否剧烈
9. final_cmd_w_total_variation         ← 左右摇摆/控制抖动
```

## 一个重要限制：cmd_vel 有多个 Publisher

你当前：

```text
/command/cmd_twist
  controller_server
  behavior_server

/cmd_vel
  velocity_smoother
  keep_alive_zero
```

rosbag2 对同一个 topic 录下来后不会给每条消息保留一个方便后处理的
“这是哪个 publisher 发的”的字段。

因此：

- 轨迹与 Path 误差：可以严谨定量。
- `cmd_vel` 曲线：可以直接看。
- publisher-specific 的控制统计：如果未来要非常严格，需要在系统里把
  velocity_smoother 和 keep_alive 的输出拆成不同 topic。

第一阶段不需要为了这个改系统。
