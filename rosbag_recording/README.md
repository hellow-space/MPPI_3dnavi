# 四轮足巡检 ROS2 Rosbag 录制目录

## 目录

```text
rosbag_recording/
├── scripts/
│   ├── check_topics.sh
│   ├── record_nav_baseline.sh
│   ├── record_terrain_raw.sh
│   ├── record_terrain_elevation.sh
│   └── inspect_bag.sh
└── bags/
    ├── nav_baseline/
    ├── terrain_raw/
    └── terrain_elevation/
```

## 推荐安装位置

```bash
~/MPPI_3dnavi/rosbag_recording
```

## 1. 自检

```bash
cd ~/MPPI_3dnavi/rosbag_recording
chmod +x scripts/*.sh
./scripts/check_topics.sh
```

## 2. 录 2D Navigation Baseline

```bash
./scripts/record_nav_baseline.sh
```

录制：

```text
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
```

## 3. 录原始地形数据

```bash
./scripts/record_terrain_raw.sh
```

录制：

```text
/cloud_registered
/aft_mapped_to_init
/tf
/tf_static
```

用于后续：

```text
rosbag replay
  ↓
/cloud_registered
  ↓
Elevation Mapping
  ↓
离线调试
```

## 4. Elevation Map 打通后录制

```bash
./scripts/record_terrain_elevation.sh
```

默认高程图 topic：

```text
/elevation_mapping_node/elevation_map
```

若实际名称不同：

```bash
ELEVATION_TOPIC=/your/topic ./scripts/record_terrain_elevation.sh
```

## 5. 检查 bag

```bash
./scripts/inspect_bag.sh bags/nav_baseline/<bag目录>
```

重点确认：

- Duration > 0
- Messages > 0
- 所需 topic 都存在
- 高频 topic 消息数量合理

## 当前已确认

```text
/cloud_registered       PointCloud2   ≈10 Hz
/aft_mapped_to_init     Odometry      ≈10 Hz
/path                   Path          laserMapping 发布
/cmd_vel                Twist         ≈20 Hz
/plan                   Path          Nav2 Global Plan
/transformed_global_plan Path
/trajectories           MarkerArray
```

`/odom` 和 `/local_plan` 当前没有 Publisher，因此不录。
