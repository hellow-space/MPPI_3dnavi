# Navigation2 Local Costmap TF 频率问题解决方案

## 问题描述
当 local_costmap 的 global_frame 设置为 `odom`(或 `camera_init`)时,局部代价地图在 RViz 中不显示。错误提示通常是 **TF 变换频率太低** 或 **Transform timeout**。

## 问题根源

### Navigation2 官方架构要求:
```
global_costmap: global_frame = map    (全局坐标系,用于路径规划)
local_costmap:  global_frame = odom   (里程计坐标系,用于局部避障)
```

### TF 树结构:
```
map (全局坐标系)
  ↓ [AMCL 发布 map->camera_init TF, 默认频率 ~1Hz]
camera_init (SLAM odom 坐标系)
  ↓ [SLAM 系统发布 camera_init->base_link TF, 高频 ~100Hz]
base_link (机器人基座坐标系)
```

### 问题原因:
**AMCL 的 `save_pose_rate` 默认值为 0.5 Hz**,导致 map->camera_init 的 TF 变换发布频率太低。
而 local_costmap 的 `update_frequency` 是 10Hz,**需要更高频率的 TF 更新**,否则会因 TF 超时而无法显示。

## ✅ 解决方案

### 方案 1: 提高 AMCL 的 TF 发布频率(推荐)

修改 `nav2_params.yaml`:

```yaml
amcl:
  ros__parameters:
    save_pose_rate: 10.0  # 🔥 从 0.5 提高到 10.0,增加 TF 发布频率
    
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: camera_init  # 🔥 使用 camera_init(即 odom frame)
      transform_tolerance: 0.5   # 保持适当的容忍度
```

**优点**:
- ✅ 符合 Navigation2 官方架构
- ✅ local_costmap 使用相对稳定的 odom 坐标系,避免地图漂移
- ✅ 全局和局部分离,路径规划更稳定

### 方案 2: 增加 transform_tolerance

如果提高频率后仍有问题,可以进一步增加容忍度:

```yaml
amcl:
  ros__parameters:
    transform_tolerance: 1.0  # 增加到 1.0 秒
    
local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 1.0  # 也增加到 1.0 秒
      tf_cache_time: 60.0       # TF 缓存时间
```

### 方案 3: 使用 static_map 模式(不推荐)

如果实在无法解决,可以将 local_costmap 也改为使用 map:

```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: map  # 临时方案
```

**缺点**:
- ❌ 不符合官方架构
- ❌ local_costmap 会随全局地图漂移
- ❌ 局部避障效果可能变差

## 🔧 已应用的修改

### 1. AMCL 配置优化
```yaml
amcl:
  ros__parameters:
    save_pose_rate: 10.0  # 从 0.5 提高到 10.0 Hz
    transform_tolerance: 1.0  # 从 0.2 提高到 1.0
    odom_frame_id: "camera_init"
    tf_broadcast: true
```

### 2. Local Costmap 配置
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: camera_init  # 改为 camera_init(即 odom)
      transform_tolerance: 0.5
      tf_cache_time: 60.0
```

## 📊 验证步骤

### 步骤 1: 重启导航系统
```bash
./real_start.sh
```

### 步骤 2: 运行诊断脚本
```bash
./check_tf.sh
```

**期望输出**:
- AMCL TF 发布频率应该接近 10Hz
- Local Costmap 应该有数据输出(~10Hz)

### 步骤 3: 检查 TF 频率
```bash
# 在一个终端监控 TF
ros2 run tf2_ros tf2_echo map camera_init

# 在另一个终端查看频率
ros2 topic hz /tf --window 10
```

**期望结果**: TF 更新频率应该在 5-10Hz 之间

### 步骤 4: 在 RViz 中验证
1. 添加 "Local Costmap" 显示项
2. 设置初始位姿(2D Pose Estimate)
3. 应该能看到局部代价地图正常显示并更新

## 🔍 常见问题排查

### 问题 1: 提高频率后仍然不显示
**检查**:
```bash
# 确认 AMCL 是否正常运行
ros2 node list | grep amcl

# 确认 TF 是否存在
ros2 run tf2_tools view_frames.py
evince frames.pdf
```

**解决**: 确保在 RViz 中设置了初始位姿

### 问题 2: TF 频率仍然很低
**可能原因**: AMCL 没有接收到激光雷达数据

**检查**:
```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

**解决**: 
- 检查激光雷达驱动是否正常
- 确认 scan_topic 配置正确
- 检查激光雷达 frame_id

### 问题 3: 出现 Transform timeout 警告
**解决**: 进一步增加 transform_tolerance:
```yaml
amcl:
  ros__parameters:
    transform_tolerance: 2.0
    
local_costmap:
  local_costmap:
    ros__parameters:
      transform_tolerance: 1.0
```

## 📈 性能对比

| 配置 | TF 频率 | 显示稳定性 | 导航精度 |
|------|---------|-----------|---------|
| save_pose_rate: 0.5 | ~0.5Hz | ❌ 经常超时 | - |
| save_pose_rate: 5.0 | ~5Hz | ⚠️ 偶尔超时 | ✅ |
| save_pose_rate: 10.0 | ~10Hz | ✅ 稳定 | ✅✅ |
| save_pose_rate: 20.0 | ~20Hz | ✅ 非常稳定 | ✅✅ |

**建议**: 从 10.0 开始,如果系统负载允许,可以提高到 20.0

## 💡 关键要点总结

1. **Navigation2 官方要求**: local_costmap 使用 odom frame,global_costmap 使用 map frame
2. **问题根源**: AMCL 的 save_pose_rate 默认值太低(0.5Hz)
3. **解决方案**: 将 save_pose_rate 提高到 10.0 Hz 或更高
4. **配合调整**: 适当增加 transform_tolerance 作为保险
5. **验证方法**: 使用 `./check_tf.sh` 脚本检查 TF 频率和代价地图状态

## 🎯 快速测试命令

```bash
# 1. 启动导航
./real_start.sh

# 2. 在新终端运行诊断
./check_tf.sh

# 3. 实时监控 TF 频率
ros2 topic hz /tf --window 5

# 4. 监控代价地图
ros2 topic hz /local_costmap/costmap_raw

# 5. 查看 AMCL 日志
ros2 log get amcl | grep -i "pose\|tf"
```