# 配置恢复说明

## ✅ 已恢复的配置

所有修改已恢复到 `before.yaml` 的原始状态:

### 1. AMCL 配置
```yaml
amcl:
  ros__parameters:
    transform_tolerance: 5.0
    max_beams: 60              # 恢复为 60
    max_particles: 2000        # 恢复为 2000
    min_particles: 500         # 恢复为 500
    save_pose_rate: 0.5        # 恢复为 0.5 Hz
    update_min_a: 0.2          # 恢复为 0.2
    update_min_d: 0.25         # 恢复为 0.25
```

### 2. Controller Server 配置
```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # 恢复为 20.0 Hz
```

### 3. MPPI 控制器配置
```yaml
FollowPath:
  time_steps: 56               # 恢复为 56
  batch_size: 1000             # 恢复为 1000
  wz_min: -2.0                 # 恢复为 -2.0
  wz_max: 2.0                  # 恢复为 2.0
  visualize: true              # 恢复为 true
```

### 4. Local Costmap 配置
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      global_frame: camera_init
      rolling_window: true     # 恢复为 true (小写)
      width: 5                 # 恢复为 5
      height: 5                # 恢复为 5
      transform_tolerance: 5.0
      plugins: ["inflation_layer", "voxel_layer"]
```

## 📊 恢复后的性能特征

| 指标 | 优化前(已恢复) | 说明 |
|------|--------------|------|
| CPU 峰值 | 🔴 90-100% | MPPI 计算量大 |
| AMCL TF 频率 | 0.5 Hz | 较低 |
| 控制频率 | 20 Hz | 较高 |
| 预测时域 | 2.8s | 较长 |
| 采样数 | 1000 | 较多 |

## ⚠️ 注意事项

恢复后,你可能会再次遇到以下问题:
1. **CPU 突然满载** - MPPI 控制器计算量过大
2. **TF 变换卡顿** - AMCL 频率太低
3. **Local Costmap 显示问题** - 需要正确配置 transform_tolerance

## 💡 如果需要再次优化

参考以下文档:
- `CPU_PEAK_SOLUTION.md` - CPU 峰值解决方案
- `PERFORMANCE_OPTIMIZATION.md` - 性能优化指南
- `RVIZ_COORDINATE_TRANSFORM.md` - RViz 坐标变换说明

## 🚀 测试建议

重启导航系统验证配置:
```bash
./real_start.sh
```

监控性能:
```bash
./monitor_performance.sh
```

---
**恢复时间**: 2026-05-09
**备份文件**: `src/fishbot_navigation2/config/before.yaml`