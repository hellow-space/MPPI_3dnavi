# 手动初始定位与重力水平基准

`initial_pose_tf.py` 将 FAST-LIVO 的 `camera_init` 对齐到 `map`。RViz 点击只设置机器人在地图里的 x/y/yaw；水平基准由静止原始 IMU 比力方向经同一测量时刻的 TF 得到，不再把当前机身 roll/pitch 当作 0。三维 LIO 的高度、俯仰和横滚继续正常传播。

## 使用

现有 HeightJump、HeightObstacle、2D with TF 和 DrawPath 启动脚本仍调用同一个入口，无需编译 Python 脚本。

1. 启动 Livox 与 FAST-LIVO，等待其初始化完成。
2. 让机器人保持静止至少约 1 秒，再在 RViz 中点击 **2D Pose Estimate**；输入 frame 应为 `map`。
3. 查看 `initial_pose_tf` 终端：必须出现 **Initial pose accepted**。RViz 出现箭头或 AMCL 打印收到消息，均不能代替此脚本的成功日志。
4. 取消旧导航任务，确认切入导航控制模式后再发送新目标，避免手动等待期间触发无进展恢复。

若拒绝，终端会说明 IMU/TF 时间过旧、样本不足、机身正在移动、TF 缺失或输入 frame 错误等原因。等健康静止数据到齐后重新点击，脚本不会稍后自动执行一次失败的初始化。

## 重定位与重启

再次点击初始位姿时仍需静止。脚本保留此前重力水平和高度零点，只修正 x/y/yaw，起身或上下坡后的重新点击不会把本体高度重新压到 0。

FAST-LIVO 重启或 `camera_init` 原点重置后，先停止/取消导航，再显式清除旧标定：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 service call /initial_pose_tf/reset std_srvs/srv/Trigger '{}'
```

然后等待静止数据并重新点击初始位姿。地图基准改变后需要清理/重建受影响的高程图历史和规划缓存；此脚本不自动清地图、不自动取消导航。reset 停止发布新的对齐 TF，但 TF 消费者中旧缓存可能暂时存在，不能把 reset 当作底盘停车指令。

ROS 时钟倒退会自动丢弃标定。部分没有时间跳变的 LIO 原点重置无法可靠自动识别，因此不能省略显式 reset。不要同时启动多份该脚本发布同一段 TF。

## 默认参数

参数在启动时读取并只读，可用 `--ros-args -p 参数名:=值` 覆盖：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `imu_topic` | `/livox/imu` | 原始 IMU，必须带正确 frame、加速度和角速度 |
| `map_frame` / `odom_frame` / `base_frame` | `map` / `camera_init` / `base_link` | TF 坐标系 |
| `window_seconds` / `min_samples` | 1.0 / 20 | 静止窗口，至少覆盖窗口的 80% |
| `max_data_age` | 0.5 s | 原始 IMU 与最新 TF 的最大年龄 |
| `pair_tolerance` | 0.05 s | 最新 TF 与可配对 IMU 的最大差值；实际运算再查 IMU 精确时刻的 TF |
| `initial_pose_max_age` | 1.0 s | 手动位姿消息最大年龄 |
| `max_gyro` | 0.08 rad/s | 窗口内最大角速度模 |
| `max_acc_cv` | 0.03 | 加速度模的相对标准差上限 |
| `max_direction_spread_deg` | 2.0° | 比力方向最大偏离 |
| `max_translation` / `max_rotation_deg` | 0.02 m / 2.0° | 同期 5 个 TF 采样点相对窗口末状态的运动上限 |
| `initial_base_height` | 0.0 m | 首次初始化时 base_link 的高度参考值 |

固定 IMU 缓冲上限 2000 样本；超高频源或很长窗口可能无法满足跨度条件。窗口内不允许大于 `min(0.2 s, window_seconds/4)` 的数据空档。再次标定时测得的重力方向若比已建立的方向偏离超过 2°，拒绝更换水平基准，要求检查源状态并显式 reset。

**高度零点不等于真实地面测量。** 默认仍把首次 base_link 原点定义为 z=0；若它实际在地面以上，高程图中的地面可以是负值。后续点击保留这个零点。

## 边界

静止比力估计不使用 IMU orientation 字段，兼容原始加速度的 g 或 m/s² 单位。要求当前 IMU 加速度包含静止重力比力，角速度单位为 rad/s，传感器 frame 到 base/LIO 的旋转外参正确。原始 IMU 若已经去除重力，不能直接使用此方法。

加速度本身无法区分持续线性加速与重力；静止检查和人工静止操作都需要保留。TF 的发布 stamp 也不能证明内部处理扫描新鲜，这轮不修改 FAST-LIVO 的时序、速度输出或滤波器。旧包没有原始 IMU，不能用旧包直接验证此次重力估计。

发布仍为固定数值的 10 Hz 动态 TF，未混入静态 TF 或高频里程计改造。现场是否改善高程图或转圈，必须重新录制比较，不能由离线测试直接认定。

## 离线验证

```bash
cd /home/nvidia/MPPI_3dnavi
source /opt/ros/humble/setup.bash
python3 -m unittest discover -s tests -p 'test_*.py'

# 独立 ROS 域，仅发布测试 IMU/TF/initialpose，不发布速度：
ROS_DOMAIN_ID=143 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  python3 docs/analysis/initial_pose_tf_fix_2026-11-04/smoke.py
```
