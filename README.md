# HeightJump MPPI critic

独立 ROS2 Humble / Nav2 MPPI 1.1.20 插件。高度差只作可通行/不可通行二值判断，不引入连续地形代价，不修改 Nav2 optimizer、CuPy 建图或运控。

## 核心行为

- 每个 rollout 位置周围查询 3×3 共 9 个位置，间距为 `jump_radius`，计算有效高度最大值减最小值。
- 达到或超过 `max_height_jump` 即 REJECT，低于阈值还需满足 unknown 规则；可通行轨迹之间没有高程优劣排序。
- 0～2 个 unknown 暂按原逻辑容忍；至少 3 个 unknown 时，只有所有缺测采样都位于固定安全起步区内才 PASS。比原公司代码的“仅中心在圈内”边界更严格。
- 已知超限高度优先拒绝，即使同时有 unknown、即使在起步区。越界不豁免。
- 起步区首次健康评分时捕获，保存在 `fixed_frame`。新目标、普通 MPPI reset、`enabled` 切换不会移动它；同一实例返回原起步区仍可豁免。显式 cleanup/configure 重新初始化。
- 无地图、缺层、无效消息、过期或未来时间、无效 TF 均使本轮失败，不通过起步区绕过。ROS 时间倒退后锁定失败，需重新初始化。
- 每轮用同一只读地图快照，TF 每轮查询，不逐点等待；不保留原有按距离复用判断的近似缓存。

Nav2 接线沿用原实现的固定二值编码：PASS 加 0；REJECT 加固定 `1e6` 一次；全部 REJECT 时设置 `CriticData.fail_flag`。这个数与高度差大小无关，也不表示数学上的硬掩码淘汰。最终控制指令、二维碰撞检查、停车与恢复行为仍由 Nav2/运控链路执行和验证。

## 复制和构建

将整个 `height_jump_mppi_critic/` 目录复制到 Jetson 的 `~/MPPI_3dnavi/src/` 下，与其他 ROS2 包平级。不要复制本机 `.so`、build、install，也不要将源码放进 CuPy 仓库。

在 Jetson 新终端执行：

```bash
source /opt/ros/humble/setup.bash
cd ~/MPPI_3dnavi
rosdep install --from-paths src/height_jump_mppi_critic --ignore-src -r -y
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --packages-select height_jump_mppi_critic --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 pkg prefix height_jump_mppi_critic
```

依赖包括 `nav2_mppi_controller`、`grid_map_ros/core/msgs`、pluginlib、rclcpp 和 TF。用户板端已确认 Nav2 二进制包版本为 `1.1.20-1jammy.20260318.083257`。当前不要求 CuPy 的两个 commit 一致，只要求订阅内容遵守原始高程/NaN 的消息约定。

## 接入现有导航参数

1. 找到实际 MPPI controller ID（下文示例是 `FollowPath`，以本机为准）。在原有 `critics` 列表末尾追加 `HeightJumpCritic`，保留其他 critic。
2. 把 [参数片段](config/height_jump.yaml) 的 `HeightJumpCritic` 块合入该 controller 的配置。
3. 填写真实 `map_topic` 和经过标定的 `max_height_jump`；默认空话题/零阈值会使配置失败，避免假设实机可跨越 30 cm。
4. 如需起步豁免，在确认初始范围安全后配置 `startup_unknown_enabled: true`、实际固定定位 frame 和实测半径。默认关闭该豁免。不要将机器人跟随 frame（例如 base_link）当作 `fixed_frame`。
5. 用已经 source 本工作空间的终端重启原导航启动命令。插件不是独立节点，不需要单独 `ros2 run`。启动日志应出现 `Critic loaded : mppi::critics::HeightJumpCritic`。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `enabled` | true | Nav2 基类开关；动态修改不重置起步区 |
| `map_topic` | 空，必须填写 | ROS2 `grid_map_msgs/msg/GridMap` 实际话题 |
| `elevation_layer` | elevation | 原始高程层；不直接使用补洞值代替观测有效性 |
| `jump_radius` | 0.10 m | 固定9点模板的采样间距，必须为正 |
| `max_height_jump` | 0，必须填写 | 局部高度差二值阈值，必须为正 |
| `map_timeout` | 1.0 s | 同时限制接收间隔和消息时间年龄；按实测更新周期调整 |
| `startup_unknown_enabled` | false | 是否允许已确认安全的固定起步区豁免 |
| `fixed_frame` | 空 | 启用起步豁免时必须指定连续、固定的定位 frame |
| `unknown_ignore_radius` | 0.70 m | 公司仿真对照值；板端按安全范围标定 |

除基类 `enabled` 外，本插件配置在初始化时读取并声明为只读，普通动态参数不会重新捕获起点。修改配置应更新 YAML 并在安全起步位置**重启控制器节点**；仅 cleanup/configure 会重建起点状态，但不会删除节点上已经存在的只读参数。定位重置、地图坐标基准跳变后也要明确重新初始化；插件不能自动识别所有定位跳变。

地图和 rollout 的 frame 必须重力对齐；不同平面 frame 支持 x/y 平移、yaw 旋转，非平面 TF 会拒绝。发布的 GridMap pose.orientation 必须为单位旋转；CuPy 原始导出已满足这个约定。GridMap 标准转换器处理矩阵布局和滚动缓冲区，接收端不再手工翻转矩阵或重复添加 z 偏移。

订阅采用 best-effort、volatile、depth 1，可连接常见 best-effort 或 reliable 持续发布者；不依赖 latched 历史消息。话题存在但收不到时，应检查实际消息类型、发布者 QoS、ROS_DOMAIN_ID 和 controller 的执行器。

## 测试

2026-09-20 本机独立 Ubuntu 22.04 / Humble amd64 容器已完成 Release 构建：Nav2 MPPI `1.1.20-1jammy.20260908.012051`、GridMap ROS `2.0.1-1jammy.20260908.091310`。20个GTest用例全部通过（6个算法、5个地图、9个插件用例）；CTest有3个测试入口，colcon聚合显示23项、0失败。测试容器的构建日期与Jetson安装包不同，不表示已完成板端二进制兼容验证。

先停止该测试环境中的实际导航，或使用独立 ROS_DOMAIN_ID。以下测试只发布合成高程，不下发运动命令：

```bash
cd ~/MPPI_3dnavi
source install/setup.bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select height_jump_mppi_critic --event-handlers console_direct+
colcon test-result --verbose
```

- `test_evaluator`：二值阈值、1/2/3/9点缺测、混合已知危险、区域边界和固定起点。
- `test_map`：真实 GridMap 转换、NaN/Inf、地图滚动、畸形数据、时间健康检查。
- `test_plugin`：通过真实 Nav2 CriticManager/pluginlib 加载插件，真实 ROS2 发布/订阅、模拟 rollout、固定圈、TF 和失败标志。

测试不代替 Jetson 实机验收。仍需实际高程话题和 TF、CuPy 在盲区的数据保留、controller 失败后的停车与恢复、最终控制轨迹、ARM64 性能验证。消息头时间更新也不一定代表 CuPy 融合成功，建图异常需要结合其处理诊断。

## 已知范围与回退

当前只检查中心附近高程，**没有 footprint/Robot Guard，也不是悬崖保护系统**。固定9点和离散 rollout 点之间仍可能漏掉窄危险带；地图分辨率较粗时采样会落入重复格。二维障碍/全局规划与可通行台阶的语义冲突也不由本插件消除。

回退时，从原有 `critics` 列表删除 `HeightJumpCritic`，重启导航即可；CuPy 与 Nav2 主仓库没有修改。运行中设 `enabled: false` 会停止本插件检查，不能作为正常部署时解决 unknown 阻塞的办法。

## 文件职责

`evaluator.hpp` 是无 ROS 的二值判断与起步区状态；`terrain_map.*` 是 ROS2 消息、快照及地图健康；`height_jump_critic.*` 是 Nav2 插件、TF 与 rollout 接线。配置和测试随源码包一起交付。
