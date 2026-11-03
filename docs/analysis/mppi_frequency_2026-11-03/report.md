# MPPI 更新频率：可视化开销对照与首轮调整

用户确认 2D、3D 都使用 FAST-LIO2 的 `/aft_mapped_to_init`，RViz 预测轨迹正常而执行时转圈。本次优先处理 MPPI 更新频率，保留这一路 odom、运动模型、1000 条采样、56 个预测步、地形阈值和全部评分项。

**结果：找到并调整了一个明确的耗时项。** 本机已安装 MPPI 优化器与可视化代码的离线对照中，原始显示配置的“优化 + 可视化”中位耗时约 70.29 ms，降低显示密度后约 39.93 ms，减少约 30.36 ms / 43.2%。50 ms 对应目标 20 Hz；调整后合成场景具备接近该频率的计算余量。尚未运行实机导航，不能据此声称现场已经恢复 20 Hz 或已经解决转圈。

**测量结果**

| 条件 | 优化器 P50 | 标记构建 P50 | 发布 P50 | 合计 P50 / P95 |
|---|---:|---:|---:|---:|
| 当前显示：trajectory_step=5、time_step=3 | 33.84 ms | 1.08 ms | 35.37 ms | 70.29 / 70.77 ms |
| 调整显示：trajectory_step=20、time_step=5 | 33.64 ms | 0.24 ms | 6.06 ms | 39.93 / 40.30 ms |
| 关闭显示，仅作为诊断 | 33.94 ms | 约 0 | 约 0 | 33.94 / 34.45 ms |
| 关闭显示和 HeightJump，仅作为诊断 | 4.39 ms | 约 0 | 约 0 | 4.39 / 4.91 ms |

各列分位数独立计算，不能要求列分位数相加恰等于合计分位数。

开启显示的两种条件均接收了 70 组 MarkerArray，关闭显示时接收 0 组，证明测试确实执行了预期的可视化分支。加入订阅者以触发实际 DDS 消息传输；“发布”阶段包括调用库 visualize() 内的处理与 publish，不是仅测一个网络发送函数。

额外按 50 ms 周期执行调整后的配置：完成间隔 P50 50.11 ms、P95 53.32 ms、最大 55.01 ms；计算加显示 P95 44.95 ms、最大 45.51 ms。存在抖动，且没有包含实机传感器、高程建图、真实 costmap 更新、RViz 渲染和 rosbag 写盘的并发负载，不能把这些结果称为真实控制器端到端 20 Hz 验证。

**为什么先改显示**

19:50 故障包中原始 `/command/cmd_twist` 实际约 11 Hz，目标 20 Hz。`/trajectories` 记录 361 条，每条平均约 933 KiB，合计 329 MiB，占该包最大份额。离线测量说明：原显示开销本身已让“优化 + 显示”超过 50 ms；这一项足以成为降频因素。完整现场为何达到约 91 ms 平均控制间隔，仍包含其他负载，不能全部归给可视化。

本次保留 `visualize: true`，只减小显示出来的候选轨迹及其采样点数量。它控制 RViz/MarkerArray 的显示密度，不改变优化器实际评估的 1000 × 56 个状态，也不改变最终轨迹选择算法。

**实际改动**

两份配置：

- `src/fishbot_navigation2/config/nav2_params_height_jump.yaml`
- `src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml`

```yaml
FollowPath:
  visualize: true
  TrajectoryVisualizer:
    trajectory_step: 20
    time_step: 5
```

执行 YAML 结构对比，确认除这两个显示参数外，与本轮修改前的配置在数值/结构上完全相同。既有未提交改动已保留，本目录 `before_*.yaml` 是本轮修改前的快照。运行中的参数没有被修改；当前没有启动实机导航。

本机 install 中两份配置都是指向源文件的符号链接，下一次启动对应导航即可读到新配置，无需重编译 C++ 插件。

**验证与边界**

- `python3 -m unittest discover -s tests -p 'test_*.py'`：24 项通过。
- `python3 tests/test_height_obstacle_config.py`：通过。
- 本轮 YAML 差异审查：仅显示 stride 两项发生语义变化，控制频率仍为 20、model_dt 仍为 0.05、batch_size 仍为 1000、time_steps 仍为 56、HeightJump 仍启用。
- 离线测量使用当前 HeightJump 配置、7 × 7 m、0.1 m 分辨率的平地、高程图每轮更新、固定起点与直线路径。预热 10 轮，统计后续 60 轮。安装的 MPPI 仍使用自己的随机采样，未固定随机种子；这是性能对照，不是逐指令等价性测试。
- 独立 ROS_DOMAIN_ID=142、ROS_LOCALHOST_ONLY=1，未创建 `/cmd_vel` 或 `/command/cmd_twist` 发布者。使用本机安装的 `Optimizer::evalControl()` 与 `TrajectoryVisualizer`，不是重写算法的替代品。匹配本机库的 xtensor / xsimd 32 字节分配器 ABI。
- 节点使用真实系统时钟、地图与规划在测试中直接供给。完整 ControllerServer 的 TF、路径转换、进度检查和真实 costmap 工作线程未纳入，合计只代表优化与显示。
- 本机功耗模式查询为 MAXN_SUPER，CPU 0～7 在线；未调整功耗、CPU 频率或后台桌面负载。测试按模式顺序执行，可能有温度和调度漂移。
- 链接存在本机已有 OpenCV 4.5d / 4.10 共存警告，程序运行与配置测试通过；本轮未修改依赖。
- HeightObstacle 配置也做相同的显示降载，但本轮计时针对纯 HeightJump，不代表包含 CostCritic / 高度障碍层的全链路耗时。

**下一次现场核对**

重新启动对应导航，在实际跟踪目标的过程中测控制器原始输出，而不是测可能由平滑器/保活节点补发的最终 `/cmd_vel`：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic hz /command/cmd_twist
```

对比同一环境、同一路径、相同录包/显示负载下的实际频率、最大消息间隔、控制周期超时以及转圈现象。如果仍低于目标，再对 HeightJump 逐项测量；此次平地消融估计其增加约 30 ms，可作为下一轮性能重点。不要通过把 `controller_frequency` 降到 10 Hz 来掩盖原先 20 Hz 的预算问题，也不要直接关闭生产配置中的唯一地形评分。

**离线复现**

```bash
bash docs/analysis/mppi_frequency_2026-11-03/run_probe.sh
```

脚本在 `/tmp/mppi_frequency_probe_build` 独立构建，运行五组诊断并更新本目录日志与 `metrics.json`。`baseline_controller.yaml` 保存本轮修改前用于对照的控制参数；`summarize.py` 汇总数据并记录参与测量的源码和安装库 SHA256。关闭 HeightJump 的条件只用于合成诊断，未应用到任何实际导航配置。
