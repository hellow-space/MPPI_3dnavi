# HeightJump 九点批量查询：实施与验证

**已实现并更新工作区 Release 插件。** 保持原有采样、地形阈值、unknown 策略和地图健康保护，将九点的重复坐标换算改为每轴三次转换；允许 unknown 通行时跳过不影响判定的起步区计算。

本轮未修改导航 YAML、控制频率、batch_size、time_steps、model_dt、运动模型或 critic 列表。没有启动实机导航或发布运动指令。现场 20 Hz 与转圈问题仍需重新录制验证。

**主要结果**

同进程配对评分基准，1000 条各 56 点，预热 10 轮、统计 100 轮，轮换三种实现的测量顺序，每轮比较所有轨迹的通过/拒绝结果：

| 场景 | 原 prepared 标量 P50 | 新批量 P50 | 原 / 新 P99 |
|---|---:|---:|---:|
| 平地，全部预测点通过 | 25.800 ms | **3.948 ms** | 25.955 / 3.976 ms |
| 台阶，轨迹可能提前拒绝 | 12.045 ms | **1.864 ms** | 12.123 / 1.956 ms |
| 全未知、拒绝 unknown | 0.445 ms | **0.059 ms** | 0.457 / 0.061 ms |
| 全未知、允许 unknown | 24.812 ms | **3.310 ms** | 24.889 / 3.396 ms |

平地内核耗时减少约 84.7%。该基准包含新查询对象的准备，不包含 ROS 传输、TF 或完整控制器；预测位置是固定随机种子的合成点，不能把这个百分比直接套到整机。

另用本机安装的 MPPI 优化器、真实可视化构建/发布以及 DDS 订阅者，在插件更新前后各运行相同合成平地场景。两侧均保留显示（trajectory_step=20、time_step=5），1000 × 56、参数、路径与固定起点相同：

| 工作 | 更新前 P50 / P95 | 更新后 P50 / P95 |
|---|---:|---:|
| MPPI 优化器 | 33.975 / 34.236 ms | **8.696 / 9.005 ms** |
| 优化器 + 轨迹显示 | 40.331 / 40.613 ms | **15.001 / 15.225 ms** |

这是持续运行的离线吞吐测试，随机采样未固定种子、机器温度与频率未固定，未包含真实高程建图、costmap 线程、底盘和录包并发。

按 50 ms 周期执行时，更新后的计算加显示 P50 24.77 ms、P95 27.98 ms、最大 29.41 ms；完成间隔 P50 50.32 ms、P95 56.46 ms、最大 58.72 ms。休眠条件下仍存在计算速度变化和调度抖动；本轮没有采集 CPU 频率，不能确定抖动来源，也不能称为实机稳定 20 Hz。更新前的同类周期测试计算 P50 40.20 ms、P95 42.27 ms，完成间隔 P95 51.97 ms。这里只能确定计算预算余量增加，不能宣称完成间隔抖动已改善。

**实现范围**

- `terrain_map.hpp/.cpp`：每轮查询缓存不可变地图几何；`neighborhood(x,y,r)` 分别转换三个 x、三个 y，按原顺序读取九格。标量 operator 和 TerrainSnapshot::query 保留作独立参考。没有按中心格缓存 Pass/Reject，也没有空间近似复用。
- `evaluator.hpp`：新增 evaluateNeighborhood，将九个 Sample 交回同一份 evaluate 逻辑，保留完整高度差、unknown 数量和决策优先级；仅在 unknown 明确允许时跳过豁免回调。
- `height_jump_critic.cpp`：接入批量查询；地图有效性、新鲜度、时钟倒退、TF、固定起步区、轨迹提前退出、固定拒绝代价和全失败标志不变。
- 测试与基准：补充批量查询、边界、unsafe 坐标、快照保活和统计优先级测试；扩展原 offline_terrain 差分与配对基准。

索引计算依据本机 [GridMap 2.0.1 的 GridMapMath 实现](https://github.com/ANYbotics/grid_map/blob/2.0.1/grid_map_core/src/GridMapMath.cpp)。保留 containment 与 getIndex 不同的减法顺序，保留除法而不改成乘倒数，并处理“任一轴 start index 非零时，两轴都执行循环索引”的规则。浮点到整数转换前有有限值及整数范围保护，最终存储边界仍检查。

**测试证据**

- 新增 unknown 允许测试在修改前失败：调用豁免 9 次，预期 0；`evaluator_red.log`。
- 新增批量 API 测试在实现前因接口缺失而编译失败；`batch_red.log`。
- 独立 CTest：2/2 通过。
- 本机插件：30 项 GTest，0 错误、0 失败、0 跳过；3 个 CTest 入口通过。包括真实 pluginlib 加载、GridMap 发布/订阅、TF、unknown/起步区与失效规则。
- 24 张合成地图、242304 个评估组合：批量与原路径的 decision、unknown_count、observed_jump 全部一致；另有 30288 个位置查询对照。
- 批量单元测试覆盖分辨率 0.05/0.10/0.13、半径 0.10/0.17、非正方形地图、两轴 start index 为零/单轴非零/双轴非零、内部与外部边界及 nextafter、原点偏移至 −4.2/−17.8 和 ±1e9、NaN/Inf、极大有限坐标的 FE_INVALID、防止越界读、旧快照保活。
- 已有 Python 测试：24 项通过。
- 独立审查未发现需返工的问题，并重新运行独立 CTest 2/2 通过。
- Release 编译参数包含 `-O3 -DNDEBUG -fno-fast-math`，安装库 `ldd -r` 未发现缺失库或未解析符号。构建有既有 GCC std::pair ABI 提示，独立构建有既有 OpenCV 多版本链接警告。

**部署状态与下一次验证**

已执行：

```bash
colcon build --packages-select height_jump_mppi_critic --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

新的共享库位于 `install/height_jump_mppi_critic/lib/libheight_jump_mppi_critic.so`。下一次重新启动对应导航即可加载。本轮改变 TerrainQuery 的内部布局，直接使用它的独立 C++ 程序应重新编译；不要把旧头文件、旧测试二进制和新库混用。Nav2 的 critic 插件接口未修改。

旧库备份：`/tmp/mppi_heightjump_before_batch_20261103/libheight_jump_mppi_critic.so`。修改前的头文件/critic 源码副本位于本目录，完整原版本仍在 Git HEAD `545feb0`。回退应恢复对应源码并重编译整个插件，不应仅替换库而保留不匹配的头文件。

下次实机跟踪目标时检查原始控制输出：

```bash
ros2 topic hz /command/cmd_twist
```

终端需 source 工作区，并使用实际导航的 ROS_DOMAIN_ID / RMW 设置。比较相同路线、相同显示和录制负载下的原始频率、最大间隔、超时告警以及转圈情况。最终 `/cmd_vel` 可能含平滑器或保活输出，不用于代替控制器真实更新频率。

**复现**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
cmake -S tests/offline_terrain -B /tmp/mppi_heightjump_batched_build \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/mppi_heightjump_batched_build -j2
ctest --test-dir /tmp/mppi_heightjump_batched_build --output-on-failure
/tmp/mppi_heightjump_batched_build/validate_terrain --benchmark
ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  colcon test --packages-select height_jump_mppi_critic
colcon test-result --test-result-base build/height_jump_mppi_critic/test_results --verbose
```

完整 MPPI 诊断沿用 `../mppi_frequency_2026-11-03/probe.cpp`，两侧分别在安装旧/新插件时运行并保存 `full_mppi_before/after.log`。其中参数使用该目录保存的 baseline_controller.yaml，并覆盖两个显示 stride 为 20/5；周期组额外设置 frequency_probe.periodic=true。诊断不创建运动指令发布者。

`metrics.json` 记录结果和参与源码/安装库 SHA256；`summarize.py` 从原始日志生成汇总。原始日志与差分输出均在本目录。
