# 高程查询加速与离线验证（第一批）

本文保留第一批结果。后续高度层扫描优化与最新测试状态见 [第二批结果](height_scan_optimization_results_2026-10-18.md)。

日期：2026-10-18。范围：HeightJumpCritic 的查询路径，以及高度障碍层 Release 构建。导航参数、采样数量和判定阈值不调整。

## 实现

评分开始时构建 `TerrainQuery`，一次查到 elevation 矩阵，持有不可变地图快照，随后九点采样直接读取同一矩阵。查询函数内联；传给 evaluate 的 lambda 只捕获引用，避免每个采样位置复制 shared_ptr。原 `TerrainSnapshot::query()` 保留为修复后参考路径。

有限坐标检查、`isInside()`、`getIndex()` 和两轴索引范围检查全部保留。合法单元读取、循环缓冲索引、NaN/Inf 转 Unknown、九点偏移、阈值比较、未知策略、起步豁免和地图过期规则不调整。没有增加邻域预计算，也没有改变栅格取整公式。

## 同进程配对性能

真实 GridMap 查询，7 × 5 m 地图、0.10 m 分辨率，1000 条各 56 点的固定随机位置。每个场景预热 10 轮，测量 100 轮；每轮交替先运行参考或优化路径，逐轨迹比较全部决策。优化路径计时包含 TerrainQuery 构造与销毁。

| 场景 | 参考 P50 | 优化 P50 | 耗时减少 | 参考 P99 | 优化 P99 |
|---|---:|---:|---:|---:|---:|
| 平地，检查完整步数 | 40.789 ms | 26.213 ms | 35.7% | 41.124 ms | 26.683 ms |
| 台阶，部分提前拒绝 | 19.026 ms | 12.369 ms | 35.0% | 19.118 ms | 12.498 ms |
| 全未知，首点拒绝 | 0.712 ms | 0.450 ms | 36.7% | 0.743 ms | 0.460 ms |

数据为当前 Jetson 本机的评分内层基准，包含 evaluate 和轨迹提前退出，不含 TF、ROS 传输、其他 critic、高度层更新或完整 MPPI 控制循环。随机位置不符合运动模型；基准未知策略为默认拒绝，不是实际导航配置的完整复刻。未固定机器温度和功耗模式，同进程交替测量用于降低顺序偏差；不将这些比例解释为整机控制周期同比加速。

第一次实验性版本省去 isInside 时曾测得 17.9 ms；该版本没有采用，因为库在检查范围之前先做浮点到整数转换，极大有限坐标会触发异常。最终使用上表保留预检的版本，不能引用 17.9 ms 作为交付收益。

## 验证与审查

- 合成地图三路对照：原地图参考、修复后的原查询、新 TerrainQuery；比较查询状态、有效高程和九点判定的 decision / unknown_count / observed_jump。涵盖 0.05/0.10/0.13 m、滚动索引、边界与相邻浮点、NaN、未知策略、起步区域、非整除半径及极大坐标。
- 增加快照生命周期测试：原快照外部引用释放、替换成未知新地图后，旧查询仍读取原图；空快照明确拒绝。
- 独立审查发现省略 isInside 会扩大极大坐标的不安全路径。新增浮点异常回归，在实验性版本上复现 FE_INVALID，恢复预检后通过。
- 独立 CTest 通过；24 组地图完成 30,288 次查询、242,304 次九点结果对照，差异为零，异常索引仍计数 200 次。
- 当前工作区的 height_jump_mppi_critic 与 height_obstacle_layer 均已 Release 构建并安装到原有 install；两者编译参数均确认包含 `-O3 -DNDEBUG -fno-fast-math`。高度层源码未改，只有构建模式由空值切为 Release，未单独测量其收益。
- 新版库上的原生测试共 5 个 CTest 入口、28 个 GTest 用例通过，其中 HeightJump 插件测试 11 项；仓库 Python 测试 24 项通过。colcon 聚合计数包括 CTest 包装入口，分别为 28 和 5，均无失败。
- 本地构建前确认没有运行中的导航进程。没有启动机器人、仿真或更改导航参数；下次使用当前工作区启动对应高程导航配置时会加载新库。

## 运行入口与报告

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select height_jump_mppi_critic height_obstacle_layer \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
python3 scripts/run_offline_validation.py --terrain-benchmark --ros-tests \
  --output /tmp/mppi-query-report-next
```

报告目录必须不存在。`terrain_benchmark.log` 分别记录 `baseline_rollout` 与 `prepared_rollout`；基准在同一进程比较两种路径。实际部署源码前应确认当前包已重新构建，`--ros-tests` 使用工作区 build 下的二进制。

本轮主要日志：`/tmp/mppi-query-final-benchmark.log`、`/tmp/mppi-query-final-correctness.log`、`/tmp/mppi-query-final-map-tests.*`。本地构建前旧库及 CMake 缓存备份位于 `/tmp/mppi-before-query-optimization-20261018/`。临时目录可能被系统清理，主要数字保留在本文。

本地包构建日志位于 `/tmp/mppi-query-local-build-log-20261018/`，测试日志位于 `/tmp/mppi-query-local-tests.log` 和 `/tmp/mppi-query-local-test-log-20261018/`。独立构建仍有此前的 OpenCV 版本链接警告，本轮未修改依赖环境。

本批完成 O1 与 O4，并完成 O5 的 MPPI 查询部分。高度层扫描重写、邻域统计预计算、共享分类结果、脏区域和采样参数降载尚未实施；本轮不需要它们来获得上述收益，也不能将其标为已验证。
