# TerrainSnapshot 地图边界修复验证

本文记录边界修复阶段。当时未替换原安装；后续查询加速已完成本地构建及测试，最新状态见 [高程查询加速与离线验证](query_optimization_results_2026-10-18.md)。

日期：2026-10-18。独立测试及隔离构建的完整插件验证均已完成。

## 修复与行为范围

地图位置在浮点边界附近时，GridMap 可能判断位置在地图内，却返回等于存储尺寸的索引。旧 `TerrainSnapshot::query()` 在 `isInside()` 后直接调用 `atPosition()`，不能阻止该非法索引访问。

修复保留原有有限坐标及 `isInside()` 检查，再获得索引并检查两轴范围；合法时读取相同单元，非法时返回 Outside。默认最近邻读取方式、NaN/Inf 高程转 Unknown、高度差阈值、未知策略、起步豁免、地图时效及导航配置均未调整。

这是一项边界保护修复，不是所有输入严格等价的优化：先前可能误读成 Known/Unknown 的非法存储索引，现在按地图外处理。合法单元查询应保持一致，地图最后一格中心仍可读取，没有新增一圈不可查询区域。

## 已完成的验证

1. 新增 `TerrainMap.RejectsRoundedIndicesBeyondStorageAtMapEdges`。修复前运行失败，两个问题点返回 Known 而非 Outside；修复后通过。覆盖两轴四边与相邻浮点位置，同时检查最后一格中心保持 Known。
2. 独立地图 GTest 共 6 项通过，包括滚动缓冲、非正方形地图、未知高程、格式校验与时间戳。
3. 合成地图对照 24 组全部通过，完成 30,240 次查询、241,920 次九点判定比较。共计 200 次参考查询检测到异常索引并按 Outside 处理；这些输入仍进入实际查询与判定对照，没有被跳过。
4. 仓库 Python 测试共 24 项通过。
5. 独立代码审查未发现阻塞源码修复的问题；要求明确区分修复前基线与修复后状态，本文及原报告已作区分。
6. 在 `/tmp/mppi-boundary-package-20261018/` 隔离构建当前 height_jump_mppi_critic 包，并以该临时安装环境执行全部 3 个 CTest 入口、23 个 GTest 用例，全部通过。其中 11 个插件测试实际加载新版库并执行评分；`ldd` 确认链接的是临时安装目录中的新库。原工作区的 `build/`、`install/` 未替换。
7. 一键入口以 `--terrain-benchmark` 复跑成功，`report.json` 中 `passed=true`、无失败步骤。本次平地基准 P50 为 40.799 ms、P95 为 41.058 ms、P99 为 41.203 ms；台阶 P50 为 18.979 ms，全未知 P50 为 0.709 ms。未固定功耗与温度，不用这些数字宣称加速收益；仍不包含完整控制周期。

原有的预构建 ROS 测试不能证明这次源码修复已通过完整插件验证；上述第 6 项使用的是本轮重新构建的产物。

## 运行与产物

独立构建与 CTest：

```bash
source /opt/ros/humble/setup.bash
cmake -S tests/offline_terrain -B /tmp/mppi-terrain-check-next -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/mppi-terrain-check-next --parallel 2
ctest --test-dir /tmp/mppi-terrain-check-next --output-on-failure
```

原有一键入口已新增执行新地图回归测试：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/run_offline_validation.py --terrain-tests --output /tmp/mppi-terrain-report-next
```

源码和测试已修改；当前工作区原有 `build/`、`install/` 及运行参数未替换。临时构建只用于验证，不表示机器人已经使用修复版本。

本轮产物：修复前失败日志 `/tmp/mppi-boundary-regression-red.log`；修复后回归日志和 XML `/tmp/mppi-boundary-regression-green.*`；完整插件测试位于 `/tmp/mppi-boundary-package-20261018/build/height_jump_mppi_critic/test_results/`；最新一键报告位于 `/tmp/mppi-boundary-report-green-20261018/report.json`。临时目录可能被系统清理，主要结果保留在本文。

原有 OpenCV 版本链接警告仍保留于构建日志，依赖环境未改动。性能数据也不等于完整控制周期；该修复不预设加速收益。
