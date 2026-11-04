# 高程修正前后：量化对照

日期：2026-11-04。这里区分旧包的离线计算、合成 GPU 实验、已有配置变化和现场验证，不能将它们混成实机导航效果。

## 旧包：同一批点、同一套记录 TF

数据来自 `nav_elevation_2026-11-02_20-19-46`。651 帧中均匀抽样48帧，46帧有精确观测时刻 TF，另2帧处在记录边界而跳过。缓存中919条 child_frame_id 为空的原始 TF 被忽略；没有据此生成或修改任何 TF。

只改变融合使用的物理原点和传感器相对点坐标，保持 map 中点位、门限参数和噪声系数不变。ROI 是以 base_link 为中心、平行地图轴的7×7 m方框，不等于逐帧实际 GPU 边界。观测变换通过整段 TF 离线查询，不模拟实时 TF 到达或新节点的时间拒绝逻辑。

全46帧中，旧原点相对雷达 TF 的偏差中位数仅0.0108 m，P95为11.388 m，最大14.011 m。说明机器人靠近初始点时差别小，远离初始点后错误显著。以下进一步列出偏差超过1 m的17帧，明确这是按位移条件选出的子集。

| 指标，17帧中位数 | 修正前 | 修正后 |
|---|---:|---:|
| 融合原点与记录雷达 TF 原点的偏差 | 5.207 m | 0 m（使用同一 TF） |
| 两者都通过门限的 ROI 点，逐帧平均测量方差 | 2.136454 m² | 0.300119 m² |
| ROI 内通过距离/高度门限的点数 | 5224 | 4145 |

- 逐帧方差比值“前/后”的中位数7.449，最大38.350；它不等于两列独立中位数的比。
- 每帧改变门限判定的 ROI 点数中位数962，最大1580。减少通过点数不自动证明删掉的都是真正无用点；这里只验证门限相对正确雷达原点执行。
- 全46帧的点在 map 中位置因 float32 运算产生的最大变化约 **0.0121 mm**；不是将整个点云挪到新位置。
- 0 m 表示代码采用记录中的雷达 TF，**不是宣称雷达外参或真实定位没有误差**。测量方差是现有 `0.05 × 距离²` 模型的输出，不是实测高度误差。
- 没有重放完整 GPU 时序、遮挡清理或机器人导航，因此这些值不能推导转圈发生率和端到端延迟。

完整数据：[bag_comparison.json](bag_comparison.json)、[逐帧CSV](bag_comparison_frames.csv)。

## 合成平面：实际 GPU 融合，与独立原生坐标输入对照

使用既有 GPU 测试场景：固定点云原点 map x=10 m，实际传感器 map x=1 m；88个平面点、重复13帧、真值 z=-0.5 m，地图4×4 m、分辨率0.1 m。同一组参数分别执行旧适配、新适配和独立的原生传感器坐标参考；参数及关闭项完整记录在 JSON。关闭 visibility/overlap 清理、放宽离群门限是为了隔离融合原点和噪声模型，不能将结果当成现场精度。

| 88个共同有效格 | 修正前 | 修正后 |
|---|---:|---:|
| 高度相对合成真值的平均绝对误差 | 15.339 mm | 0.0779 mm |
| 格子平均融合方差 | 0.306783 m² | 0.00155745 m² |
| 高度相对原生传感器参考的最大差异 | 16.873 mm | 0 mm |
| 高度门限以上的测试点，错误建图格数 | 1 | 0 |
| 距雷达仅5 cm的测试点，错误建图格数 | 1 | 0 |

新适配与原生参考的方差最大差异也为0。上述误差下降反映特定重复无噪声观测下的融合收敛行为，不是实机精度承诺。

完整数据：[gpu_comparison.json](gpu_comparison.json)、[运行日志](gpu_comparison.log.gz)。

## 性能和地图大小

- 已保存的18000点适配层基准：200次测量、30次预热，新增 CPU 几何转换 P50 **1.336 ms**、P95 **1.373 ms**。该修正是正确性修正，会增加转换工作；没有数据证明它降低了总延迟或提高 MPPI 频率。
- 本次一并提交的既有配置把地图边长20 m改为7 m，分辨率保持0.1 m。有效格数理论上从200×200=40000变为70×70=4900，减少87.75%；包含外圈的内部格数从202²=40804变为72²=5184，减少87.30%。这是格数计算，不是实测耗时或整体内存下降比例，覆盖范围也随之缩小。
- raw 默认地图 frame 从 camera_init 改为 map，与 local costmap 一致。旧约34°安装外参不等于机身姿态，也不能替代本次重力定位修正后的实测地图角度；这里不再用“map→base_link 姿态必然为0”证明地图水平。

## 本轮提交前新运行的验证

- 主工程 `python3 -m unittest discover -s tests -p 'test_*.py'`：52项通过，11.034 s。
- 容器 `colcon build --packages-select elevation_mapping_cupy --symlink-install`：成功，1包完成，1.41 s。
- 高程 callback、launch、真实 GPU、raw 模式及 frame 默认值5个测试文件：23项通过，2.70 s。
- ROS domain144/localhost：实际 PointCloud2/TF→GPU→GridMap 链路、移动物理原点、拒绝缺失 TF 后不刷新观测时间，3项检查通过；日志见[ros_smoke_recheck.log.gz](ros_smoke_recheck.log.gz)。未发布速度指令。
- 前次完整 colcon 15组中6组因 PyTorch 缺少 libcudss.so.0 失败，未解决也未在这轮重复运行。当前 raw 模式不加载该后端；不能称完整测试全绿。

没有修改后的新实机 rosbag。地面覆盖率、稀疏程度、漏检率、转圈情况、全链路延迟和 MPPI 频率目前都没有可靠前后对照，不能给出改善百分比。

## 复现

```bash
cd /home/nvidia/MPPI_3dnavi
source /opt/ros/humble/setup.bash
python3 docs/analysis/elevation_sensor_origin_2026-11-04/compare_bag.py

# 容器仅需运行空闲 bash；以下程序不初始化 ROS，也不发布任何话题。
docker cp docs/analysis/elevation_sensor_origin_2026-11-04/compare_gpu.py \
  elevation_env:/tmp/compare_sensor_origin_gpu.py
docker exec elevation_env bash -lc \
  'source /opt/ros/humble/setup.bash && source /root/elevation_map/install/setup.bash && python3 /tmp/compare_sensor_origin_gpu.py --output /tmp/sensor_origin_gpu_comparison.json'
```
