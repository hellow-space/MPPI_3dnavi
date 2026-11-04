# initial_pose_tf.py 代码审查

> 历史记录：下文针对修复前的脚本。后续修复及新验证见 `../initial_pose_tf_fix_2026-11-04/report.md`。本目录 `probe.py` 是原实现的缺陷复现，应配合修复前源码运行，不能把它的旧断言用于当前修复版本。

日期：2026-11-04。目标是用户高程导航实际采用的手动定位对齐脚本。仅做源码审查、离线回调/TF 运算与旧包复核，未改动定位脚本、导航参数或外部建图工作区，未初始化 ROS 节点或发布运动指令。

## 结论

核心公式和四元数转换正确；存在需要修正的输入校验、初始化水平基准、重定位生命周期与 TF 发布策略问题。不能据此认定这些问题已导致 `nav_elevation_2026-11-02_20-19-46` 的全部转圈。该包的 `map → camera_init` 平移和旋转全程恒定，没有这段变换反复跳动的证据；用户补充的提前发送目标、手动等待再接管，仍是解释无进展失败/旋转恢复的重要机制。

## 实际调用链

- `scripts/start_all_height_jump.sh:63` 启动 FAST-LIVO，`:75` 启动 raw elevation，`:86` 启动高程导航。
- `scripts/start_navigation_height_jump.sh:52` 与 HeightObstacle / DrawPath 入口都运行 `scripts/initial_pose_tf.py`。
- `src/fishbot_navigation2/config/nav2_params_height_jump.yaml:34` 为 `amcl.tf_broadcast: false`；HeightObstacle 同样关闭。
- `scripts/start_navigation_with_tf.sh` 的 2D 入口也运行同一手动对齐脚本；`2d_params.yaml` 也关闭 AMCL TF。因此不能把该脚本本身当作 2D/3D 唯一差异。
- AMCL 节点仍可能随 Nav2 bringup 启动、订阅 `/initialpose` 并打印日志，但其 TF 广播关闭。控制器的里程计是 `/aft_mapped_to_init`。启动时出现 AMCL 日志不等于当前 TF 由 AMCL 修正。

实际位置链：

```text
RViz /initialpose
    ↓ 一次对齐
map → camera_init                  initial_pose_tf.py 固定数值、10 Hz 广播
          ↓
       aft_mapped                  FAST-LIVO 动态状态
          ↓
       base_link                   安装外参
```

脚本没有点云配准、粒子滤波或持续地图匹配。它提供人工初始化的固定地图对齐，后续全局精度取决于 FAST-LIO 的相对定位与漂移。对短程、稳定 LIO 的系统可以使用；它不会主动消除累计地图偏差。

## 1. 核心数学正确

`initial_pose_tf.py:93` 查的是把 base_link 坐标转换到 camera_init 的变换，记为 `T_camera_base`。

```text
T_map_camera = T_map_base_desired × inverse(T_camera_base)
T_map_camera × T_camera_base = T_map_base_desired
```

这是正确的变换方向，不能把乘法顺序或逆变换随意替换。2000 组随机三维刚体变换验证，组合误差最大约 8.88e-16，四元数/矩阵往返误差约 1.19e-15。未发现正常单位四元数输入下的符号、分支或转向反号错误。

## 2. 高优先级：把点击时机身姿态当作地图水平基准

位置：`initial_pose_tf.py:106–125`。

RViz 的通常 2D 初始姿态只有 yaw，即把期望 base 的 roll/pitch 设为 0；代码又强制其 z=0。然后用当前完整三维 `T_camera_base` 的逆矩阵做对齐。

因此该脚本隐含以下假设：点击瞬间的 base_link 已水平，而且把该瞬间 base_link 原点高度定义为地图零高程。若点击时机身俯仰/横滚尚未稳定，瞬时机身倾斜会被吸收到固定地图旋转中。之后身体恢复水平，这个对齐不会自动重新计算。

离线调用原脚本回调复现：真实水平地面、点击时机身俯仰 10°、之后恢复水平，得到地图内底盘俯仰 −10°，地面沿一个方向延伸 2 m 时高度差约 0.3473 m。这是人为注入的演示条件，不代表旧包曾实际有 10° 姿态误差。

影响是高程地图水平基准/高度表达错误，并可能影响地形解释；不能由此直接推断必然触发 HeightJump。半径 0.1 m 邻域在 10° 平面上的高差仅约 0.035 m，低于当前 0.30 m 阈值，不能把跨 2 m 的高差冒充 0.2 m 邻域的障碍高度。

修正方向是用可信重力方向或稳定地面标定 roll/pitch，手动定位主要更新地图 x/y/yaw。**不能简单将 `map → camera_init` 的 roll/pitch 清零：camera_init 本来可能包含约 34° 的传感器初始化/安装倾斜，必须保留正确的三维补偿。** z=0 也只是基准选择，应明确 base_link 原点是否实际代表地面，不能自动称为地面零高程。

## 3. 高优先级：初始化不检查 TF 年龄，也不与输入时刻对齐

位置：`initial_pose_tf.py:93–97`。

`lookup_transform(..., Time())` 请求缓存中最新的 TF；“最新”不保证测量接近当前时间。脚本未读取返回 TF 的 stamp，也不检查 `/initialpose` 的 stamp，更不会验证上游处理的扫描时间。

离线使用真实 tf2 Buffer，放入时间为 1 s 的 `camera_init → base_link`，给原回调时间为 100 s 的初始位姿，回调仍然接受并记录成功。这说明没有年龄保护，不代表现场实际有 99 s 差值。

若点击时机器人还在动，或 TF 停更/上游正在处理历史扫描，就会把地图目标和另一时刻的机体状态拼在一起，形成固定的位置/角度误差。等待几十秒不会自动修正这个固定对齐；稳定后重新点击才会重新计算。

用户手动模式静止等待的操作可以减轻时间不一致的空间误差，但不能代替输入健康检查。即使加入 TF header 年龄限制，也需注意 FAST-LIVO 使用发布 `now()` 盖章，真实扫描年龄可能被隐藏；这一点应通过处理扫描时间与原始输入诊断补充。

依据：ROS 2 Humble tf2 Buffer 的 `time=0` 语义为查询最新变换，而不是强制查询当前时刻的新测量。

## 4. 明确的输入校验缺失：忽略 frame_id

位置：`initial_pose_tf.py:106` 开始，直接取消息 x/y/四元数，后续固定发布父 frame 为 map。

脚本不校验 `msg.header.frame_id`。离线给同一位姿分别设置 `map` 和 `odom`，回调输出完全相同，说明其他坐标系的输入会被静默当作 map 使用。RViz Fixed Frame 正常设为 map 时不会触发；来自其他工具或配置变更的初始位姿可能错位。

应要求输入属于 map，或者先转换到 map；空 frame、非有限位姿、零范数/非单位四元数也应明确校验。目前四元数转换依赖输入已经合法。

## 5. 定位/地图重置没有生命周期协调

位置：`initial_pose_tf.py:128` 直接替换缓存，`:134–158` 不检查上游状态地持续广播。

- FAST-LIO 重启、camera_init 原点重置后，脚本没有自动使旧对齐失效的机制；旧对齐若继续使用，会映射到错误位置。
- 再点 `/initialpose` 会立即改变全局对齐，没有与正在执行的目标、滚动高程地图历史或其他缓存协调。
- 当前高程节点按 TF 转换点云并融合既有地图；手动对齐脚本没有调用地图清理/重建接口。不能默认仅清 Nav2 costmap 就等于清理高程图历史。

应在上游坐标基准重置后明确重新初始化；运行中重定位应协调取消任务、重新对齐和受影响地图历史的处理。具体清理范围需要按最终坐标系设计确定，不建议在回调内无条件清掉所有地图。

本旧包 `map → camera_init` 全程数值恒定，因此以上是代码风险；没有证据表明该包中发生了反复手动对齐或该边的 TF 跳变。

## 6. 次优发布策略：恒定变换按 10 Hz 动态广播

位置：`initial_pose_tf.py:83` 与 `:145`。

固定数值持续盖当前时间戳，并不会提供新的地图匹配信息。tf2 查询完整链的最新共同时间时，这条 10 Hz 动态边可能使查询落在较旧的时间。

使用真实 tf2 Buffer 的离线复现：运动边有 8 s 和 10 s 数据，固定对齐边只发布到 9 s，整链最新查询得到 9 s 的插值位姿，而不是 10 s。

按旧包 recorder 当时已接收到的两段 TF 估算，该固定动态边可能额外限制共同时间的量级为 P50=0、P95≈53 ms、最大≈87 ms。这个估算不等于 controller 内实际缓存与查询结果，也不包含上游传感器处理年龄。

**不能把该节点 10 Hz 直接当作 MPPI 约 10.9 Hz 的成因。** 控制器调度不会因为 TF 发布 10 Hz 就被硬性限制为 10 Hz；它可以重复使用已有 TF。本脚本没有持续迭代定位算法，不能据其名字推断存在很高的计算负担。

若初始化后真的是固定对齐，可以考虑带明确重定位更新语义的静态 TF；若保留动态 TF，应设计与源状态时间一致的发布方式。不能只增大发布频率就宣称解决延迟，更不能只保留错误的旧状态却持续盖新时间。

## 7. TF 权威与启动风险

AMCL TF 在当前配置中关闭，因此没有证据表明它与此脚本争抢 `map → camera_init`。但不同启动入口使用不同 tmux 会话，只清理自己的会话；若多个导航入口同时留存，可能运行多个同名手动 TF 节点。部署应保证每条 TF 只有一个明确的发布权威。

这是潜在风险，不是已在旧包证实的冲突；旧包该边数值恒定，缺少发布者身份信息，不能声称已观察到争抢。

## 修改优先级

1. 先补输入 frame、四元数、TF/源观测年龄和初始化条件校验，明确失败原因。
2. 明确重力/地面水平基准，保留安装与 LIO 初始化倾斜的必要三维补偿。
3. 定义 FAST-LIO 重启、手动重定位与地图历史的联动方式。
4. 调整固定对齐的 TF 发布策略，再实测整链 pose 年龄；持续地图匹配属于后续能力选择，不是修正矩阵乘法。

这些是审查建议，本轮没有修改实际定位代码。Nav2 接受目标后即计时的机制仍需通过“取消旧任务 → 稳定定位 → 切入导航 → 重新发送目标”或显式开始流程解决，不能指望改 TF 脚本自动暂停导航任务。

## 复现

```bash
cd /home/nvidia/MPPI_3dnavi
source /opt/ros/humble/setup.bash
python3 docs/analysis/initial_pose_tf_review_2026-11-04/probe.py
```

`probe.py` 导入现有脚本，直接调用其回调，使用内存中的 tf2 Buffer；不调用 `rclpy.init()`，不创建 Node、listener 或 publisher。随机矩阵比较使用 SciPy 独立实现。结果保存于 `metrics.json` 与 `probe.log`；旧包附加估算使用已有 `/tmp/mppi_indoor_spin_201946_data.pkl`，没有该缓存时仍可运行其他复现项目。

参考：

- [Humble tf2 Buffer 实现](https://github.com/ros2/geometry2/blob/humble/tf2_ros_py/tf2_ros/buffer.py)
- [REP-105 坐标系语义](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
