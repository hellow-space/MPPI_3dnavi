# 高程图实验设计与 rosbag 录制充分性评估

日期：2026-10-17。对象：当前工作区的高程导航配置、录包脚本、三份已有导航 bag，以及同机 `/home/nvidia/elevation_map` 中的 raw 高程配置。

## 1. 结论

你的“平地 → 平地绕障 → 斜坡 → 楼梯”顺序合理，覆盖了从基础建图到连续起伏、离散高度突变的主要场景，适合作为实验主线。但仅按四种场景各跑一次、观察 RViz，不能充分证明高程图精度、更新能力和障碍判定有效。

建议保留四大类，在每类中加入明确真值、运动条件、评价指标和重复实验，并横向增加：阈值边界、未知/遮挡区域、地图滚动与重访、障碍移除、输入中断。将“地图正确”“阈值判断正确”“导航行为正确”分开评价。

当前录制方案不足以完整支撑这套实验：

- 已有三份 bag 都没有高程图和输入点云，只能用于部分导航分析。
- `record_terrain_elevation.sh` 适合基础建图实验，但缺少高度差图、代价地图及导航行为。
- `record_nav_elevation.sh` 有高程图和导航信息，却缺输入点云、高度差图和局部/全局代价地图。
- 应优先把输入点云、完整高程 GridMap、高度差图、TF、位姿、两类 costmap 和任务结果串成同一次录制。

本文是静态代码审查和 bag 内容清点，不是实机实验结果；没有改变现有录包脚本、导航参数或机器人状态。配置值必须在实验前用实际节点参数再次核对。

## 2. 当前实现决定了哪些实验必须做

### 2.1 高度差不是物体的绝对高度

依据 `src/height_obstacle_layer/include/height_obstacle_layer/height_difference.hpp`、`src/height_obstacle_layer/src/height_obstacle_layer.cpp` 和 `src/height_jump_mppi_critic/include/height_jump_mppi_critic/evaluator.hpp`：

每个位置在 X/Y 方向取偏移为 `{-r, 0, +r}` 的九个采样位置，对有效高程计算：

```text
Δh = max(有效采样高程) - min(有效采样高程)
Δh >= H → 障碍/轨迹拒绝
```

当前 `nav2_params_height_obstacle.yaml` 中 `r=0.10 m`、`H=0.30 m`，局部和全局高度层、HeightJumpCritic 使用相同阈值。你的“超过阈值”需修正为“达到或超过阈值”。0.30 m 只是配置值，不能据此认定机器人可跨越 0.30 m。

直接影响：

- 高平台内部可能没有明显局部高度差，边缘才触发障碍；不能要求物体整个顶面都判为障碍。
- 过窄物体可能落在离散采样点之间；需比较不同宽度和栅格对齐位置。
- 楼梯判定取决于窗口内跨了几级，而不只取决于单级高度。
- 对理想连续平面 `z=ax+by+c`，九点采样的高度范围为 `2r(|a|+|b|)`。沿网格轴的坡面约为 `2r·tan(θ)`，45°方向同坡度可达 `2√2r·tan(θ)`。这是忽略栅格离散、缺测和噪声的解析推导，说明要测坡向，不能将高度差阈值直接当成坡度阈值。
- 这个判据不表达打滑、机身姿态、接触稳定性；通过高度阈值只能证明当前几何规则允许，不能单独证明物理可通行。

### 2.2 当前地图、未知区域和过期策略

同机 raw 配置 `.../elevation_mapping_cupy/config/setups/raw/elevation.yaml` 输入 `/cloud_registered`，发布 `elevation`、`variance`，目标发布频率 10 Hz，默认 `map_frame=map`、`base_frame=base_link`。core 配置高程分辨率 0.10 m、地图边长 7 m；导航 costmap 分辨率 0.05 m。更细的 costmap 不会增加高程观测的真实分辨率。以上为源码配置，launch 覆盖及实际安装版本仍需核实。

raw 模式未提供生产用 `slope`、`roughness` 输出；可以离线从高程计算坡度，但不能在报告中把未发布的层说成已有测量。`variance` 单位为 m²，描述估计不确定性，不等同于地面粗糙度或实际高度误差。

当前高度障碍配置：

- `unknown_as_obstacle=false`，HeightJumpCritic 的 `unknown_is_traversable=true`，全局规划 `allow_unknown=true`。未知区域通过不等于已确认平整，应单独统计有效覆盖率。
- `map_timeout=1.0 s`。高度层过期时保留最近有效障碍快照并将自身置为非 current；HeightJumpCritic 遇到缺失/过期地图会置失败标志。系统最终是否停住、延迟多久，仍需结合速度平滑、底盘和任务状态实测。
- HeightObstacleLayer 要求 GridMap frame 与 costmap global frame 名称一致；该回调没有自动 TF 转换。当前两者配置为 `map`。
- 地图必须在重力对齐坐标系中评价。相邻高程工作区已有坐标系倾斜的历史记录，因此平地测斜率既是精度测试，也是 TF 基准检查。

## 3. 四类实验的测试点与增强方案

| 实验 | 原方案能验证什么 | 必须增加的子工况 | 主要评价量 |
|---|---|---|---|
| 平地 | 建图是否工作、平面是否稳定 | 静止；直行；转弯/原地转向；往返重访；驶过滚动地图边界；至少两档速度 | 高度偏差/RMSE、固定世界位置的时间波动、拟合坡度、有效覆盖率、重访差异、更新间隔 |
| 平地绕障 | 高度突变是否进入障碍判定和导航 | 阈值下/边界/上；窄/宽障碍；正向/斜向接近；移入/移除；有绕行路线/没有绕行路线 | Δh误差、漏检/误检、障碍建立与清除延迟、最小净空、成功率、任务时间 |
| 斜坡 | 连续高程变化和坡面重建 | 上坡/下坡；不同已知角度；坡向相对地图轴0°/45°/90°；坡底和坡顶过渡；必要时坡侧接近 | 坡角误差、坡面残差、边界定位误差、有效覆盖率、Δh与理论一致性、姿态相关误差 |
| 楼梯 | 离散阶跃和重复结构重建 | 单级后多级；不同踏步深度；阈值下/附近/上；上行视角/下行视角；正向/斜向扫描；遮挡边缘 | 单级高度和深度误差、边缘定位、可见级数、跨级窗口效应、缺测比例、正确拒绝率 |

“上行/下行视角”可以是从两侧观察或受控采集，不要求机器人实际爬楼。若硬件能力尚未验证，斜坡/楼梯先做感知与判定实验；对于设为不可通行的结构，正确停止或绕行才是成功。

### 3.1 平地：分开测精度、稳定性和更新

建议静止观察 30–60 s，再按固定路线运动、转向、返回，重复经过同一地面区域。

1. 精度：用独立测量的平面评价；若不知道绝对高程零点，报告对齐方法和相对误差。不能每帧自由拟合并消除全部倾斜后，宣称绝对高度及坡度都准确。
2. 稳定性：对固定世界坐标 ROI 的有效栅格计算时间标准差和长期偏移。滚动 GridMap 的同一数组下标可能对应不同地面，必须根据地图几何、位置和循环索引对齐。
3. 更新：记录新的地面进入视野后，首次有效建图及稳定收敛的时间。发布频率稳定，不代表每帧都有新的点云融合。
4. 重访：比较同一区域两次经过时的高度和边缘位置，识别定位漂移与地图移动带来的伪变化。

### 3.2 平地绕障：先证明判定，再证明绕行

设置 `H-δ`、约 `H`、`H+δ` 的台阶/障碍高度，δ应大于测量不确定度并结合实测噪声确定。实物“正好等于阈值”无法严格保证，边界精确语义用离线可控数据验证，实机报告临界区间内的判定翻转率。

每次对齐观察“点云 → elevation → height_difference → costmap → 路径/速度”。若绕障成功，但高程层没有识别，不能据此给高程算法记成功；静态地图或其他障碍层可能已经影响路径。

增加障碍移除后持续观察，测残留障碍清除时间。增加“无可行绕行路线”工况，期望受控停止/任务失败，不把到达目标作为唯一成功条件。

### 3.3 斜坡：坡面与过渡边缘分别统计

用倾角仪/几何尺寸测量真实坡度，在坡面内部拟合 `z=ax+by+c`，计算 `θ=atan(sqrt(a²+b²))`。坡底/坡顶单独设 ROI，避免边缘混合栅格污染坡面精度。

比较上坡和下坡观测的覆盖率、边缘偏差及机身姿态。高度差阈值允许陡坡时，不代表该坡物理可爬；如果实验目标还包括最大坡度限制，需要另有坡度判据，当前代码没有完整表达这个目标。

### 3.4 楼梯：验证分辨率和遮挡限制

先用单级台阶测清楚高度和边缘，再增加多级。记录每级高度、踏步深度、宽度及相对地图方向；尤其比较踏步深度接近高程栅格大小、接近采样窗口跨度和明显大于窗口的情况。

单级低于 H，但窗口跨两级后总差达到 H，是当前局部高度差规则的预期结果，不能简单归为误判。反之，只看到顶面而没有看到落差时，不能将“没有障碍”解释为正确识别了可通行楼梯。

2.5D 高程图每个 XY 位置表达一个高度，不能完整描述竖直立面、悬空和多层重叠结构。楼梯评价应聚焦可观测踏面、边缘和高度跃迁。

### 3.5 横向补充项：不必新增第五个大场景

| 补充测试 | 嵌入位置 | 为什么需要 |
|---|---|---|
| 遮挡、稀疏点云、未知区域 | 绕障和楼梯 | 当前 unknown 允许通过，容易将缺测误当作通行能力 |
| 负台阶/坑边 | 楼梯的下行观测 | 落差同样有高度差，但通常更难观测，缺测不能等于无风险 |
| 输入停止/恢复、时间戳异常、地图越界 | 优先离线回放或受控静止条件 | 验证过期、冻结地图、越界与恢复行为 |
| 粗糙平地、窄障碍、不同表面回波 | 平地/绕障扩展项 | 区分噪声、采样缺失和真实局部高度差 |
| 重启冷图与保留历史地图 | 四类均适用 | 控制先验地图对建图时间与成功率的影响 |

## 4. 让结果可量化、可比较

### 4.1 指标定义

| 指标 | 定义/注意事项 | 所需数据 |
|---|---|---|
| 高度 MAE/RMSE、P95绝对误差 | 对独立真值、固定坐标和有效区域计算；同时报告有效样本数量 | elevation、TF、真值 |
| 时间稳定性 | 固定世界栅格/ROI 的高程标准差、漂移趋势 | 连续 GridMap、TF |
| 有效覆盖率 | 目标 ROI 中有效高程面积/目标面积；未知不能从分母默默剔除 | GridMap、ROI真值 |
| 坡角/台阶尺寸误差 | 与独立测量对比，边缘与内部区域分开 | elevation、几何真值 |
| 障碍 precision/recall/IoU | 先明确评的是“当前Δh规则标签”还是“物理不可通行标签”；二者分开统计 | 高度差图/离线重算、标注 |
| 更新与清除延迟 | 场景变化至有效地图/障碍变化的时间；记录事件时刻和定义 | 输入点云、地图、事件标记 |
| 地图新鲜度 | 唯一源时间戳更新率、最大更新间隔、过期持续时长 | 点云及GridMap header、bag时间 |
| 导航行为 | 成功/正确拒绝次数、任务时间、轨迹长度、足迹最小净空、停止延迟 | 任务结果、TF/odom、cmd、地形真值 |

bag 接收时间减消息 header 时间只能作为近似消息年龄，需要统一时钟；不能直接叫作算法计算耗时。若要精确计算点云融合延迟，应有输入输出关联及节点内计时。目前 raw 节点使用 `_last_t` 设置地图时间戳，因此还要区分重复发布和新观测，时间戳变化也不单独证明融合成功。

最小净空按机器人足迹到障碍边界计算，不能只用机器人中心点距离。`/cmd_vel` 是命令，不能当作真实速度；里程计速度字段也应确认有效，必要时由位姿推算或增加底盘反馈。

### 4.2 建议执行规模和验收方式

- 预实验：每个子工况先做一次，检查消息、坐标系、场景尺寸和指标能否计算。
- 正式实验：关键工况至少重复5次；作为工程初步比较，不宣称已经有充分统计功效。失败、人工干预和无效录包分别记录，不只保留成功样本。
- 固定地图分辨率、阈值、采样半径、速度档位、初始位置、地图冷启动策略及障碍摆放；比较参数时一次只改一个主要因素。
- 同一份原始输入可离线比较建图/分类参数；导航性能比较需要独立闭环试验，因为不同决策会改变后续运动和观测。
- 如需证明高程模块对导航的贡献，可比较基线、仅HeightJumpCritic、HeightObstacleLayer+Critic；明确各组 costmap、其他 critics 是否一致，否则只是整套方案比较。
- 验收阈值在正式采集前根据传感器噪声、栅格大小、机器人能力和任务需求填写：高度误差上限、有效覆盖下限、最大更新间隔、漏检容限、最小净空。没有实测依据时不编造统一的厘米级及成功率标准。

## 5. 现有录包内容审查

### 5.1 五种脚本的实际覆盖

| 脚本 | 已有主要内容 | 对本次实验的缺口 |
|---|---|---|
| `rosbag_recording/scripts/record_terrain_raw.sh` | registered点云、odom、TF | 无地图输出、分类和导航；可作为高程重算输入，前提是输入依赖完整 |
| `rosbag_recording/scripts/record_terrain_elevation.sh` | 上述输入+完整高程GridMap | 无高度差、costmap、任务/控制；基础建图最接近可用 |
| `rosbag_recording/scripts/record_nav_baseline.sh` | TF、odom/path、AMCL、控制、目标/路径/轨迹 | 无map、点云、高程、costmap、任务结果 |
| `nav_experiment_toolkit/scripts/record_nav_baseline.sh` | 导航信息+map+局部/全局costmap；尝试附加action状态 | 无点云、高程、高度差；action发现存在隐藏话题问题 |
| `nav_experiment_toolkit/scripts/record_nav_elevation.sh` | 导航信息+map+高程；尝试附加action状态 | 明确未录costmap，也没有点云和高度差；不能完整追溯绕障原因 |

### 5.2 三份已有 bag 的实际内容

已对三份目录运行 `ros2 bag info` 并与 `metadata.yaml` 对照：

| bag时间后缀（2026-09-19） | 所在目录 | 时长 | 总消息数 | `/map` | `/tf_static` |
|---|---|---:|---:|---:|---:|
| `17-09-58` | `rosbag_recording/bags/nav_baseline` | 47.603 s | 4222 | 未记录 | 3 |
| `17-31-49` | `nav_experiment_toolkit/bags/nav_baseline` | 69.297 s | 6780 | 1 | 3 |
| `18-08-50` | `nav_experiment_toolkit/bags/nav_baseline` | 39.561 s | 3777 | 1 | 3 |

三份均有 `/tf`、`/aft_mapped_to_init`、`/path`、`/amcl_pose`、两路速度命令、规划路径及 `/trajectories`。三份均没有：`/cloud_registered`、高程图、高度差图、局部/全局costmap、`/goal_pose`、action状态/反馈。

结论：脚本中列了话题，并不证明实际 bag 中有消息。已有 bag 不能补算从未记录的高程信息，也不能仅凭轨迹宣称任务成功。`/tf_static`有3条只说明有静态TF消息，尚未逐条验证所需TF链完整性。

### 5.3 当前分析工具的局限

`nav_experiment_toolkit/analysis/analyze_nav_bag.py` 的读取列表没有高程图、高度差、点云和action状态，因此当前工具不计算高程精度、覆盖率、更新稳定性或真实action成功率。

它的到达判断是位姿容差近似判断，默认0.30 m/0.35 rad；当前高度障碍导航配置 goal checker 是0.20 m/0.15 rad。分析时应使用对应运行参数，并将“几何到达”和“action成功”分别报告。缺少 `/goal_pose` 时尤其不能把脚本结果当作正式任务成功判据。

## 6. 建议录制话题清单

### 6.1 四类实验共同的核心数据

| 优先级 | 话题 | 用途与说明 |
|---|---|---|
| 必须 | `/cloud_registered` | 高程输入、覆盖和异常来源、离线重算；它是处理后的点云，不是原始激光数据 |
| 必须 | `/elevation_mapping_node/elevation_map` | 保存完整GridMap，核查实际layers含elevation/variance；不需要单独variance话题 |
| 必须 | `/tf`、`/tf_static` | 点云、地图、机器人坐标变换和姿态关联 |
| 必须 | `/aft_mapped_to_init` | 独立保留位姿/里程计消息，核对frame、姿态和速度字段 |
| 判定实验必须 | `/height_obstacle_layer/height_difference_map` | 当前局部高度层已配置发布，保留阈值判定前的连续Δh；纯建图模式未启动插件时不要求存在 |
| 建议 | `/rosout`、`/parameter_events` | 过期、TF、转换错误与参数变化；日志级别会影响可见内容，参数事件不能替代初始参数快照 |

### 6.2 绕障与闭环导航时必须补齐

| 话题 | 作用 |
|---|---|
| `/map` | 静态地图背景和全局规划环境 |
| `/local_costmap/costmap`、`/global_costmap/costmap` | 验证高度障碍是否进入代价地图、膨胀及规划输入 |
| `/command/cmd_twist`、`/cmd_vel` | 控制器侧和最终速度命令，观察停走/抖动 |
| `/goal_pose`、`/plan`、`/transformed_global_plan` | 目标、规划参考、控制器参考；确认实际任务入口是否发布goal_pose |
| `/navigate_to_pose/_action/status`、`/navigate_to_pose/_action/feedback` | 使用NavigateToPose时的状态、UUID关联及进度；反馈不等于完整结果 |
| `/follow_path/_action/status`、`/follow_path/_action/feedback` | 直接FollowPath任务时使用，应核对实际action名称 |
| `/path`、`/amcl_pose` | 辅助核对定位与轨迹来源，不能作为独立地面真值 |

当前两类costmap配置 `always_send_full_costmap: True`，录完整costmap可覆盖其发布快照；若实际参数改为增量发布，应同时录对应 `costmap_updates`。如果需要精确分析Nav2原始代价值，可额外录实际存在的 `costmap_raw`，避免仅依据显示转换后的OccupancyGrid判断原始数值。

手绘路径模式额外录 `/custom_path/raw_path`、`/custom_path/preview_path`、`/custom_path/status`，可选 `/custom_path/preview_marker`。`/goal_pose`和NavigateToPose状态不能替代这条任务链。

### 6.3 按研究目标选录

| 数据/候选话题 | 何时需要 | 当前确认程度 |
|---|---|---|
| 原始LiDAR、原始IMU | 要重跑定位、解释姿态突变/运动畸变时 | 本次未确认真实话题名，不能直接写死 `/imu` 或 `/livox/lidar` |
| `/scan` | 使用AMCL/2D障碍源并需要复现这部分计算时 | 导航配置指定scan_topic为`/scan`，实机发布仍需核对 |
| 底盘速度、关节/轮速、机身姿态、接触/运动模式 | 要评价坡楼梯真实执行、打滑或姿态稳定性时 | 名称按底盘接口确认；命令速度不能替代反馈 |
| `/diagnostics`及GPU/CPU/磁盘性能记录 | 评价实时性、负载和丢帧原因时 | 话题可能不存在；也可保存带时间戳的系统日志 |
| `/clock` | 仿真或以ROS仿真时间采集时 | 实机系统时间模式不要求；回放可由`--clock`生成 |
| `/trajectories` | 需要观察MPPI采样轨迹时 | 已录，但可视化有抽样，且无每条轨迹的完整拒绝原因/代价 |
| 试验事件标记、任务result/错误码 | 准确划分开始、移障、干预、结束及失败原因 | 当前没有统一记录器；先用与ROS时钟对齐的旁置记录，后续可新增专用话题 |

当前HeightJump详细计数是节流DEBUG日志，默认级别未必输出；录 `/rosout` 也不保证有每周期判定。若研究每条rollout为何被拒绝，需额外诊断输出，不能把 `/trajectories` 当作完整内部决策日志。

## 7. 录包和回放中要修正的具体问题

### 7.1 隐藏action话题的发现

现有工具脚本用普通 `ros2 topic list` 搜索 `/navigate_to_pose/_action/status`。本机Humble的CLI确认默认不列隐藏话题，应使用：

```bash
ros2 topic list --include-hidden-topics -t
ros2 bag record --include-hidden-topics --output BAG_DIRECTORY <本次明确选择的话题列表>
```

上面是命令格式，不是可以原样运行的完整录制命令。应把实际使用的action话题显式加入列表，并在短录包中检查非零消息。Humble本机 `ros2 bag record --help` 未提供完整action服务录制选项；不能假定录状态/反馈就保存了goal请求和result服务响应。正式任务结果建议由任务客户端保存UUID、目标、完成状态和错误码。

### 7.2 QoS、起止时机和消息完整性

录制前用 `ros2 topic info <topic> -v` 核查发布者QoS。重点是静态TF、静态地图及best-effort传感器话题；必要时用 `--qos-profile-overrides-path` 配置兼容订阅，并通过实际消息数确认效果。rosbag2官方说明支持录制/回放QoS覆盖，具体选项以本机Humble帮助为准：[rosbag2官方说明](https://github.com/ros2/rosbag2#readme)。

先启动录制，确认核心订阅后再发目标；开头留5–10 s，结束后留5–10 s观察静止、地图残留和状态。静止稳定性子实验另保留30–60 s。`/map`或`/tf_static`消息少不一定异常，但必须含全部需要的信息；动态话题要检查活跃区间内频率、最大间隔及唯一header时间戳。

录包前检查脚本目前偏向2D导航，缺高程/点云/高度差的完整性检查。未来应按“纯建图”“判定”“导航”三种任务选择必需话题，避免要求没有运行的导航模块发布消息。

### 7.3 两种回放目标必须分开

- 展示已有结果：播放录下的地图、高度差、costmap、轨迹，供RViz和离线评估读取。
- 重新计算高程：只将输入点云、必要TF及其他实际输入喂给建图节点，输出到独立话题/新bag，不能同时把录下的旧高程图播放到同名输出。

回放计算节点应统一 `use_sim_time=true` 并使用回放时钟。历史TF、地图初始化状态、参数和实际输入依赖缺失时，即使有点云也不能保证复现。若要从原始传感器重跑定位，则必须另录原始LiDAR/IMU与标定，registered点云不够。

### 7.4 数据量与实验记录

已有69.3 s导航bag约559.1 MiB，加入点云后需提前测实际写入带宽和剩余空间。`/trajectories`是可降级项，但本次未做逐话题字节统计，不能断定全部体积都来自它。先做60 s短录包压力检查，按实际带宽估算容量；比较录包前后更新间隔，防止录制造成实时性退化。

每个bag旁保存：实验编号/重复号、场景尺寸与测量不确定度、照片和ROI、启动命令、实际参数dump、软件提交号及未提交配置差异、TF/外参、QoS配置、速度、阈值/半径、地图重置方式、事件时间和人工干预原因。`/parameter_events`不包含可靠的完整初始参数快照。

## 8. 推荐最小落地顺序

1. 统一一次录制的核心输入输出：点云、高程（含variance）、TF、odom；启用阈值模块时加入高度差图，导航时加入两类costmap、任务状态和控制。
2. 先做60 s平地短录包，确认真实layers、frame、频率、时间戳、静态TF、数据量及离线可读性。
3. 完成平地静止/运动/重访，再完成阈值上下边界和移障清除；这些通过后再解释复杂坡面和楼梯结果。
4. 斜坡和楼梯先报告几何重建及规则判定，再单独报告物理通过或导航拒绝行为。
5. 每个工况按预先约定的指标重复采集，保存失败样本和参数；补充高程离线分析，现有导航分析脚本无法代替这一部分。

## 9. 证据索引与审查边界

- 录制脚本：`rosbag_recording/scripts/record_*.sh`、`nav_experiment_toolkit/scripts/record_nav_*.sh`。
- 实际bag：第5.2节三份目录，已运行`ros2 bag info`；未逐条反序列化消息验证坐标链和数据质量。
- 障碍判据：`src/height_obstacle_layer/include/height_obstacle_layer/height_difference.hpp`及`src/height_obstacle_layer/src/height_obstacle_layer.cpp`。
- MPPI判据和健康保护：`src/height_jump_mppi_critic/include/height_jump_mppi_critic/evaluator.hpp`、`src/height_jump_mppi_critic/src/height_jump_critic.cpp`。
- 导航参数：`src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml`；有其他入口，不代表实机必然使用此文件。
- 手绘路径话题：`src/custom_path_navigation/src/path_executor.cpp`。
- 分析能力：`nav_experiment_toolkit/analysis/analyze_nav_bag.py`。
- 高程源配置：`/home/nvidia/elevation_map/src/elevation_mapping_cupy/elevation_mapping_cupy/config/core/core_param.yaml`、`config/setups/raw/elevation.yaml`、`launch/raw_elevation.launch.py`及`elevation_mapping_cupy/elevation_mapping_node.py`。
- 高程工作区历史记录：`/home/nvidia/elevation_map/PROJECT_STATUS.md`、`EXPERIENCES.md`的raw和坐标系相关条目；历史实测不视为当前实机状态。
- 本机CLI：已核查`ros2 bag record --help`、`ros2 topic list --help`，确认Humble隐藏话题和QoS覆盖选项。

尚未确认：当前机器人实际运行的launch与参数、原始传感器/底盘反馈话题、完整实时TF链、真实传感器精度和机器人坡楼梯能力。因此本文提供可执行的实验与录制设计，不给出未经测量的精度或通过能力结论。
