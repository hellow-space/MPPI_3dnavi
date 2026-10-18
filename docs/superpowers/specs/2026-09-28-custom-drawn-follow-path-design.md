# RViz 自由绘制 FollowPath 设计

## 1. 背景与目标

当前系统使用 Nav2 MPPI 生成速度指令，并由强化学习控制器执行底层运动。用户需要在没有实机接入时先验证一种可重复使用的自定义路线输入方式：在 RViz 中按住鼠标自由绘制一条完整二维曲线，预览并确认后，把这条曲线直接交给 Nav2 `FollowPath` 执行。路径最后一点就是终点。

该功能必须与现有全局规划、高程图、`HeightObstacleLayer`、`HeightJumpCritic` 和强化学习控制器解耦。高程障碍链路仍可独立影响 MPPI 和 costmap，但自定义路径模块不订阅或解析任何高程数据。

## 2. 范围

### 2.1 本次实现

- RViz 工具栏中的独立 `Draw Follow Path` 工具。
- 鼠标自由绘制完整路径。
- 松开鼠标后预览，不立即执行。
- `Enter` 确认执行。
- `Esc` 清除候选路径或取消正在执行的目标。
- `Ctrl+Z` 恢复上一条候选路径。
- 独立执行节点处理路径并调用 Nav2 `FollowPath` Action。
- 新的独立 RViz 模板、launch 文件和启停脚本。
- 新脚本允许选择 Nav2 参数 YAML，并默认使用 `nav2_params_height_obstacle.yaml`。

### 2.2 明确不做

- 不读取高程图，不计算或填充路径高度。
- 不订阅 `grid_map_msgs/GridMap`，不依赖 GridMap 软件包。
- 不调用 Nav2 全局规划器，不根据终点重新规划。
- 不把绘制路径转换成途经点任务。
- 不支持拖动局部控制点编辑路径。
- 不修改现有 HeightObstacleLayer、HeightJumpCritic 或强化学习控制器。
- 不直接发布速度或零速度命令。
- 不修改现有导航启动脚本、现有 RViz 模板或现有 Nav2 YAML。

## 3. 总体架构

新增通用 ROS2 包 `custom_path_navigation`，包含两个运行组件：

```text
RViz DrawFollowPathTool
  ├─ 捕获鼠标轨迹
  ├─ 保存一层撤销历史
  ├─ 发送确认/取消请求
  └─ 发布原始候选路径
             │
             │ nav_msgs/Path + std_srvs/Trigger
             ▼
custom_path_executor
  ├─ 过滤、平滑、等距重采样
  ├─ 检查 frame、TF 和起点距离
  ├─ 发布处理后预览与状态
  └─ 调用 nav2_msgs/action/FollowPath
             │
             ▼
          Nav2 MPPI
             │
             ▼
   /command/cmd_twist → 强化学习控制器
```

核心包仅通过标准 ROS2、TF、RViz 和 Nav2 接口工作。它不包含项目高程实现的头文件、参数或话题名称。

项目侧新增独立集成文件：

```text
src/fishbot_navigation2/launch/navigation_draw_path.launch.py
src/fishbot_navigation2/rviz/navigation_draw_path.rviz
scripts/start_navigation_draw_path.sh
scripts/stop_navigation_draw_path.sh
```

新的集成 launch 启动与当前导航等价的 Nav2、机器人方框、RViz，以及 `custom_path_executor`，并保留 `/cmd_vel` 到 `/command/cmd_twist` 的重映射。它使用新 RViz 模板，不修改原有 launch 或 RViz 文件。

## 4. ROS 接口

### 4.1 话题

| 名称 | 类型 | 发布者 | 用途 |
|---|---|---|---|
| `/custom_path/raw_path` | `nav_msgs/msg/Path` | RViz Tool | 原始鼠标候选路径 |
| `/custom_path/preview_path` | `nav_msgs/msg/Path` | executor | 平滑和重采样后的二维路径 |
| `/custom_path/preview_marker` | `visualization_msgs/msg/Marker` | executor | 按状态着色的路径预览 |
| `/custom_path/status` | `std_msgs/msg/String` | executor | 当前状态和简短失败原因 |

路径全部使用 `z=0`。`raw_path` 和 `preview_path` 使用 RViz Fixed Frame；默认且推荐为 `map`。executor 只接受与参数 `fixed_frame` 一致的路径，frame 不一致时拒绝，不做隐式转换。

### 4.2 服务

| 名称 | 类型 | 语义 |
|---|---|---|
| `/custom_path/execute` | `std_srvs/srv/Trigger` | 验证并执行最近一条候选路径 |
| `/custom_path/cancel` | `std_srvs/srv/Trigger` | 取消活动目标并清除候选路径 |

服务响应明确返回成功与失败原因。`Enter` 和 `Esc` 由 RViz Tool 转换成这两个服务请求。

### 4.3 Nav2 Action

executor 是 `nav2_msgs/action/FollowPath` 客户端。默认 Action 名称为 `/follow_path`，默认 `controller_id` 为 `FollowPath`，默认 `goal_checker_id` 为 `precise_goal_checker`，三者均为参数。

## 5. RViz 交互

RViz 新模板默认加载 `Draw Follow Path` 工具、处理后路径显示和状态 Marker。原版 `2D Nav Goal` 工具仍然存在，但只有被用户选中时才工作；自定义工具不发布 `NavigateToPose` 目标，因此两者不会同时响应一次鼠标操作。

交互规则：

1. 用户选择 `Draw Follow Path`。
2. 左键按下进入 `DRAWING`，按住并移动形成路径。
3. 松开左键发布候选路径，进入 `READY`，不执行运动。
4. 再次绘制会替换当前候选路径，并把旧候选保留为一层撤销历史。
5. `Ctrl+Z` 恢复上一条候选路径；没有历史时不改变状态。
6. `Enter` 调用执行服务。
7. `Esc` 在绘制过程中取消当前笔画；在 READY 时清除候选；在 ACTIVE 时取消 Action 并清除候选。

路径颜色：

- 绿色：候选路径通过静态验证，可以执行。
- 红色：候选路径无效，并在状态中说明原因。
- 蓝色：路径已发送并处于执行中。
- 灰色：成功、取消或历史预览状态。

## 6. 路径处理

处理流水线保持确定性，避免高阶样条产生用户没有画过的过冲：

1. 拒绝非有限坐标。
2. 删除与上一保留点距离小于 `min_raw_spacing` 的点，默认 `0.02 m`。
3. 至少需要两个有效点，且总路径长度不得小于 `min_path_length`，默认 `0.20 m`。
4. 对中间点执行可配置次数的三点移动平均，默认一次；起点和终点始终不移动。
5. 按弧长以 `sample_spacing` 等距重采样，默认 `0.05 m`。
6. 中间点 yaw 使用相邻重采样点的切线方向。
7. 最后一点 yaw 使用最后一段曲线方向。
8. 所有 Pose 的 `z` 固定为 `0`。

executor 保存原始候选和处理后候选。`Enter` 只会执行最近一次处理并验证成功的路径。

## 7. 起点与执行规则

执行时重新查询 `fixed_frame → robot_frame` TF，默认 `map → base_link`。绘制路径第一个点与机器人当前位置的二维距离必须不大于 `start_tolerance`，默认 `0.5 m`。

该限制不自动添加机器人到绘制起点之间的隐藏路径。超出范围时执行服务返回失败，候选路径保留为红色，用户可以重新绘制。

路径最后一点直接作为 FollowPath 的终点。该模块不会向全局规划器发送终点，也不会替换用户画出的曲线。

## 8. 状态机与并发目标

executor 使用以下状态：

```text
IDLE → DRAWING → READY → SENDING → ACTIVE
                         ACTIVE → SUCCEEDED
                         ACTIVE → FAILED
                         ACTIVE → CANCELED
```

`DRAWING` 主要由 Tool 管理；executor 从收到候选路径开始管理 READY 及后续状态。

当已有活动目标时收到新的 `Enter`：

1. 请求取消旧 FollowPath 目标。
2. 在 `cancel_timeout` 内等待服务器确认。
3. 取消确认后发送新路径。
4. 如果取消失败或超时，不发送第二个目标，并报告失败。

仅绘制或预览新路径不会打断当前活动目标，只有按下 `Enter` 才会触发替换。

## 9. 参数

新包使用自己的 `config/custom_path.yaml`，不向 Nav2 YAML 增加参数：

```yaml
custom_path_executor:
  ros__parameters:
    fixed_frame: map
    robot_frame: base_link
    start_tolerance: 0.5
    min_raw_spacing: 0.02
    min_path_length: 0.20
    sample_spacing: 0.05
    smoothing_passes: 1
    follow_path_action: /follow_path
    controller_id: FollowPath
    goal_checker_id: precise_goal_checker
    action_server_timeout: 2.0
    cancel_timeout: 2.0
```

参数在启动时验证。距离、时间必须为有限正数，平滑次数必须为非负整数。

## 10. 启动方式

新增脚本的接口：

```bash
./scripts/start_navigation_draw_path.sh
./scripts/start_navigation_draw_path.sh nav2_params_height_obstacle.yaml
./scripts/start_navigation_draw_path.sh another_nav2_params.yaml
./scripts/stop_navigation_draw_path.sh
```

未提供参数时使用 `nav2_params_height_obstacle.yaml`。只接受 `src/fishbot_navigation2/config/` 中存在的常规 YAML 文件名，拒绝不存在的文件和目录穿越路径。

新脚本使用独立 tmux session `mppi_nav_draw_path`。它保留当前导航启动中的 `keep_zero.py` 和 `initial_pose_tf.py` 窗格，但调用新的 `navigation_draw_path.launch.py`。旧会话和旧脚本不受影响。

## 11. 无实机行为

- RViz 绘制、撤销、过滤、平滑和预览不要求 FollowPath Action 存在。
- 没有 TF 时可以绘制，但路径显示红色，执行服务拒绝。
- 没有 `/follow_path` Action Server 时，执行服务返回失败，保持 READY，不发送其他命令。
- 可以通过现有初始位姿 TF 或测试 TF 验证起点检查。
- 自动测试使用假的 FollowPath Action Server，不连接强化学习控制器，不发布速度命令。

## 12. 错误处理

- 无效路径：保留候选并发布明确原因。
- TF 不可用或过期：拒绝执行，不猜测机器人位置。
- Action Server 不可用：拒绝执行并保持候选。
- Action Goal 被拒绝：状态变为 FAILED，保留路径供重画参考。
- 执行期间 executor 或 RViz退出：不额外发布速度；Nav2 现有 lifecycle、progress checker 和控制链路负责后续停车行为。
- `Esc` 通过 Action cancel 请求停止目标，不绕过 Nav2 直接发送零速度。

## 13. 测试策略

### 13.1 无 ROS 算法测试

- 短距离重复点过滤。
- 最短路径长度。
- 三点平滑保持首尾不变。
- 等距重采样间距。
- 直线、折线和曲线 yaw。
- 最后一点位置和朝向。
- NaN、Inf 和退化路径拒绝。

### 13.2 ROS 节点测试

- frame 不一致拒绝。
- TF 缺失拒绝。
- 起点距离边界 `0.5 m` 接受，超过边界拒绝。
- 假 FollowPath Server 接收处理后路径。
- 最后 Pose 与预览最后 Pose 完全一致。
- Action 拒绝、失败、成功和取消状态。
- 新目标先取消旧目标，取消失败时不发送新目标。

### 13.3 RViz 与脚本测试

- 插件描述可被 pluginlib 加载。
- 键盘状态转换：Enter、Esc、Ctrl+Z。
- 新 RViz 模板包含 Tool、Path 和 Marker 显示。
- 启动脚本默认 YAML、指定 YAML、不存在文件及重复启动行为。
- 旧启动脚本和旧配置回归测试继续通过。

## 14. 验收标准

1. 用户能在新 RViz 模板中自由绘制曲线路径。
2. 松开鼠标不会执行运动。
3. `Enter` 只发送验证成功的完整 FollowPath 路径。
4. 路径最后一点是 Nav2 收到的最后 Pose 和终点。
5. 起点超过机器人 `0.5 m` 时不能执行。
6. `Ctrl+Z` 能恢复上一条候选，`Esc` 能清除或取消。
7. 新路径替换活动路径时不存在两个并行 FollowPath 目标。
8. 没有实机、TF 或 Action Server 时不会发布速度或产生隐式运动命令。
9. 新包不依赖 GridMap、高程图或自定义高程插件。
10. 现有导航脚本、RViz模板、Nav2 YAML、HeightObstacleLayer 和 HeightJumpCritic 均不修改。

