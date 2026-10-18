# RViz 自由绘制 FollowPath

在 RViz 中自由画出一条二维路线，松开鼠标后查看预览，按 Enter 将整条处理后的路线交给 Nav2 `FollowPath`。最后一点就是终点；路径始终为 `z=0`，不经过全局规划器重新规划。本包使用标准 ROS2、TF、RViz 和 Nav2 接口，不读取高程数据。

## 构建与启动

在工作区根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select custom_path_navigation fishbot_navigation2
source install/setup.bash
./scripts/start_navigation_draw_path.sh
# 可选：config 目录中已存在的 Nav2 YAML basename
./scripts/start_navigation_draw_path.sh nav2_params_height_jump.yaml
./scripts/stop_navigation_draw_path.sh
```

脚本需要 `tmux` 和 `gnome-terminal`。默认参数为 `src/fishbot_navigation2/config/nav2_params_height_obstacle.yaml`；只接受该目录内已存在的 `.yaml` 文件名，不接受完整路径、目录或含 `..` 的名称。新会话名为 `mppi_nav_draw_path`，三个窗格分别运行导航（含 RViz/executor）、`keep_zero.py` 和 `initial_pose_tf.py`。重复启动先对旧会话各窗格发送 Ctrl-C 并关闭旧会话；关闭失败会中止新启动。停止脚本可重复运行。

也可以直接运行，分别覆盖 Nav2 和本包参数：

```bash
ros2 launch fishbot_navigation2 navigation_draw_path.launch.py \
  params_file:=/absolute/path/nav2.yaml \
  custom_path_params_file:=/absolute/path/custom_path.yaml \
  use_sim_time:=false
```

launch 还提供 `map` 参数，默认地图与现有导航相同。Nav2 的 `/cmd_vel` 保持重映射到 `/command/cmd_twist`。自定义工具与 executor 自身不发布速度或零速度；集成脚本保留的 `keep_zero.py` 按原有逻辑向 `/cmd_vel` 转发指令，并在指令中断时补发零速度。

## RViz 操作

新模板保留机器人方框、地图、costmap、规划路径和 MPPI 可视化，并加载 `Draw Follow Path` Tool 和两个预览显示。若在其他模板中使用，点击工具栏的 **Add Tool**，选择 `custom_path_navigation/DrawFollowPath`；随后点击 **Draw Follow Path** 选中它。原有 `2D Nav Goal` 仍可单独选择，只有当前选中的工具处理鼠标操作。

将 Fixed Frame 设为 `map`，把鼠标焦点放在 RViz 绘图区域：

| 操作 | 结果 |
| --- | --- |
| 按住左键拖动 | 在地面 `z=0` 上绘制连续曲线 |
| 松开左键 | 发布候选并显示平滑、等距重采样后的预览，不立即执行 |
| Enter | 验证最近候选并异步发送完整 FollowPath 路径 |
| Esc | 丢弃当前笔画、清除候选；有活动目标时请求取消 |
| Ctrl+Z | 恢复上一条候选，保留一层撤销历史；无历史时无变化 |

重新绘制会替换候选，但正在执行的目标继续运行；按 Enter 才会先取消旧目标，再发送新候选。取消失败或超时不会发送第二个目标。切换 Fixed Frame 会丢弃当前笔画，需要重新绘制。

绿色表示候选通过验证，红色表示无效或执行失败，蓝色表示正在发送/执行，灰色表示已成功、已取消或历史状态。状态颜色由 `/custom_path/preview_marker` 显示；`/custom_path/preview_path` 显示路线形状。具体状态与原因可查看：

```bash
ros2 topic echo /custom_path/status
```

## 执行条件与无实机验证

至少绘制两个有效点，总长至少 `0.20 m`。默认过滤间距 `0.02 m`，三点移动平均一次（首尾不移动），重采样间距 `0.05 m`。参数位于 [config/custom_path.yaml](config/custom_path.yaml)。路径 frame 必须与 executor 的 `fixed_frame` 一致，默认 `map`。

执行时必须有可用的 `map → base_link` TF，且路径起点距机器人不超过 `0.5 m`（含边界）；不会自动补画机器人到起点之间的路线。Nav2 controller server 必须已激活并提供 `/follow_path` Action，所选 Nav2 YAML 必须配置 `FollowPath` controller 与 `precise_goal_checker` goal checker，或在本包参数中设置对应 ID。

没有实机时仍可绘制、撤销和预览。缺少 TF 时预览为红色并拒绝执行；缺少 FollowPath Action 时执行请求失败且保留候选，不发送其他运动命令。可以用脚本保留的初始位姿 TF 或测试 TF 验证起点检查。`/custom_path/execute` 的成功响应表示已通过即时验证并允许异步发送，最终成功/失败/取消以 `/custom_path/status` 为准。Esc 通过 Action cancel 停止目标；后续停车由 Nav2 和现有控制链路处理。
