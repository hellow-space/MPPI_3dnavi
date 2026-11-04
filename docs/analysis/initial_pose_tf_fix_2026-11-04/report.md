# 手动定位重力基准修正

日期：2026-11-04。分支：`feat/location`。修改前 HEAD：`daf64a0`。

## 最终行为

`initial_pose_tf.py` 在接受 RViz 初始位姿时，用静止原始 IMU 比力经同一测量时刻 TF 得到 `camera_init` 中的向上方向，将其对齐地图 Z，再设置地图 x/y/yaw。机器人当时真实的 roll/pitch 不再被强行消去，后续 FAST-LIO 三维运动仍完整传递。

首次 base_link 高度默认 0 只定义高程参考，不是地面测量。再次点击只改变 x/y/yaw，保留水平和垂直基准。原始 IMU/TF 太旧、静止窗口不足、运动、非有限数据、错误 frame 等情况下拒绝新初始化，不退回旧的机身水平假设。

提供 `/initial_pose_tf/reset`，用于停止导航后清除旧标定、重新采集静止数据。ROS 时钟倒退会自动清除；没有时间跳变的 FAST-LIO 原点重置仍需显式 reset。重定位后的高程历史清理由操作流程管理，不自动调用底盘或地图服务。

## 文件

- `scripts/initial_pose_alignment.py`：无 ROS 的重力、刚体变换、静止窗口计算。
- `scripts/initial_pose_tf.py`：有界 IMU 缓冲、同步 TF、初始化/重定位、reset、错误诊断；保留 10 Hz 动态对齐 TF。
- `scripts/README_initial_pose.md`：启动、参数、操作要求与边界。
- `tests/test_initial_pose_alignment.py` 与 `tests/test_initial_pose_tf.py`：28 项专门测试。
- `tests/test_nav_recording_profiles.py`：全套测试发现此前用户要求的 2 倍速和回放 `/cmd_vel` 已生效，但旧测试仍要求 0.5 倍速且禁止 `/cmd_vel`；同步既有测试断言，本轮未修改回放脚本。

外部 FAST-LIVO 工作区、Nav2 参数、控制接口和滤波器均未改动，没有启动实机导航或发布运动指令，没有创建提交。

## 实际验证

1. 修改前回调测试出现 7 项断言失败，包括机身带俯仰导致地图倾斜、旧 TF、缺失 IMU、运动和错误 frame 被接受。初始测试夹具另有一个旧接口不支持的异常，单独记录在 `callback_red.log`，不计作行为验证。
2. 重力/相机倾斜、保留机身姿态、重定位保留 Z、输入无效等测试通过。
3. 增加连续性与诊断复现：大 IMU 空档和被丢弃旧 IMU 没有诊断分别先失败，随后修正；测试夹具曾重复插入同一时间 TF，修正夹具后再验证，不把夹具错误当产品问题。
4. 独立代码审查发现：最新 TF 之后已开始运动会被忽略；新无效 IMU 到来后旧静止窗口仍可使用。两个回归用例先失败，再修正为检查最新原始窗口，且无效输入清空候选窗口。修正后复核无剩余 critical/important 问题。
5. `python3 -m unittest discover -s tests -p 'test_*.py'`：**52 项通过**，包括 28 项定位专门测试。
6. `smoke.py` 在 **ROS_DOMAIN_ID=143、ROS_LOCALHOST_ONLY=1、CycloneDDS** 下实际运行两个测试节点，发布测试 IMU/TF/initialpose，确认完整 ROS 传输与 TF 输出、机身俯仰从 10° 到 15° 后地图不重定义、reset 生效、原始 IMU 过旧时拒绝以及健康数据恢复后可重新定位。没有速度发布者。
7. Python 语法编译和 `git diff --check` 通过。

日志中的 loopback 无 multicast 提示来自隔离测试域的 CycloneDDS 配置，未导致测试失败。合成输入只验证数学和通信链路，不是现场精度或性能证据。

## 使用变化与限制

无需编译 Python，重启原有导航脚本后生效。机器人静止约 1 秒再点击定位，并检查 `Initial pose accepted`。如果原始 IMU 时间仍像旧日志中那样落后数十秒，将明确拒绝初始化；需要先处理输入时间，不能通过反复点击来补偿。

至少一个健康静止窗口和正确 IMU 旋转外参是必要条件。静止比力无法独立区分恒定线性加速；TF header 也未必反映内部扫描年龄。既有原始 IMU 时间异常、odom twist 全零、10 Hz 固定动态边和模式切换前提前发送导航目标的问题，不属于此次全部修复的内容。

旧包没有原始 IMU，也没有控制模式切换记录，无法直接用它重跑本次标定。现场是否改善地图倾斜或转圈，需要按正确交接顺序重新测试并记录原始 IMU、TF 和模式切换。

复现命令和参数详见 `scripts/README_initial_pose.md`。`verification.json` 保存本次源码摘要，日志用于复核；本报告没有声称实机转圈已解决。
