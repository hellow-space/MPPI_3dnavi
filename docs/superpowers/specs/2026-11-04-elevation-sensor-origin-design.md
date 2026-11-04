# 高程点云传感器原点修正

用户要求先提交当前工作再继续修改。检查点 `3c59a62` 已创建；下一阶段只修复实际传感器原点，暂不增加激进 ROI/高度过滤或改变雷达/FAST-LIVO。

外部仓库 `/home/nvidia/elevation_map` 创建 `fix/cloud-sensor-origin`，保留已有未提交配置/文档；原始工作文件与补丁已保存 `/tmp/elevation_sensor_origin_before`。本轮修改只提交自身增量。

选择在 ROS 适配层将注册点云重新表达为当前物理传感器坐标：由 map←cloud 与 map←sensor 计算 sensor←cloud，保持额外通道，随后调用既有 GPU API 的 R=map←sensor.R、t=当前传感器原点。点在 map 中的几何位置不变，但距离噪声、距离/高度门限和清理射线的起点得到修正。不改变 CUDA kernel/API；需量化额外 CPU 变换耗时。

每个点云 subscriber 可显式配置 `sensor_frame`。原始传感器坐标点云未配置时仍使用消息 frame，保持兼容。raw 真实入口默认物理 frame 为 livox_frame；合成输入固定使用 base_frame。提供 launch 参数 pointcloud_sensor_frame，其他硬件需正确配置。

TF 优先用消息时刻；沿用现有可使用最新 TF 的兼容场景，但对新点云路径设置有界年龄（默认 0.2 s）并明确告警，不在缺失/过旧时退回 map/camera_init 原点。点云接受成功后才推进数据时间；失败不让旧地图伪装成新观测。转换应无额外阻塞等待。

验证 raw/cloud/map 三种 frame、非单位传感器旋转、移动原点、额外通道、无效 XYZ、缺失/过旧 TF、实际 GPU 高程融合和独立 ROS 域的输出；运行包构建与对应测试。源码已绑定到 Docker，使用容器做编译/隔离验证，不启动实机雷达或导航。遵循外部 AGENTS，更新项目状态与经验并对本轮增量单独提交。
