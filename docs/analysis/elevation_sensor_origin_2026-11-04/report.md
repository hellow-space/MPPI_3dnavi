# 高程融合传感器原点修正交接

2026-11-04。用户要求先提交当前工作，再开始修改点云处理。

## 检查点与仓库

- 主仓库 `/home/nvidia/MPPI_3dnavi`，分支 `feat/location`：先提交 `3c59a62 fix: align manual map initialization to measured gravity`。提交前重新运行 Python 全套 52/52，通过 staged whitespace 检查。包含定位修正、回归测试及点云分析，没有修改雷达点率。
- 高程仓库 `/home/nvidia/elevation_map`，从 `4ea09a2` 新建 `fix/cloud-sensor-origin`，提交 `4190bb9 fix: use physical sensor origin for registered cloud fusion`。仓库通过 bind mount 映射到容器 `/root/elevation_map`，已在容器编译；现有启动入口下次启动即可使用。
- 创建4190bb9时，高程仓库已有的配置/文档用户改动被保留，未混入该提交。已将初始工作文件与补丁备份到 `/tmp/elevation_sensor_origin_before`，并验证新增/删除行的净差异没有改变。之后用户明确要求提交全部剩余内容，见下方量化对照和最终提交记录。

## 实现

节点显式区分注册点云所在的固定 frame 与物理传感器 frame。点云重新表达到当前传感器坐标，再复用既有 GPU API：地图中的点位不变，距离噪声、距离/高度门限及清理射线使用当前物理雷达原点。不改 CUDA、FAST-LIVO 或 MPPI，不新增激进高度/ROI 过滤。

raw 真实输入默认 `sensor_frame=livox_frame`，可用 `pointcloud_sensor_frame` launch 参数覆盖；测试生成器自动使用它的 base_frame。缺失或过旧 TF 丢弃该帧，不推进最近成功融合观测的时间。TF 非阻塞精确查询优先，latest fallback 年龄最多 0.2 s 并告警，零时间动态 TF 无法冒充静态 TF。

## 验证与限制

- 包构建成功。
- 18 项新功能测试通过，覆盖 callback、launch、真实 GPU 地图与原生传感器参考的高程/方差比较、距离及高度门限。
- 独立域144实际测试 PointCloud2/TF→GPU→GridMap，确认移动传感器原点但保持世界点不变，拒绝帧不刷新观测时间；没有速度发布者。
- 独立代码复核通过；100 组随机三维变换下保留额外属性，±100 m 范围最大 float32 世界点误差约 0.046 mm。
- 18000 点 CPU 适配 P50 1.336 ms、P95 1.373 ms，200 次测量；只包含新增 CPU 几何转换，不含 TF/DDS/GPU，也不能代表现场总延迟。
- 完整 colcon 已执行：15 组中6组失败，64 用例10 errors/10 failures，根因是默认神经模式导入 PyTorch 缺 `libcudss.so.0`，继而导致对应 mapper/map/service 不可用。独立 `import torch` 可复现，未安装或改动依赖栈。MCAP 插件也不存在，本轮保存测试尚未进入存储阶段。
- 既有 `ros2_numpy` 强度字段解码问题未修；raw XYZ 路径不使用该分支。当前物理 TF 的外参精度、已知 IMU/LiDAR 约5.5 cm 原点语义差异、原始输入时间问题及34°安装视场均未由此解决。
- 没有实机导航或新包对照，不能声称转圈、地形缺测或整体延迟已修复。

详细配置、测试日志和复现脚本：`/home/nvidia/elevation_map/docs/SENSOR_ORIGIN_FIX.md` 与 `/home/nvidia/elevation_map/docs/analysis/sensor_origin_2026-11-04/`。

## 剩余内容提交与量化对照

用户随后明确授权提交剩余改动。已检查既有 map frame、7 m地图大小、对应测试/文档，并将参数注释说明中的20 m同步为实际7 m。专项验证和新旧点云/GPU量化实验见 [comparison.md](comparison.md)。没有增加新的生产过滤功能，也没有实际启动 Nav2 或雷达。

高程剩余9个配置/测试/说明文件已提交为 `3b1ebbf fix: finalize gravity-aligned raw elevation configuration`；高程仓库工作区已清空。主工程本批交接/设计文档和新增对照证据将以 `docs: record elevation origin before-after comparison` 提交，不混入生产导航修改。
