# 旧导航配置归档

归档日期：2026-11-04。归档时仓库内未发现以下配置被启动脚本或 launch 引用，文件内容保持不变。

| 配置 | 原用途 |
| --- | --- |
| `before.yaml` | 局部代价地图使用 VoxelLayer 的旧导航配置 |
| `volex.yaml` | 局部代价地图使用 VoxelLayer 的另一套导航配置 |

日常导航继续使用上级目录中的 `2d_params.yaml`、`nav2_params_height_jump.yaml`、`nav2_params_height_obstacle.yaml`。`nav2_params.yaml` 仍是 `navigation2.launch.py` 的默认配置。

如需恢复，将对应 YAML 移回上级 `config/` 目录，再重新安装 `fishbot_navigation2` 包。归档目录会随现有 CMake 安装规则一同安装，但现用启动入口不会自动选择其中的 YAML。
