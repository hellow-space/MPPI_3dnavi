# 旧导航入口归档

这些脚本保留用于查阅和恢复，不再作为日常导航入口。归档只移动文件，未修改脚本内容或执行权限。

| 脚本 | 原用途 | 当前推荐入口 |
| --- | --- | --- |
| `start_navigation.sh` | 启动使用 `2d_params.yaml` 的二维导航和 `keep_zero`，不启动 TF 辅助节点 | `../start_navigation_with_tf.sh` |
| `stop_navigation.sh` | 停止旧 `mppi_nav` 会话 | 新入口配套使用 `../stop_navigation_with_tf.sh` |
| `lalla.sh` | 分别打开导航和 `keep_zero` 窗口，导航使用默认 `nav2_params.yaml` | `../start_navigation_with_tf.sh` |

上述替代入口还会启动 TF 辅助节点；如果外部已提供相同 TF，需先核对后再使用。

HeightJump 和 HeightObstacle 继续使用上级目录内各自的 start/stop 脚本。

如需恢复旧入口，将相应文件移回 `scripts/`。历史分析文档中的旧路径保留作为当时的记录。
