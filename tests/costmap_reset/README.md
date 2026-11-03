# Jump 局部地图清图回归测试

读取实际 `nav2_params_height_jump.yaml`，通过本机 Nav2 初始化局部地图占位层，检查静止状态下反复清图和滚动地图后的就绪状态、空白地图的所有格子仍为 FREE_SPACE。

```bash
source /opt/ros/humble/setup.bash
cmake -S tests/costmap_reset -B /tmp/jump_costmap_reset_build
cmake --build /tmp/jump_costmap_reset_build -j2
ctest --test-dir /tmp/jump_costmap_reset_build --output-on-failure
```

CTest 设置独立的 `ROS_DOMAIN_ID=87`、`ROS_LOCALHOST_ONLY=1` 和 `/tmp` 下的日志目录；不启动导航、不发布速度、不连接运行中的 Domain 42 导航。

该测试覆盖此前错误的假设：启用但没有障碍源的 InflationLayer 可以在 reset 后自行重新就绪。原配置实测失败于 `Jump costmap never becomes ready after clear/reset at a stationary pose`；将纯 Jump 局部地图的占位膨胀层设为 `enabled: false` 后通过。全局地图膨胀层、HeightJumpCritic 和 HeightObstacle 配置不受本修复影响。
