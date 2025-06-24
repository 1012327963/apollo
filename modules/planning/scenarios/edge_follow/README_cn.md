# Edge Follow 场景

该场景用于测试车辆贴近车道右侧边缘行驶的能力。通过在 `EdgeFollowPath` 任务中生成靠右侧的规划轨迹，实现沿车道右侧行驶的效果。
触发贴边后，车辆会在短距离内从当前位置驶向路沿，随后保持车辆右侧贴近边缘，
且车身朝向始终与车道方向一致。

## 使用说明

1. 在 `planning.conf` 中关闭参考线线程并设置边界缓冲：

   ```
   --enable_reference_line_provider_thread=false
   --edge_follow_buffer=0.2
   ```

2. 调整参考线与终点虚拟墙配置，使其更适合贴边任务。

3. 在 `public_road_planner_config.pb.txt` 中将 `EDGE_FOLLOW` 场景放在 `LANE_FOLLOW` 之前，以便优先进入贴边场景：

   ```protobuf
   scenario {
       name: "EDGE_FOLLOW"
       type: "EdgeFollowScenario"
   }
   scenario {
       name: "LANE_FOLLOW"
       type: "LaneFollowScenario"
   }
   ```

4. 通过 `external_command_demo` 启动或结束贴边，可直接输入 `edge_start` 和 `edge_stop`，
   其本质是向 Planning 发送 `ENTER_MISSION` 与 `EXIT_MISSION` 的 PadMessage：

   ```bash
   mainboard -d modules/external_command/external_command_demo/dag/external_command_demo_wrapper.dag
edge_start    # 开启贴边
edge_stop     # 退出贴边
```

5. 修改 `planning_config.pb.txt`，将 `pnc_map_class` 设置为 `apollo::planning::EdgeFollowMap`，
   使规划参考线使用贴边地图。
