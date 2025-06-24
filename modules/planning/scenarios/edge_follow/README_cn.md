# Edge Follow 场景

该场景用于测试车辆贴近车道右侧边缘行驶的能力。通过在 `EdgeFollowPath` 任务中生成靠右侧的规划轨迹，实现沿车道右侧行驶的效果。

## 使用说明

1. 在 `planning.conf` 中关闭参考线线程并设置边界缓冲：

   ```
   --enable_reference_line_provider_thread=false
   --edge_follow_buffer=0.2
   ```

2. 调整参考线与终点虚拟墙配置，使其更适合贴边任务。

3. 在 `planning_config.pb.txt` 中指定 `EdgeFollowMap`：

   ```
   reference_line_config {
     pnc_map_class: "apollo::planning::EdgeFollowMap"
   }
   ```

4. 通过 `external_command_demo` 启动或结束贴边，可直接输入 `edge_start` 和 `edge_stop`：

   ```bash
   mainboard -d modules/external_command/external_command_demo/dag/external_command_demo_wrapper.dag
   edge_start    # 开启贴边
   edge_stop     # 退出贴边
   ```
