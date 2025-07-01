# EdgeFollowMap

`EdgeFollowMap` 是为贴边清扫任务准备的参考线地图插件，内部基于
`LaneFollowMap` 实现。在生成路段时会根据车辆速度使用
`edge_follow_look_forward_*` 与 `edge_follow_look_backward_distance`
等参数计算搜索距离，并对得到的 `RouteSegments` 做有效性裁剪。
后续可在此基础上扩展贴边场景特有的参考线逻辑。
