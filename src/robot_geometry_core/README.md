# robot_geometry_core

`robot_geometry_core` 是不依赖 ROS、Nav2、Gazebo 的二维机器人碰撞几何核心。

## 责任边界

- `RobotCollisionEnvelope`：一次决策使用的不可变几何快照，包含完整 footprint、保守外接半径、frame、来源和版本。
- `IRobotGeometryProvider`：只描述“取得几何快照”，不关心几何来自仿真模型、配置文件还是实机标定。
- `StaticRobotGeometryProvider`：承载 YAML 或离线标定产物生成的固定几何。
- `checkFootprint()`：在纯 `GridMap` 上按目标位姿和 yaw 做精确多边形碰撞检查。

当前 ROS 节点通过 `exploration_nodes` 内的参数适配器创建静态 Provider。它不是 Gazebo API
提取器；仿真阶段使用配置好的车型轮廓，实机阶段可把靠墙标定结果写成同一份 polygon 配置，
或者新增实现 `IRobotGeometryProvider` 的标定产物 Provider，而不改 Frontier 和 Navigation。

## 两种消费方式

- Frontier 只取 `circumscribed_radius`，生成方向无关的外包圆进行保守粗筛。
- Navigation 取完整 `footprint`，结合导航目标 yaw 做精确落脚检查。

这样非圆机器人不会因为 Frontier 固定使用 `yaw=0` 而漏判，Navigation 也不会退化成过于保守的圆。

## 参数契约

圆形配置继续兼容原有 `robot_radius` 和 `footprint_padding` 参数。新增公共元数据：

```yaml
robot_geometry:
  shape: circle
  frame_id: base_link
  source: configured_simulation_profile
  revision: 1
```

实机标定多边形示例：

```yaml
robot_geometry:
  shape: polygon
  footprint: [-0.20, -0.10, 0.30, -0.10, 0.30, 0.10, -0.20, 0.10]
  frame_id: base_link
  source: wall_calibration
  revision: 2
footprint_padding: 0.0
```

多边形必须已经包含安全余量，点坐标必须位于声明的机体 frame 中，因此不允许再叠加
`footprint_padding`。少于三个点、重复点、零面积、自交、非有限数值或过小外接半径都会直接拒绝。
