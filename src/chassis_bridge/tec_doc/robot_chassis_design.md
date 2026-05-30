# 机器人底盘抽象设计

## 职责

`fake_chassis_node` 负责在真实底盘未接入前提供稳定的 ROS2 底盘接口：

- 输入：`/cmd_vel`
- 输出：`/odom`
- TF：`odom -> base_link`

节点不负责真实串口通信、PID 控制、编码器读取、IMU 融合、SLAM 或 Nav2 路径规划。

## 参数

参数从 `config/fake_chassis.yaml` 加载，包括 frame、topic、控制频率、速度上限和 TF 发布开关。

## 运动模型

当前 MVP 使用二维平面速度积分：

- `x += vx * cos(theta + wz * dt / 2) * dt`
- `y += vx * sin(theta + wz * dt / 2) * dt`
- `theta = normalize(theta + wz * dt)`

速度指令会按配置的最大线速度和最大角速度限幅。
