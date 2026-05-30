# chassis_bridge

`chassis_bridge` 提供底盘抽象 MVP：

- `fake_chassis_node`：订阅 `/cmd_vel`，按二维速度积分发布 `/odom`，可发布 `odom -> base_link` TF。
- `ChassisProtocol`：Jetson 与 ESP32 后续串口通信使用的二进制协议编解码工具。
- `chassis_bridge_node`：真实串口底盘桥接预留节点，当前不实现串口读写。

## Build

```bash
colcon build --packages-select chassis_bridge --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch chassis_bridge fake_chassis.launch.py
```

## Test

```bash
colcon test --packages-select chassis_bridge
colcon test-result --verbose
```
