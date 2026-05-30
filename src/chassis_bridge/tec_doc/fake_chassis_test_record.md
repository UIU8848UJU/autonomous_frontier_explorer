# Fake Chassis 测试记录

## 数据测试

已添加两组 gtest：

- `test_chassis_protocol`：使用多组速度指令和里程计反馈测试数据验证协议 encode/decode，并覆盖 CRC 损坏、payload 长度错误、payload 超限和心跳空 payload。
- `test_fake_chassis_math`：使用多组运动数据验证直线、原地旋转、弧线积分、连续积分、非正 dt 和角度归一化。

## 验证命令

```bash
colcon test --packages-select chassis_bridge
colcon test-result --verbose
```
