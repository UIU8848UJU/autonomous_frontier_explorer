# PC、Jetson 与嵌入式部署边界

> 状态：持续路线图
>
> 同步基线：`d7be6c6`（2026-09-24）
>
> 说明：这是目标部署边界，不表示所有训练、同步和多机器人能力已经实现。

## PC — 后台 / 训练中心

负责**非实时、高算力、全局性的事情**：

* 收集 Jetson 上传的运行数据
* 保存地图、轨迹、决策结果、传感器摘要
* 构建 Dataset
* Reward / Label 计算
* 模型训练、离线评估、版本管理
* Gazebo / 多地图批量仿真训练
* 将验证通过的模型下发到 Jetson
* 任务管理：开始探索、停止任务、保存地图
* 日志、监控、Dashboard、RViz 等可视化
* 后续可以负责多机器人统一管理

一句话：

> **PC 决定“以后怎样做得更好”，以及管理机器人。**

---

## Jetson — 机器人本机决策中心

负责**机器人当前应该做什么**：

* ROS2 主系统运行
* SLAM / Mapping
* Localization / TF / Robot State
* Frontier Detection
* Exploration Strategy
* 加载 PC 训练好的 ML / RL 模型并推理
* 选择下一探索目标
* Nav2 全局路径规划
* Local Planner / 动态障碍避障
* Recovery
* 将最终运动指令 `cmd_vel` 发给嵌入式
* 采集运行数据并上传 PC
* 与 PC 断开后仍能独立完成当前任务

一句话：

> **Jetson 决定“现在去哪、怎么过去”。**

这里可以进一步拆成：

```text
SLAM / State
      ↓
Exploration Policy
      ↓
Goal
      ↓
Navigation
      ↓
cmd_vel
```

---

## 嵌入式 — 实时控制 / 安全执行层

负责**机器人具体怎么动，而且必须安全可靠地动**：

* 编码器采集
* IMU / 电流 / 电压 / 碰撞等传感器采集
* 接收 Jetson 的目标速度
* `cmd_vel → 左右轮目标速度`
* PID / FOC 等电机闭环
* PWM 输出
* 里程计基础数据
* 通信协议：UART / CAN / Ethernet
* Watchdog
* 通信超时自动停车
* 电流过载保护
* 电机异常保护
* 急停
* 速度 / 加速度物理限制
* Jetson 崩溃时保证机器人不会继续乱跑

一句话：

> **嵌入式决定“电机具体怎么执行，以及绝不能怎么执行”。**

---

这样整个系统就是一个很清晰的三级控制体系：

```text
PC
训练 / 管理 / 分析
“以后怎么更聪明”
        ↓ Model / Mission

Jetson
感知 / SLAM / 决策 / 导航
“现在去哪、怎么过去”
        ↓ cmd_vel

STM32 / ESP32
实时控制 / 电机 / 安全
“轮子具体怎么转”
        ↓

      Robot

        ↑
Telemetry / Sensor / Outcome
        ↑
STM32 → Jetson → PC
```

而且还有一个很重要的**故障边界**：

```text
PC 挂了
→ Jetson + STM32 仍然可以继续当前任务

Jetson 挂了
→ STM32 watchdog 停车

STM32 检测到危险
→ 可以无条件拒绝 Jetson 指令
```

这个设计就已经非常像真正机器人产品的系统架构了。

你这个 Plan 我会最终概括成：

> **PC = Learning & Management Plane**
> **Jetson = Intelligence & Decision Plane**
> **Embedded = Real-time Control & Safety Plane**

后面你重构现在这个 `autonomous_frontier_explorer`，其实就可以直接拿这三层作为最高层架构边界。
