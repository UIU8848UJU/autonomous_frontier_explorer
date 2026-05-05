# mk_nav2

<p align="center">
  <a href="https://docs.ros.org/en/humble/">
    <img alt="ROS 2 Humble" src="https://img.shields.io/badge/ROS%202-Humble-blueviolet">
  </a>
  <a href="https://releases.ubuntu.com/22.04/">
    <img alt="Ubuntu 22.04" src="https://img.shields.io/badge/Ubuntu-22.04-orange">
  </a>
  <a href="https://en.cppreference.com/w/cpp/17">
    <img alt="C++17" src="https://img.shields.io/badge/C%2B%2B-17-blue">
  </a>
  <a href="https://docs.nav2.org/">
    <img alt="Nav2 Exploration" src="https://img.shields.io/badge/Nav2-Exploration-2ea44f">
  </a>
  <a href="https://github.com/SteveMacenski/slam_toolbox">
    <img alt="SLAM Toolbox" src="https://img.shields.io/badge/SLAM-Toolbox-0f766e">
  </a>
  <a href="https://gazebosim.org/docs">
    <img alt="Gazebo Simulation" src="https://img.shields.io/badge/Gazebo-Simulation-brown">
  </a>
</p>

<p align="center">
  <a href="https://colcon.readthedocs.io/en/released/">
    <img alt="colcon" src="https://img.shields.io/badge/build-colcon-informational">
  </a>
  <a href="https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html">
    <img alt="ament_cmake" src="https://img.shields.io/badge/build%20system-ament__cmake-lightgrey">
  </a>
  <a href="CHANGELOG.rst">
    <img alt="Status" src="https://img.shields.io/badge/status-active%20development-yellow">
  </a>
  <a href="src/frontier_explorer/doc/frontier_explorer_node_doc.md">
    <img alt="Frontier Explorer Docs" src="https://img.shields.io/badge/docs-frontier_explorer-0ea5e9">
  </a>
  <a href="src/frontier_explorer/doc/exploration_architecture.md">
    <img alt="Exploration Architecture" src="https://img.shields.io/badge/architecture-BT--ready%20exploration-blue">
  </a>
</p>

面向自主探索、在线建图和导航验证的一体化 ROS 2 Workspace。当前阶段已经完成了基于 2D frontier 的自主探索闭环：Gazebo 仿真、SLAM Toolbox 建图、Nav2 路径规划与控制、frontier 能力节点、BehaviorTree.CPP 探索编排、NavigationNode 导航中间层、TaskManager 任务入口可以通过 bringup 一起运行。

当前演示使用 **RPP（Regulated Pure Pursuit）控制器**。DWB 在本仓库当前场景下更容易出现抖动、原地调整或短暂停滞，依赖决策层兜底恢复；因此阶段性演示主要采用 RPP，并展示不同探索策略风格下的效果。

## 当前阶段结论

当前 2D frontier exploration 版本已经阶段性收口。它可以作为一个完整的规则探索 baseline：规则负责安全边界，BT 负责执行、恢复和失败重选，NavigationNode 负责落脚和路径可执行性检查。

近期调试也暴露出一个明确问题：继续在 2D frontier 主策略中叠加规则，会逐渐进入“策略链地狱”。候选退避、环形采样、unknown ratio、goal inset、footprint、path safety、retry / blacklist 都有价值，但组合维护成本会快速升高。后续不再继续深挖规则策略调参，而是优先优化代码架构和决策数据闭环。

下一阶段方向暂定为：

```text
规则负责安全
ML 负责排序
BT 负责执行恢复
```

计划先记录每次探索决策数据，再训练 success / gain ranker，用机器学习排序替代继续堆叠人工规则。BT 仍作为执行和恢复骨架，规则层继续保证碰撞、unknown、可执行性等硬安全边界。

## 演示

> 两段视频均为 6 倍速。

## 🎬 Demo Preview

### 第一版收口

https://github.com/user-attachments/assets/cac2c7f5-62c3-438f-a46b-0e572ae47c14

### 激进探索demo（旧版本）

https://github.com/user-attachments/assets/9e4f8a30-9b6f-4366-87e5-293d407cfe1d

## 当前能力

- 一键 bringup：仿真、SLAM、Nav2、RViz、FrontierExplorer、TaskManager 分阶段启动。
- 自主探索：基于 OccupancyGrid 检测 frontier，通过 BT 请求候选、选择可执行目标并触发 NavigationNode 导航。
- 架构分层：`FrontierExplorerNode` 只负责 frontier 能力，`ExplorationBtOrchestratorNode` 负责流程，`NavigationNode` 负责 Nav2 桥接和可执行性检查。
- 决策分层：frontier 决策拆成 detector、pruner、scorer、selector，BT 插件负责流程节点组合。
- 权重策略：通过 YAML 权重组合表达探索风格，当前作为规则 baseline 保留。
- 小边界兜底：小 frontier 不会被直接删除，但会延后到正常候选耗尽后再选择。
- 可执行性检查：NavigationNode 组合 footprint 落脚检查、Nav2 `ComputePathToPose` 和 path safety 检查。
- 状态发布：frontier 能力、BT 编排和 TaskManager 分别发布状态，旧 `/exploration_state` 保留兼容。

探索节点这里只做概览。详细设计、状态机、参数和调试说明见：

<p>
  <a href="src/frontier_explorer/README.md">
    <img alt="Open Frontier Explorer README" src="https://img.shields.io/badge/open-frontier_explorer_README-blue">
  </a>
  <a href="src/frontier_explorer/doc/frontier_explorer_node_doc.md">
    <img alt="Open Frontier Explorer Docs" src="https://img.shields.io/badge/open-frontier_explorer_doc-0ea5e9">
  </a>
  <a href="src/frontier_explorer/doc/exploration_architecture.md">
    <img alt="Open Exploration Architecture" src="https://img.shields.io/badge/open-exploration_architecture-2563eb">
  </a>
</p>

## 仓库结构

```text
mk_nav2/
├── README.md
├── CHANGELOG.rst
├── media/                              # 阶段性演示视频
├── maps/                               # 静态地图输入/输出目录
├── src/
│   ├── autonomousr_explorer_bringup/    # 统一 launch/config/rviz
│   ├── frontier_explorer/               # frontier 探索节点与决策模块
│   ├── task_manager/                    # 高层任务编排
│   ├── robot_interfaces/                # 自定义消息/服务
│   └── util_package/                    # 日志等公共工具
├── build/
├── install/
└── log/
```

## 模块概览

### autonomousr_explorer_bringup

集中管理系统启动入口和运行参数：

- `full_system.launch.py`：根据地图文件是否存在，自动选择 SLAM 探索或静态地图模式。
- `full_system_slam.launch.py`：在线 SLAM + Nav2 + RViz + FrontierExplorer + TaskManager。
- `full_system_static.launch.py`：静态地图定位 + Nav2 + RViz + FrontierExplorer + TaskManager。
- `config/nav2_exploration.yaml`：探索模式 Nav2 参数，当前 FollowPath 使用 RPP。
- `config/frontier_explorer.yaml`：frontier 决策权重与候选过滤参数。

### frontier_explorer

负责从 `/map` 中寻找 frontier，提供候选生成、失败标记、blacklist、marker 和 state。当前探索链路是：

```text
TaskManagerNode
  -> ExplorationBtOrchestratorNode
  -> BehaviorTree.CPP XML + BT plugins
  -> FrontierExplorerNode / FrontierGoalProvider
  -> NavigationNode
  -> Nav2
```

`FrontierExplorerNode` 不再直接发送 Nav2 goal。BT 负责什么时候请求候选、什么时候导航、失败后什么时候标记失败和重新选点。`NavigationNode` 对外提供 `/navigation_node/navigate_to_pose`，内部桥接 Nav2 `NavigateToPose`。

### task_manager

提供高层任务入口，负责把建图、探索、导航状态串起来：

- `/start_mapping`
- `/start_navigation`
- `/stop_all`
- `/task_manager_state`

## 快速开始

### 依赖

目标环境：

- Ubuntu 22.04
- ROS 2 Humble
- Nav2
- SLAM Toolbox
- TurtleBot3 Gazebo
- colcon

仓库内依赖：

- `robot_interfaces`
- `util_package`

### 构建

```bash
cd ~/mk_nav2
colcon build
source install/setup.bash
```

### 启动完整系统

```bash
ros2 launch autonomousr_explorer_bringup full_system.launch.py
```

`full_system.launch.py` 会先启动 Gazebo，然后根据 `explore_map.yaml` 指向的地图文件是否存在选择：

- 有地图：静态地图导航模式
- 无地图：SLAM 探索模式

### 启动探索

```bash
ros2 service call /start_mapping std_srvs/srv/Trigger {}
```

TaskManager 会进入建图流程，并触发 `ExplorationBtOrchestratorNode` 开始探索 BT。

### 常用控制

```bash
# 直接启动探索 BT
ros2 service call /exploration_bt_orchestrator_node/start_exploration std_srvs/srv/Trigger {}

# 停止探索 BT
ros2 service call /exploration_bt_orchestrator_node/stop_exploration std_srvs/srv/Trigger {}

# 停止任务管理器中的当前任务
ros2 service call /stop_all std_srvs/srv/Trigger {}
```

### 常用状态话题

```bash
ros2 topic echo /exploration_state
ros2 topic echo /task_manager_state
ros2 topic echo /behavior_tree_log
```

## 策略配置

当前 frontier 决策通过 YAML 表达规则 baseline：

```yaml
frontier_decision:
  weight_distance: 1.4
  weight_cluster_size: 0.35
  weight_unknown_risk_penalty: 2.0
  candidate_unknown_margin_cells: 2
  candidate_goal_inset_cells: 3
  candidate_max_unknown_ratio: 0.25
  defer_small_clusters: true
  small_cluster_size_threshold: 5
```

调参方向：

- 当前 2D frontier 规则策略已阶段性收口，不再继续深挖策略调参。
- 更激进或更保守的风格仍可通过 YAML 权重表达，但后续重点转向数据记录和 ranker 学习。
- 保留小边界完备性仍依赖较低 `min_frontier_cluster_size`、small cluster 延后和 BT 失败重选。

当前实现中，小 cluster 不会被直接丢弃；当存在正常候选时，小 cluster 会延后选择，只有没有正常候选时才作为兜底目标。探索末期若残留单格 unknown，后续更适合通过独立 cleanup exploration 或 ML ranker 处理，而不是继续在主策略中追加规则。

## Nav2 控制器

当前探索配置使用 RPP：

```yaml
FollowPath:
  plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
```

阶段性结论：

- RPP 在当前探索场景下更稳定，能更自然地对齐 path 并前进。
- DWB 可用作对比，但在窄边界、贴近未知区域和频繁重规划时更容易抖动或短暂停滞。
- 若看到 path 轻微贴近灰色区域，优先确认它是 unknown 还是 inflation/cost 区；`allow_unknown: false` 只禁止穿真正 unknown cell。

## 调试建议

### frontier 是否正常产生

```bash
ros2 topic echo /exploration_state
```

节点日志中会周期性输出 frontier cell 和 raw cluster 数量。

### Nav2 是否进入 recovery

```bash
ros2 topic echo /behavior_tree_log
```

重点观察：

- `ComputePathToPose`
- `FollowPath`
- `RecoveryActions`
- `Wait`

### 速度链路

当前主速度链路：

```text
controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> turtlebot3_diff_drive
```

检查命令：

```bash
ros2 topic info /cmd_vel_nav -v
ros2 topic info /cmd_vel -v
ros2 topic echo /cmd_vel
```

## 当前阶段限制

- 当前 2D frontier exploration 可以作为阶段性规则 baseline，但不再继续追加复杂策略规则。
- 候选策略链已经较长，继续维护退避、环形采样、unknown ratio、goal inset、path safety、blacklist 等规则组合成本较高。
- RViz 中 global path 在选点阶段可能短暂跳动，通常来自候选可执行性检查连续调用 planner 产生的临时 path。
- 探索末期仍可能残留单格 unknown，需要后续 cleanup exploration 或数据驱动排序解决。
- 真实机器人部署前还需要重新标定 footprint、inflation、速度限制和传感器噪声参数。

## 后续演进方向

后续重点从“继续手写策略”转向“记录数据 + 学习排序 + BT 执行恢复”：

- 记录每次 frontier 决策数据：候选几何、score 分项、costmap 特征、path 长度、footprint/path safety、是否成功、实际 gain。
- 训练 success / gain ranker，让 ML 负责候选排序。
- 规则层只保留硬安全约束：碰撞、unknown、越界、不可执行路径、blacklist。
- BT 继续负责任务执行、恢复、失败标记、重试、完成判断。
- 架构层继续收口接口和数据结构，为后续策略树和学习式 ranker 接入做准备。

## 参考文档

- [Frontier Explorer 详细设计](src/frontier_explorer/doc/frontier_explorer_node_doc.md)
- [Frontier Explorer README](src/frontier_explorer/README.md)
- [Exploration 架构说明](src/frontier_explorer/doc/exploration_architecture.md)
- [Exploration BT 设计](src/frontier_explorer/doc/exploration_bt_design.md)
- [顶层变更记录](CHANGELOG.rst)
