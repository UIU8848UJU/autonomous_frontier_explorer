# frontier_explorer

[![Architecture](https://img.shields.io/badge/Architecture-BT--Ready%20Exploration-blue)](doc/exploration_architecture.md)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-green)](https://docs.ros.org/en/humble/)
[![Nav2](https://img.shields.io/badge/Nav2-Integrated-orange)](doc/exploration_architecture.md#navigationnode-职责)

`frontier_explorer` 是面向 ROS 2 Humble / Nav2 的 frontier exploration 包。当前版本已经从单体探索节点重构为“能力节点 + BehaviorTree.CPP 编排 + NavigationNode 中间层”的结构，便于后续扩展商业化探索流程、地图保存、返航和任务管理。

## 当前架构

```text
TaskManagerNode
  -> ExplorationBtOrchestratorNode
      -> BehaviorTree.CPP XML + BT plugins
          -> FrontierExplorerNode / FrontierGoalProvider
          -> NavigationNode
              -> Nav2 ComputePathToPose
              -> Nav2 NavigateToPose
```

核心边界：

- `FrontierExplorerNode`：只提供 frontier 候选/目标生成、失败标记、blacklist 清理、marker 和 state。
- `ExplorationBtOrchestratorNode`：唯一探索编排入口，负责按 BT 请求候选、选择可执行目标、导航、失败重选、完成判断。
- `NavigationNode`：导航能力中间层，负责 footprint 落脚检查、path safety 检查和 Nav2 `NavigateToPose` 桥接。
- `FrontierGoalProvider`：纯 C++ 能力类，复用 detector / pruner / scorer / selector，不直接发送导航 goal。

完整架构图见 [探索架构说明](doc/exploration_architecture.md)。

## 主要能力

- 基于 `/map` 检测 frontier cell 并聚类。
- 基于 retry / blacklist / cluster blacklist 管理失败目标。
- 基于 distance、cluster size、clearance、unknown risk、retry penalty 等组件化分数排序候选。
- 支持候选回退策略：centroid fallback、向机器人方向退避、环形采样、候选 yaw 朝向 frontier centroid。
- 使用 `/global_costmap/costmap` 对候选落脚点做安全硬约束，避免退避/采样候选落入障碍或 inflation 区。
- 通过 NavigationNode 做 goal feasibility 检查：footprint 落脚碰撞 + Nav2 `ComputePathToPose` + path unknown/high-cost 审计。
- 通过 BehaviorTree.CPP 插件库加载探索 BT 节点。
- 发布 raw / candidate / scored / selected / blacklist / rejected marker，支持 RViz 调试。

## 关键接口

Frontier 能力节点：

```text
/frontier_explorer_node/get_frontier_candidates
/frontier_explorer_node/get_next_frontier_goal
/frontier_explorer_node/mark_frontier_failed
/frontier_explorer_node/clear_frontier_blacklist
/frontier_explorer_node/get_exploration_state
```

导航能力节点：

```text
/navigation_node/check_goal_feasibility
/navigation_node/check_pose_reachability
/navigation_node/navigate_to_pose
```

BT 编排节点：

```text
/exploration_bt_orchestrator_node/start_exploration
/exploration_bt_orchestrator_node/stop_exploration
/exploration_bt_orchestrator_node/pause_exploration
/exploration_bt_orchestrator_node/resume_exploration
```

状态和可视化：

```text
/frontier_explorer/state
/exploration_orchestrator/state
/exploration_state
/frontier/raw_markers
/frontier/candidate_markers
/frontier/scored_markers
/frontier/selected_marker
/frontier/blacklist_markers
/frontier/rejected_markers
```

## 启动

构建探索相关包：

```bash
./build.sh exploration
```

完整系统由 bringup 启动：

```bash
ros2 launch autonomousr_explorer_bringup full_system.launch.py
```

启动探索：

```bash
ros2 service call /exploration_bt_orchestrator_node/start_exploration std_srvs/srv/Trigger {}
```

停止探索：

```bash
ros2 service call /exploration_bt_orchestrator_node/stop_exploration std_srvs/srv/Trigger {}
```

清理 frontier blacklist：

```bash
ros2 service call /frontier_explorer_node/clear_frontier_blacklist robot_interfaces/srv/ClearFrontierBlacklist {}
```

## 当前已知现象

- RViz 中 global path 在选点阶段可能短时间跳动。这通常来自 `SelectFeasibleFrontier` 对多个候选调用 `ComputePathToPose` 做可执行性检查，planner 临时 path 被 RViz 显示出来；不一定代表真正 `NavigateToPose` goal 被反复发送。
- 探索末期一格左右的 unknown 残留可能被当前保守策略过滤。原因通常是 `min_frontier_cluster_size`、small cluster 延后、unknown ratio、goal inset、footprint/path safety 共同作用。后续更适合用“收尾模式”解决，而不是直接放宽主策略。
- 当前候选策略已经可运行，但末期策略链较复杂。后续需要把 normal exploration 和 cleanup exploration 明确拆成两个策略阶段，降低调参互相影响。
- 可执行性检查依赖 Nav2 planner / global costmap 时序；启动初期 costmap 或 planner 未 ready 时会出现 recoverable wait。

## 文档

- [探索架构说明](doc/exploration_architecture.md)
- [Behavior Tree 设计](doc/exploration_bt_design.md)
- [FrontierExplorerNode 技术文档](doc/frontier_explorer_node_doc.md)
- [更新日志](CHANGELOG.rst)

