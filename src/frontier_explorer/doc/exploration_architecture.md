# Exploration 架构说明

## 目标

`frontier_explorer` 当前采用“能力节点 + BT 编排节点”的结构：

- `FrontierExplorerNode` 是 ROS wrapper，只负责 frontier 能力接口、订阅、marker 和 state。
- `FrontierGoalProvider` 是纯 C++ 能力类，负责检测、过滤、打分、选择和 retry / blacklist 策略状态。
- `ExplorationBtOrchestratorNode` 是唯一探索决策编排层。
- retry / blacklist 仍由 frontier 能力层内部维护，对外只暴露失败事件接口。
- Nav2 `NavigateToPose` 暂由 BT 节点直接调用，后续可替换为独立 `NavigationNode`。

## 运行链路

```text
TaskManagerNode
  -> /exploration_bt_orchestrator_node/start_exploration
  -> ExplorationBtOrchestratorNode
  -> behavior_trees/exploration_tree.xml
  -> ComputeNextFrontierGoal
      -> /frontier_explorer_node/get_next_frontier_goal
  -> NavigateToFrontier
      -> Nav2 NavigateToPose
  -> MarkFrontierFailed
      -> /frontier_explorer_node/mark_frontier_failed
```

探索结束由 `GetNextFrontierGoal` 的 `exploration_complete` 字段驱动。

## 边界规则

- `FrontierExplorerNode` 只暴露 frontier 能力接口，不决定探索流程。
- `FrontierGoalProvider` 不依赖 action client，不发送 goal，不知道 BT tick 节奏。
- `ExplorationBtOrchestratorNode` / BT 负责什么时候请求 goal、什么时候导航、失败后什么时候重新选点、什么时候完成或失败。
- BT 不直接读取或修改 retry / blacklist 内部结构；它只通过 `mark_frontier_failed` 通知一次导航失败事件。
- retry 计数、blacklist 阈值、goal blacklist、cluster blacklist、clear 规则均由 `FrontierGoalProvider` 内部维护。
- `NavigationNode` / Nav2 负责真正的 `NavigateToPose`。当前 `NavigateToFrontier` 直接调用 Nav2 action，后续可替换为独立 `NavigationNode`。

## FrontierExplorerNode 职责

`FrontierExplorerNode` 是探索能力 ROS wrapper，负责：

- 订阅 `/map`。
- 可选订阅 `/global_costmap/costmap` 作为 clearance / safety scoring 输入。
- 通过 TF 查询 `global_frame <- robot_base_frame`，为能力层提供 map frame 下的机器人位姿输入。
- 托管 `FrontierGoalProvider`。
- 提供 frontier 能力服务。
- 发布 frontier marker。
- 发布 `/frontier_explorer/state`。
- 兼容发布旧 `/exploration_state`。

`enable_internal_navigation_loop` 已废弃并被忽略。节点不会主动发送 Nav2 goal。
默认 `show_all_candidate_markers: false`，RViz 中只显示最终选中候选球；需要调试完整候选集合时可改为 `true`。

## FrontierGoalProvider 职责

`FrontierGoalProvider` 是可单测的 C++ 能力类，负责：

- 维护 `CostmapAdapter`。
- 复用现有 `FrontierDetector`、`FrontierPruner`、`FrontierScorer`、`FrontierSelector`。
- 计算下一个 frontier goal。
- 接收 map frame 下的机器人位姿，不直接订阅 `/odom`，避免把漂移的 odom frame 当作 map frame 使用。
- 可选通过注入的 reachability checker 过滤 Nav2 planner 不可达候选；provider 只依赖抽象接口，不持有 ROS action client。
- 维护 retry / blacklist 的内部数据结构和更新规则。
- 返回 marker 发布所需的可视化快照。
- 不创建 ROS service、topic、timer 或 action client。

## ExplorationBtOrchestratorNode 职责

`ExplorationBtOrchestratorNode` 是唯一探索编排节点，负责：

- 加载可配置 BT XML。
- 注册当前进程内置 BT 节点。
- 提供 start / stop / pause / resume 服务。
- 周期 tick BehaviorTree。
- 调用 frontier goal 服务。
- 调用 Nav2 `NavigateToPose` action。
- 导航失败时调用 `mark_frontier_failed`，但不直接操作 retry / blacklist。
- 根据 BT 结果决定继续请求 frontier、完成探索或进入失败状态。
- 发布 `/exploration_orchestrator/state`。

普通 C++ 状态机版 orchestrator 已删除，避免多套编排逻辑并存。

## 服务接口

FrontierExplorerNode：

- `/frontier_explorer_node/get_next_frontier_goal`
- `/frontier_explorer_node/mark_frontier_failed`
- `/frontier_explorer_node/clear_frontier_blacklist`
- `/frontier_explorer_node/get_exploration_state`

ExplorationBtOrchestratorNode：

- `/exploration_bt_orchestrator_node/start_exploration`
- `/exploration_bt_orchestrator_node/stop_exploration`
- `/exploration_bt_orchestrator_node/pause_exploration`
- `/exploration_bt_orchestrator_node/resume_exploration`

旧的 `/start_exploration` 和 `/stop_exploration` 仍由 `FrontierExplorerNode` 保留，仅用于兼容旧控制面。
新系统入口应使用 `exploration_bt_orchestrator_node`。

## 状态 topic

- `/frontier_explorer/state`: frontier 能力节点状态。
- `/exploration_orchestrator/state`: BT 编排流程状态。
- `/exploration_state`: 兼容旧 TaskManager / MapManager 的状态 topic。

## 参数约定

服务名和 action 名属于运行时拓扑配置，不通过 CMake 写死。默认值集中在：

```text
include/frontier_explorer/nodes/exploration_bt_defaults.hpp
```

生产部署应通过 YAML 覆盖：

```yaml
exploration_bt_orchestrator_node:
  ros__parameters:
    frontier_goal_service: /frontier_explorer_node/get_next_frontier_goal
    mark_failed_service: /frontier_explorer_node/mark_frontier_failed
    navigate_to_pose_action: navigate_to_pose
    tick_period_sec: 0.1
    service_retry_delay_sec: 2.0
```

## 启动

单独启动 frontier 能力节点：

```bash
ros2 launch frontier_explorer frontier_explorer.launch.py
```

单独启动 BT 编排节点：

```bash
ros2 launch frontier_explorer exploration_bt_orchestrator.launch.py
```

启动探索：

```bash
ros2 service call /exploration_bt_orchestrator_node/start_exploration std_srvs/srv/Trigger {}
```

停止探索：

```bash
ros2 service call /exploration_bt_orchestrator_node/stop_exploration std_srvs/srv/Trigger {}
```

## 目录结构

```text
include/frontier_explorer/nodes/
  exploration_bt_context.hpp
  exploration_bt_defaults.hpp
  exploration_bt_orchestrator_node.hpp
  bt/
    action/
      compute_next_frontier_goal_action.hpp
      mark_frontier_failed_action.hpp
      navigate_to_frontier_action.hpp
    is_exploration_complete_condition.hpp

src/nodes/
  frontier_explorer_main.cpp
  frontier_explorer_node.cpp
  bt/
    exploration_bt_context.cpp
    orchestrator/
      exploration_bt_orchestrator_main.cpp
      exploration_bt_orchestrator_node.cpp
    action/
      compute_next_frontier_goal_action.cpp
      mark_frontier_failed_action.cpp
      navigate_to_frontier_action.cpp
    is_exploration_complete_condition.cpp

include/frontier_explorer/core/
  frontier_goal_provider.hpp

src/core/
  frontier_goal_provider.cpp
```

## 后续扩展

1. 抽出独立 `NavigationNode`，由 `NavigateToFrontier` 调用。
2. 将进程内 BT 节点拆成 BehaviorTree.CPP plugin。
3. 增加 `SaveMap`、`ReturnHome`、`ClearFrontierBlacklist` BT 节点。
4. 为 BT 节点增加单元测试或 launch smoke test。
5. 将当前内置 Nav2 planner 可达性检查拆成独立 `NavigationNode` 或 BT plugin。
