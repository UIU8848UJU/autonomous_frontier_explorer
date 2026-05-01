# Exploration 架构说明

## 目标

`frontier_explorer` 当前采用“能力节点 + BT 编排节点”的结构：

- `FrontierExplorerNode` 只负责 frontier 目标生成能力。
- `ExplorationBtOrchestratorNode` 是唯一探索决策编排层。
- retry / blacklist 仍由 `FrontierExplorerNode` 内部维护。
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

## FrontierExplorerNode 职责

`FrontierExplorerNode` 是探索能力节点，负责：

- 订阅 `/map`。
- 可选订阅 `/global_costmap/costmap` 作为 clearance / safety scoring 输入。
- 维护 `CostmapAdapter`。
- 复用现有 `FrontierDetector`、`FrontierPruner`、`FrontierScorer`、`FrontierSelector`。
- 计算下一个 frontier goal。
- 维护 retry / blacklist。
- 发布 frontier marker。
- 发布 `/frontier_explorer/state`。
- 兼容发布旧 `/exploration_state`。

默认参数 `enable_internal_navigation_loop: false`。此时节点不会主动发送 Nav2 goal。

## ExplorationBtOrchestratorNode 职责

`ExplorationBtOrchestratorNode` 是唯一探索编排节点，负责：

- 加载可配置 BT XML。
- 注册当前进程内置 BT 节点。
- 提供 start / stop / pause / resume 服务。
- 周期 tick BehaviorTree。
- 调用 frontier goal 服务。
- 调用 Nav2 `NavigateToPose` action。
- 导航失败时调用 `mark_frontier_failed`。
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
```

## 后续扩展

1. 抽出独立 `NavigationNode`，由 `NavigateToFrontier` 调用。
2. 将进程内 BT 节点拆成 BehaviorTree.CPP plugin。
3. 增加 `SaveMap`、`ReturnHome`、`ClearFrontierBlacklist` BT 节点。
4. 为 BT 节点增加单元测试或 launch smoke test。
