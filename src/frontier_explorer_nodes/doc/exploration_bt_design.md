# Exploration Behavior Tree 设计

## 当前实现

`exploration_bt_orchestrator_node` 是当前唯一探索编排节点，默认加载：

```text
behavior_trees/exploration_tree.xml
```

当前 BT 节点已拆成 BehaviorTree.CPP 动态 plugin，默认插件库：

```text
lib/libfrontier_explorer_bt_nodes.so
```

`exploration_bt_orchestrator_node` 不再手写注册具体 BT 节点，只负责加载插件库、加载 XML、注入共享上下文、tick tree 和发布流程状态。

## 当前 XML

```xml
<root main_tree_to_execute="ExplorationTree">
  <BehaviorTree ID="ExplorationTree">
    <Fallback name="ExploreOrComplete">
      <IsExplorationComplete/>
      <Sequence name="ExploreOneFrontier">
        <ComputeFrontierCandidates/>
        <Fallback name="SelectNavigateOrBlacklist">
          <Sequence name="SelectAndNavigate">
            <SelectFeasibleFrontier/>
            <NavigateToFrontier/>
          </Sequence>
          <MarkFrontierFailed/>
        </Fallback>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

语义：

- `IsExplorationComplete`：如果上下文已标记探索完成，返回 `SUCCESS`。
- `ComputeFrontierCandidates`：调用 frontier 能力服务，请求按分数排序的候选目标列表。
- `SelectFeasibleFrontier`：逐个调用 NavigationNode 可执行性服务，在检查窗口内选择综合 frontier score 和 path length 后最优的可执行候选。
- `NavigateToFrontier`：将目标发送给 NavigationNode `NavigateToPose`。
- `MarkFrontierFailed`：导航失败后通知 frontier 能力节点处理失败事件。

orchestrator 每次 BT 返回 `SUCCESS` 后会重新加载树并进入下一轮 frontier 请求。
当 `ComputeFrontierCandidates` 返回 `exploration_complete=true` 时，orchestrator 结束探索流程。

## 边界约束

BT 负责流程编排：请求 frontier、导航、失败后触发失败通知、重新选点、完成或失败。
BT 不直接维护 retry / blacklist。`retry_count`、`blacklisted`、goal blacklist、cluster blacklist 等策略状态只在 `FrontierGoalProvider` / `FrontierSelector` 内部更新。

## BT 节点映射

| BT 节点 | 类型 | 依赖接口 | 职责 |
| --- | --- | --- | --- |
| `ComputeNextFrontierGoal` | StatefulAction | `robot_interfaces/srv/GetNextFrontierGoal` | 请求下一个 frontier goal |
| `ComputeFrontierCandidates` | StatefulAction | `robot_interfaces/srv/GetFrontierCandidates` | 请求 frontier 候选列表 |
| `SelectReachableFrontier` | StatefulAction | `robot_interfaces/srv/CheckPoseReachability` | 兼容旧 XML，从候选列表中选择第一个 planner 可达目标 |
| `SelectFeasibleFrontier` | StatefulAction | `robot_interfaces/srv/CheckGoalFeasibility` | 在候选窗口内选择综合 score 和 path length 最优的可执行目标 |
| `NavigateToFrontier` | StatefulAction | `robot_interfaces/action/NavigateToPose` | 导航到当前 frontier goal |
| `MarkFrontierFailed` | StatefulAction | `robot_interfaces/srv/MarkFrontierFailed` | 通知 frontier 能力节点处理导航失败事件 |
| `IsExplorationComplete` | Condition | BT context | 判断探索是否完成 |

## BT 共享上下文

`ExplorationBtContext` 保存当前 BT 执行期间共享的运行状态：

- ROS node 指针。
- service client。
- Nav2 action client。
- 当前 frontier goal。
- exploration complete 标志。
- stop requested 标志。
- 状态 detail 文本。
- 候选列表和当前已提交导航 goal。

上下文内部通过 mutex 保护跨 callback 访问。

插件节点通过 BehaviorTree.CPP blackboard 获取共享上下文，blackboard key 为：

```text
exploration_bt_context
```

这个设计让 BT 节点可以动态加载，同时避免每个插件节点直接创建 ROS client 或复制流程状态。

## 参数

默认值集中在：

```text
include/frontier_explorer_nodes/nodes/exploration_bt_defaults.hpp
```

生产部署通过 YAML 覆盖：

```yaml
exploration_bt_orchestrator_node:
  ros__parameters:
    frontier_goal_service: /frontier_explorer_node/get_next_frontier_goal
    frontier_candidates_service: /frontier_explorer_node/get_frontier_candidates
    mark_failed_service: /frontier_explorer_node/mark_frontier_failed
    reachability_service: /navigation_node/check_pose_reachability
    goal_feasibility_service: /navigation_node/check_goal_feasibility
    navigation_action: /navigation_node/navigate_to_pose
    max_frontier_candidates: 8
    max_feasibility_recoverable_retries: 2
    feasible_path_length_weight: 0.6
    bt_plugin_libraries:
      - /home/xxx/mk_nav2/install/frontier_explorer_nodes/lib/libfrontier_explorer_bt_nodes.so
    tick_period_sec: 0.1
    service_retry_delay_sec: 2.0

navigation_node:
  ros__parameters:
    navigation_action: ~/navigate_to_pose
    check_pose_reachability_service: ~/check_pose_reachability
    check_goal_feasibility_service: ~/check_goal_feasibility
    navigate_to_pose_action: navigate_to_pose
    compute_path_to_pose_action: compute_path_to_pose
    footprint_costmap_topic: /global_costmap/costmap
    enable_footprint_collision_check: true
    allow_unknown_footprint: false
    enable_path_safety_check: true
    allow_unknown_path: false
    robot_radius: 0.1
    footprint_padding: 0.0
    footprint_cost_threshold: 253
    path_cost_threshold: 253
    reachability_planner_id: ""
    nav2_server_timeout_ms: 1000
    reachability_timeout_ms: 2000
```

`bt_plugin_libraries` 支持多插件库加载，商业部署中可以把不同策略节点拆到独立库中，通过 YAML 切换。

## 当前 BT 插件

当前插件库导出：

| 插件节点 | 类型 | 说明 |
| --- | --- | --- |
| `ComputeNextFrontierGoal` | StatefulAction | 兼容旧 XML，请求 frontier 能力节点返回下一个 goal |
| `ComputeFrontierCandidates` | StatefulAction | 请求 frontier 能力节点返回候选列表 |
| `SelectReachableFrontier` | StatefulAction | 兼容旧 XML，调用 NavigationNode 可达性服务选择可达候选 |
| `SelectFeasibleFrontier` | StatefulAction | 调用 NavigationNode 可执行性服务选择可执行候选 |
| `NavigateToFrontier` | StatefulAction | 调用 NavigationNode `NavigateToPose` 导航到当前 goal |
| `MarkFrontierFailed` | StatefulAction | 导航失败后通知 frontier 能力节点 |
| `IsExplorationComplete` | Condition | 判断共享上下文中的完成标志 |

## 后续插件扩展方向

后续可继续增加：

- `ClearFrontierBlacklist`
- `SaveMap`
- `ReturnHome`
- `CheckMappingReady`
- `EnterCleanupExploration`
- `PublishFeasibilityDebugPath`

扩展后，`exploration_bt_orchestrator_node` 仍只负责：

- 加载 plugin library。
- 加载 BT XML。
- 提供 start / stop 服务。
- tick tree。
- 发布状态。

## 后续 XML 示例

```xml
<root BTCPP_format="4">
  <BehaviorTree ID="ExplorationTree">
    <ReactiveSequence>
      <CheckMappingReady/>
      <RetryUntilSuccessful num_attempts="3">
        <ComputeNextFrontierGoal/>
      </RetryUntilSuccessful>
      <Fallback>
        <NavigateToFrontier/>
        <Sequence>
          <MarkFrontierFailed/>
          <ForceFailure/>
        </Sequence>
      </Fallback>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

## 当前已知现象

`SelectFeasibleFrontier` 会对多个候选连续调用 `CheckGoalFeasibility`，而该服务内部会调用 Nav2 `ComputePathToPose`。如果 RViz 显示的是 planner server 的全局路径 topic，选点阶段可能看到 path 在 0.25 秒左右短暂跳到不同候选目标。

这属于候选评估路径和真正导航路径在可视化层混在一起的问题。后续建议将 feasibility check 的 debug path 单独发布到 `/frontier/debug/feasibility_path`，并默认关闭或用低透明度显示；真正导航 path 只显示 NavigationNode 已接受的 active goal。
