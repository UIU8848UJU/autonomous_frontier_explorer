# Exploration Behavior Tree 设计

## 当前实现

`exploration_bt_orchestrator_node` 是当前唯一探索编排节点，默认加载：

```text
behavior_trees/exploration_tree.xml
```

当前 BT 节点在 orchestrator 进程内注册，暂未做成动态 plugin。这样做的原因是当前阶段优先保证：

- 服务/action 边界稳定。
- 编排逻辑可配置。
- 构建和部署简单。
- 后续可以平滑拆为 BehaviorTree.CPP plugin。

## 当前 XML

```xml
<root main_tree_to_execute="ExplorationTree">
  <BehaviorTree ID="ExplorationTree">
    <Fallback name="ExploreOrComplete">
      <IsExplorationComplete/>
      <Sequence name="ExploreOneFrontier">
        <ComputeNextFrontierGoal/>
        <Fallback name="NavigateOrBlacklist">
          <NavigateToFrontier/>
          <MarkFrontierFailed/>
        </Fallback>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
```

语义：

- `IsExplorationComplete`：如果上下文已标记探索完成，返回 `SUCCESS`。
- `ComputeNextFrontierGoal`：调用 frontier 能力服务，请求下一个目标。
- `NavigateToFrontier`：将目标发送给 Nav2 `NavigateToPose`。
- `MarkFrontierFailed`：导航失败后通知 frontier 能力节点更新 retry / blacklist。

orchestrator 每次 BT 返回 `SUCCESS` 后会重新加载树并进入下一轮 frontier 请求。
当 `ComputeNextFrontierGoal` 返回 `exploration_complete=true` 时，orchestrator 结束探索流程。

## BT 节点映射

| BT 节点 | 类型 | 依赖接口 | 职责 |
| --- | --- | --- | --- |
| `ComputeNextFrontierGoal` | StatefulAction | `robot_interfaces/srv/GetNextFrontierGoal` | 请求下一个 frontier goal |
| `NavigateToFrontier` | StatefulAction | `nav2_msgs/action/NavigateToPose` | 导航到当前 frontier goal |
| `MarkFrontierFailed` | StatefulAction | `robot_interfaces/srv/MarkFrontierFailed` | 记录导航失败并触发 retry / blacklist |
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

上下文内部通过 mutex 保护跨 callback 访问。

## 参数

默认值集中在：

```text
include/frontier_explorer/nodes/exploration_bt_defaults.hpp
```

生产部署通过 YAML 覆盖：

```yaml
exploration_bt_orchestrator_node:
  ros__parameters:
    frontier_goal_service: /frontier_explorer_node/get_next_frontier_goal
    mark_failed_service: /frontier_explorer_node/mark_frontier_failed
    navigate_to_pose_action: navigate_to_pose
    tick_period_sec: 0.1
    service_retry_delay_sec: 2.0
```

## 后续 plugin 化方向

后续将当前进程内注册节点拆成动态 plugin：

- `ComputeNextFrontierGoal`
- `NavigateToFrontier`
- `MarkFrontierFailed`
- `IsExplorationComplete`
- `ClearFrontierBlacklist`
- `SaveMap`
- `ReturnHome`

plugin 化后，`exploration_bt_orchestrator_node` 只负责：

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
